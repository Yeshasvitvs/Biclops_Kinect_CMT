#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#endif

#include <math.h>       // for sqrt()
#include <iostream>
#include <fstream>      // for ifstream
using namespace std;
#include <string.h>  // for strcpy, strcmp

#include "PMDUtils.h"   // access to file parsing
#include "PMDCollections.h"
#include "PMDGet.h"
#include "Biclops.h"    // class being implemented here

//----------------------------------------------------------------------------
Biclops::Biclops() {
    for (int i=Pan;i<=Verge;i++) axes[i] = NULL;
    allAxesOnOneController = true;
    hasBeenInitialized = false;
    rollCenter = pitchCenter = 512;
}

//----------------------------------------------------------------------------
PMDAxisControl *Biclops::GetAxis(int axis) {
    if (axes[axis] == NULL) {
        const char *AxisUniqueIDLUT[4] = {"Pan", "Tilt", "Roll", "Verge"};
        GetAxis(AxisUniqueIDLUT[axis]);
    }
    return axes[axis];
}

//----------------------------------------------------------------------------
PMDAxisControl *Biclops::GetAxis(const char *axis) {
    AxisCollection *axisCollection = PMDGet::GetAxes();
    PMDAxisControl *axisP = (*axisCollection)[axis];
    if (axisP != NULL) {
        // Map the axis name to a numeric index (yeah, this is bad, BTW).
        int axisNum = 0;
        if (strcmp(axis,"Tilt") == 0) axisNum = 1;
        else if (strcmp(axis,"Roll") == 0) axisNum = 2;
        else if (strcmp(axis,"Verge") == 0) axisNum = 3;
        axes[axisNum] = axisP;
    }
    return axisP;
}

//----------------------------------------------------------------------------
bool Biclops::Initialize(const char *fileName) {

    if (!hasBeenInitialized) {
        hasBeenInitialized = PMDUtils::Initialize(fileName, true, false);

        // Get shortcuts to all the axes
        int i;
        for (i=Pan;i<=Verge;i++) GetAxis(i);

        // Determine if all the axes are on the same controller. 
        // This result is used by the Move() command to know whether a MoveMultiple
        // is possible.
        if (hasBeenInitialized) { 
            if (axes[Pan]==NULL) {
                std::cerr << "ERROR! Pan axis not defined\n";
                return false;
            }
            PMDController *controller = axes[Pan]->GetController();
            if (!controller->ConfigureController()) {
                std::cerr << "ERROR! Unable to configure Pan\n";
                return false;
            }
            for (i=Tilt;i<=Verge;i++)
                if (axes[i] != NULL) {
                    PMDController *controller2 = axes[i]->GetController();
                    if (controller2 != controller) {
                        if (!controller2->ConfigureController()) {
                            std::cerr << "ERROR! axis not defined\n";
                            return false;
                        }
                        allAxesOnOneController = false;
                    }
                }
        }
    }
    return hasBeenInitialized;
}

//----------------------------------------------------------------------------
bool Biclops::HomeAxes(int axisMask, bool forceHoming) {

    bool panStat = true,tiltStat = true, rollStat = true, vergeStat = true;

    // Homing order is always tilt, roll, verge, pan
    // The logic for each axis is: the axis must be requested to be homed
    // and the axis must be controllable (chipset has that axis) and then
    // find home if forced to or the axis is not already homed.
    if (axisMask & RollMask && axes[Roll] != NULL)
        rollStat  = axes[Roll]->FindHome(forceHoming);
    if (axisMask & TiltMask && axes[Tilt] != NULL)
        tiltStat  = axes[Tilt]->FindHome(forceHoming);
    if (axisMask & VergeMask && axes[Verge] != NULL)
        vergeStat = axes[Verge]->FindHome(forceHoming);
    if (axisMask & PanMask && axes[Pan] != NULL)
        panStat  = axes[Pan]->FindHome(forceHoming);

    return (panStat && tiltStat && rollStat && vergeStat);

}

//----------------------------------------------------------------------------
bool Biclops::Move(int axisMask, int32_t waitPeriod) {
    PMDAxis axis;

    if (allAxesOnOneController) {

        // To utilize the MultiUpdate command, we essentially have to replicate
        // the PMDAxisControl::Move() method logic here for making sure the
        // axis is in the correct state for moving. 
        for (axis = PMDAxis1; axis <= PMDAxis4; axis++) {
            if (axisMask & 1<<axis) {
                axes[axis]->PreMove();
//                PMDuint16 eventStatus = axes[axis]->GetLastEventStatus();
//                stat = axes[axis]->ResetEventStatus(PMDAxisControl::MoveEventMask);
//                if (eventStatus & PMDEventMotionErrorMask)
//                    stat = axes[axis]->SetMotorMode(PMDMotorOn);
            }
        }

        // Now the axes motion can begin.
        GetAxis(Pan)->MultiUpdate(axisMask);

        for (axis = PMDAxis1; axis <= PMDAxis4; axis++)
            if (axisMask & 1<<axis)
                axes[axis]->PostMove();

    } else {

        // Since axes are on more than one controller, just issue the move
        // command on each axis with no wait time and let the follow-on loop
        // do the actual waiting.
        
        // But first, the event status must be cleared if this multi-axis move
        // wants to wait for motion completion.
        if (waitPeriod > 0)
            for (axis = PMDAxis1; axis <= PMDAxis4; axis++)
                if (axisMask & 1<<axis)
                    axes[axis]->ResetEventStatus(PMDAxisControl::MoveEventMask);

        // Okay, now we can command the moves.
        for (axis = PMDAxis1; axis <= PMDAxis4; axis++) 
            if (axisMask & 1<<axis) axes[axis]->Move(false);

    }


    // Wait for all motion to complete if desired.
    if (waitPeriod > 0) {
        for (axis = PMDAxis1; axis <= PMDAxis4; axis++) {
            if (axisMask & 1<<axis) {
                while (!GetAxis(axis)->IsMotionDone()) {
                    PMDUtils::DoSleep(waitPeriod);
                }
            }
        }
    }
    return true;
}

//----------------------------------------------------------------------------
void Biclops::SetLevelBaseAttitude() {
    const PMDuint16 pitchAxisAnalogChannel = 6;
    const PMDuint16 rollAxisAnalogChannel = 7;

    if (axes[0] != NULL) {
        axes[0]->ReadAnalog(pitchAxisAnalogChannel,pitchCenter);
        axes[0]->ReadAnalog(rollAxisAnalogChannel,rollCenter);
        cout << "level attitude " << (pitchCenter >> 6) << " " << (rollCenter >> 6) << endl;
    }
}

//----------------------------------------------------------------------------
void Biclops::GetBaseAttitude(double &pitch, double &roll) {
    const PMDuint16 pitchAxisAnalogChannel = 6;
    const PMDuint16 rollAxisAnalogChannel = 7;

    PMDuint16 analogValue;
    PMDuint32 avSum;

    if (axes[0] != NULL) {
        int i;
        for (i = avSum = 0; i<5;i++) {
            axes[0]->ReadAnalog(rollAxisAnalogChannel,analogValue);
            avSum += analogValue;
        } analogValue = avSum/5;
        roll = AnalogToRevolutions(analogValue,rollCenter);

        for (i = avSum = 0; i<5;i++) {
            axes[0]->ReadAnalog(pitchAxisAnalogChannel,analogValue);
            avSum += analogValue;
        } analogValue = avSum/5;
        pitch = AnalogToRevolutions(analogValue,pitchCenter);
    } else {
        roll = pitch = 0.0;
    }

}

//----------------------------------------------------------------------------
double Biclops::AnalogToRevolutions(PMDuint16 analogValue, PMDuint16 centerValue) {

    const int PMDAnalogBitShift = 6;
//    const int A2DBits = 10;
//    const double A2DMinVoltage = 0;
//    const double A2DMaxVoltage = 3.3;
//    const double A2DSensitivity = (A2DMaxVoltage - A2DMinVoltage)/((1<<10) - 1);
//    const double ZeroGDCBias = A2DMaxVoltage/2; // Sensor zero-g value is half
                                                // of supply voltage.
//    const double MMA6270QSensitivity = 0.8; // units in volts/g for 1.5g setting.
//    const double GeesPerBit = 250;  //+-250 counts = +-800mV

    // Shift the PMD analog values and make them signed.
    int scaledAnalogValue = analogValue >> PMDAnalogBitShift;
    int scaledCenterValue = centerValue >> PMDAnalogBitShift;
    int centerRelativeValue = scaledAnalogValue - scaledCenterValue;

    // Translate the analog value from the digital reading into g-force.
    double gees = (double) (PMDUtils::Crop(centerRelativeValue,centerValue-250,centerValue+250))/250;

    cout << " ana = "; cout.width(3);     cout <<            showpos << centerRelativeValue
         << " g's = "; cout.precision(3); cout << fixed   << showpos << gees
         << " deg = "; cout.precision(3); cout << fixed   << showpos << PMDUtils::RadsToDegs(asin(gees));

    // Convert g's into an angle and return the value in revolutions.
    return PMDUtils::RadsToRevs(asin(gees));
}

//----------------------------------------------------------------------------
bool Biclops::AnAxisIsPowered() {
    bool anAxisIsPowered = false;
    for (PMDAxis axis = PMDAxis1; axis <= PMDAxis4; axis++) {
        if (axes[axis] != NULL)
            anAxisIsPowered = anAxisIsPowered || axes[axis]->IsAmpEnabled();
    }
    return anAxisIsPowered;
}

//----------------------------------------------------------------------------
int Biclops::GetDebugLevel() {
    int level;
    if (axes[0] != NULL)
        level = axes[0]->GetController()->GetDebugLevel();
    else
        level = 0;
    return level;
}
//----------------------------------------------------------------------------
void Biclops::SetDebugLevel(int level) {

    for (int axis = Pan; axis <=Verge; axis++)
        if (axes[axis] != NULL)
            axes[axis]->GetController()->SetDebugLevel(level);
}
