#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#else
#include <stdlib.h>
#endif

#include <sstream>
#include <fstream>          // for homing file reading

using namespace std;

#include <math.h>           // for pow() 
#include <limits.h>	        // for INT_MIN, INT_MAX defs
#include <string.h>         // for strcpy, strcmp
#include "PMDUtils.h"   
#include "PMDOPCodes.h"     // for exception handling
#include "Parser.h"         // for parsing configuration files

#include "PMDCollections.h" // for access to controller collection
#include "PMDController.h"  // need access to controller for comm
#include "PMDAxisControl.h" // class being implemented here
#include "PMDMemMap.h"
#include "DebugMacros.h"

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

// Debug declarations
const char* PMDAxisControl::classStr = "PMDAxisControl";

#undef dbgCout
#define dbgCout cout << classStr << "(" << (int)ctrlID << 'x' << GetAxisNumber() <<  ")::" << methodStr 

PMDuint16 rslt; // Status result returned from all PMD functions.
#define TRYPMDCMD(cmd,exception) if ((rslt = cmd) != PMD_ERR_OK) throw exception;

const double PMDAxisControl::CyclePeriodStepSizeInUSec = 51.2;
const int PMDAxisControl::NormalControllerCycleMin = 2;
const int PMDAxisControl::ExtendedControllerCycleMin = 3;
const PMDuint16 PMDAxisControl::UserIOAddress = 0;
const PMDuint16 PMDAxisControl::AutostopEventMask = 
                PMDEventMotionErrorMask | 
//				PMDEventCaptureReceivedMask |
                PMDEventInPositiveLimitMask |
                PMDEventInNegativeLimitMask;
const PMDuint16 PMDAxisControl::MoveEventMask = 
                PMDAxisControl::AutostopEventMask | 
//				PMDEventCaptureReceivedMask |
                PMDEventMotionCompleteMask;
const char *PMDAxisControl::StartToken = "[axis]";
const char *PMDAxisControl::EndToken = "[/axis]";

//----------------------------------------------------------------------------
PMDAxisControl::PMDAxisControl( char *uniqueID, 
                                PMDController *controller,
                                PMDAxis axis) : 
    PMD(controller->GetIOTransport(),axis) {
    this->controller = controller;

    strcpy(this->uniqueID,uniqueID);

    SetDefaults();

    // Inherit the debug level from the controller.
    SetDebugLevel(controller->GetDebugLevel());

}

//----------------------------------------------------------------------------
PMDAxisControl::PMDAxisControl(FILE *file, char *token) {

    // Initialize this instance with the defaults.
    SetDefaults();

    // Load values from the configuration file.
    if (ReadConfig(file,token)) {
        // Inherit the debug level from the controller.
        SetDebugLevel(controller->GetDebugLevel());

        // Load the controller ID # for future use by debug messages.
        tagPMDSerialProtocol protocol;
        ((PMDSerial *)controller->GetIOTransport())->GetProtocol(protocol,ctrlID);
    } else
        cerr << "file read error before token " << token << endl;

}

//----------------------------------------------------------------------------
void PMDAxisControl::SetDefaults() {

    ctrlID = 0;
    debugLevel = 0;
    eventStatus = 0;
    signalSense = 0;
    eventStatusLoadedSinceLastMove = false;

    // Actuator defaults
    hasBrake = false;
    brakeIsEngaged = true;  // Failsafe brakes default to engaged
    ampType = PMDMotorOutputPWMSignMagnitude; 
    ampPolarity = ActiveHigh;
    ampIsOn = false;        // desired power-on condition, but...
    isBrushless = false;
    poleCount = 6;
    phaseInitMode = PMDPhaseInitAlgorithmic;
    invertHalls = false;
    commutationMode = PMDCommutationModeHallBased;

    // Encoder defaults
    hasEncoderIndex = false;
    countsPerEncoderCycle = 2000;   // run-of-the-mill 500 count encoder
    encoderCyclesPerAxisCycle = 1;
    motorCyclesPerEncoderCycle = 1;
    countsPerAxisCycle = countsPerEncoderCycle*encoderCyclesPerAxisCycle;
    countsPerMotorCycle = countsPerEncoderCycle*motorCyclesPerEncoderCycle;

    // Analog sensor defaults assume a linear transfer function
    // with zero angle at midpoint of analog range
    absPosAnalogChannel = 1;
    aLUT[0] = 0;
    aLUT[1] = 360.0/1024;
    aLUT[2] = aLUT[3] = 0.0;
    rangeOfMotion = (u_int32_t)countsPerAxisCycle;  // default to 1 axis cycle
    invertMotorOut = false;

    // Physical parameters
    revLimitCounts = revSoftLimitCounts = 0; //INT_MIN+1; //INT_MAX was unsafe
    fwdLimitCounts = fwdSoftLimitCounts = 0; //INT_MAX-1;
    radiusMeters = 0;
    lengthMeters = 0;
    dhAlphaRadians = 0;
    proximalAMeters = 0;
    proximalDMeters = 0;
    distalAMeters = 0;
    distalDMeters = 0;

    // Homing defaults
    homingMode = NoHoming;
    isHomed = false;
    calibratedHomeOffset = 0;
    hasLimitSensors = false;
    hasHardLimits = false;
    hasHomeSensor = false;
    homeSensorWidth = 0;
    homeSignalPolarity = ActiveLow;   // PMD chipset default
    limitSignalPolarity = ActiveLow;   // PMD chipset default
    limitsMayBind = false;
    hasAnalogSensor = false;
    softLimitPad = 0;
    homeDelta = 0;
    waitPeriod = 100;

    ServoFilter defaultFilter = {0,0,0,0,0,0,0,0,0,0};
    CountsProfile defaultProfile = {0,1,1,0,0};
    SetFilter(defaultFilter,true);
    SetProfile(defaultProfile,true,false);
    runFilter = defaultFilter;
    CountsToUnits(runProfile,defaultProfile);

    profileMode = PMDTrapezoidalProfile;

    configurationWasLoaded = false;
    captureEventTriggered = false;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::ConfigureHardware() {


    char methodStr[] = "ConfigureHardware: ";

    ifDbgCout << "Configuring...\n";

    if (controller->HasMemory())
    {
        // See if the data in memory is valid.
        const int buffStart[] = 
            {BuffAddrAxis0,BuffAddrAxis1,BuffAddrAxis2,BuffAddrAxis3};
        SetBufferStart(BuffIDAxis0,buffStart[GetAxisNumber()] +
                                   AxisBuffOffsetCfgDataValid);
        SetBufferLength(BuffIDAxis0,AXIS_CFG_BUFF_SIZE);

        PMDint32 data = 0;
        ReadBuffer(BuffIDAxis0,data);
        if (data == PMDTRUEMEMVALUE)
        {   // Memory-resident configuration data is valid.
            ifDbgCout << "loading configuration from controller memory\n";
            configurationWasLoaded = ReadConfigDataFromMemory(BuffIDAxis0);
            if (configurationWasLoaded) {
	      ifDbgCout << " loading successful\n"; 
	    }
            else
	      ifDbgCout << " loading failed\n"; 
        }
    }

    // If configuration couldn't be loaded from memory, try from a file.
    if (!configurationWasLoaded) 
    {
      //char fName[256];
        stringstream stream(stringstream::in | stringstream::out);
//        stream << "etc/"<<uniqueID << ".cfg" << ends; Why Patrick, why?
        stream << uniqueID << ".cfg" << ends;
        //stream >> fName;

        //FILE *file = fopen(fName, "r");
	FILE *file = fopen(stream.str().c_str(), "r");
        if (file != NULL) {
	  //ifDbgCout << "Loading configuration from " << fName << endl;
	  ifDbgCout << "Loading configuration from " << stream.str() << endl;
            char token[100];
            if (ReadConfig(file,token)) {
	      //ifDbgCout << "Loading from " << fName << " succeeded\n"; 
	      ifDbgCout << "Loading from " << stream.str() << " succeeded\n"; 
            } else {
	      //ifDbgCout << "Loading from " << fName << " failed at " << token << endl;
	      ifDbgCout << "Loading from " << stream.str() << " failed at " << token << endl;
            }
            fclose(file);
        } else
	  //ifDbgCout << "Unable to open config file " << fName << endl;
	  ifDbgCout << "Unable to open config file " << stream.str() << endl;
    }

    // Skip configuration process if axis parameters were never loaded.
    if (!configurationWasLoaded) 
    {   ifDbgCout << "Can't configure axis because parameters not loaded\n";
        return false;
    }

    try {

        // Set the motor type (this must be the first command because
        // it has many (undocumented by PMD) side effects.
        SetMotorTypeInHardware(isBrushless);

        ifDbgCout << "Clearing move event from event status\n";
        ResetEventStatus(~MoveEventMask);

        // Configure the axis out pin to drive the motor amplifier output 
        // enable signal.
        // The trick to doing this is to tell the controller not to tie the
        // axis out pin to any internal register bit and then drive the pin 
        // by alternating its "signal sense" command, as implemented in the
        // EnableAmp/DisableAmp methods.
        ifDbgCout << "Set axis out pin for use as amp enable control\n";
        SetAxisOutSource(GetAxisNumber(),0,PMDAxisOutSourceNone);

        // Make sure the brake is enabled.
        ifDbgCout << "Make sure brake is engaged\n";
        EngageBrake();

        // Turn off the motor amplifier and closed loop control initially.
        ifDbgCout << "Turn off closed loop control and motor amp\n";
        SetMotorMode(PMDMotorOff);
            PMDuint16 motorMode;
            GetMotorMode(motorMode);
        DisableAmp();

        // Set up the hardware limit switch mode and limit signal sense.
        if (hasLimitSensors)
            EnableLimitSensorProtection();
        else
            DisableLimitSensorProtection();

        // Get the current signal sense of the axis from the controller. 
        // This value is modifed throughout the remainder of the
        // configuration process.
        GetSignalSense(signalSense);

        // Set the rotation direction of the motor relative to the encoder
        if (invertMotorOut)
            signalSense |= PMDSignalMotorDirectionMask;
        else
            signalSense &= ~PMDSignalMotorDirectionMask;

        // Set the home & limit sensor signal's active sense
        if (homeSignalPolarity == ActiveHigh)
            signalSense &= ~PMDSignalEncoderHomeMask;   //Clear bit
        else
            signalSense |= PMDSignalEncoderHomeMask;    //Set bit
        if (limitSignalPolarity == ActiveHigh)
            // PMD signal sense default is active low (zero in signal sense
            // register, so invert signal sense for active high input.
            signalSense |=  (PMDSignalPositiveLimitMask |
                             PMDSignalNegativeLimitMask);
        else
            // Make sure the limit signals are stored in the signal status
            // register with the same logic level as they are from the sensor.
            signalSense &= ~(PMDSignalPositiveLimitMask |
                             PMDSignalNegativeLimitMask);
        ifDbgCout << "Set signal sense for home signal and limit polarity\n";
        SetSignalSense(signalSense);

        // Clear all events
        ifDbgCout << "Clear events, set motor output mode, and set motor type\n";
        ResetEventStatus(0);

        // Set the motor output mode (either PWM50/50 or sign/magnitude).
        SetOutputMode(ampType);

        // If this axis has a brushless motor, configure the controller 
        // accordingly. Refer to the Pilot User's Guide, section 12.7.2 
        // Hall-based Initialization Sequence, for command sequence explanation.
        if (isBrushless) {

            ifDbgCout << "Brushless settings\n";

            // TESTING
            if (false) {
                SetPhaseInitializeMode(1);
                SetPhaseCounts(933);
                GetSignalSense(signalSense);
                signalSense |= PMDSignalMotorDirectionMask;
                SetSignalSense(signalSense);
                SetPhaseCorrectionMode(0);
                SetMotorMode(0);
                InitializePhase();
                SetMotorMode(0);
                SetMotorCommand(-6000);
                EnableAmp();
                Update();
                PMDUtils::DoSleep(2000);
                SetMotorCommand(6000);
                Update();
                PMDUtils::DoSleep(2000);
                SetMotorCommand(0);
                Update();
                DisableAmp();
            }
            // END TESTING

            // Set hall signal sense.
            if (invertHalls)
                signalSense |= PMDSignalHallAMask | PMDSignalHallBMask | PMDSignalHallCMask;
            else
                signalSense &= ~(PMDSignalHallAMask | PMDSignalHallBMask | PMDSignalHallCMask);
            SetSignalSense(signalSense);

            // Always 3 phase brushless.
            // NOTE: All PMD controllers capable of driving brushless will default to
            // 3-phase. This code does not currently support any other brushless motor type.

            // Set encoder counts/motor phase
            SetPhaseCounts((PMDuint16)(countsPerMotorCycle/(poleCount/2)));

            // Enable index pulse phase correction if encoder counts/motor rev
            // is not an integer multiple of the motor's electrical cycles
            // WARNING: This logic has been commented out because phase 
            // correction will not work if the encoder pulse occurs more 
            // than once per motor rotation. Since our typical application
            // doesn't rotate an axis more than one or two revolutions, 
            // disabling index pulse phase correction should be okay even if
            // the encoder counts/rev is not an integer multiple of the poles
//            if (((int)(countsPerMotorCycle/(poleCount/2)))*(poleCount/2) ==
//                (int)countsPerMotorCycle)
            SetPhaseCorrectionMode(PMDPhaseCorrectionDisabled);
//            else
//                SetPhaseCorrectionMode(PMDPhaseCorrectionEnabled);

            // Set the commutation mode. Sinusoidal 
            // commutation requires encoder feedback with an index pulse if the
            // number of encoder counts/rev is not evenly divisible by half the
            // number of rotor magnetic poles. See Pilot User's Guide section 12.4
            // for further details.
            SetCommutationMode(commutationMode);

            // Only need to do phase initialization for sinusoidal commutation.
            if (commutationMode == PMDCommutationModeSinusoidal) {

                // Select the phase initialization mode.
                SetPhaseInitializeMode(phaseInitMode);
                SetMotorMode(PMDMotorOff);
                if (phaseInitMode == PMDPhaseInitAlgorithmic) {
                    // Prepare axis for algorithmic phase initialization motion.
                    SetPhaseInitializeTime(3000);
                    SetMotorCommand(6000);
                    EnableAmp();
                    DisengageBrake();
                }

                // Initialize phasing and wait for it to complete if algorithmic.
                InitializePhase();
                if (phaseInitMode == PMDPhaseInitAlgorithmic) {
                    PMDuint16 activityStatus;
                    do {
                        GetActivityStatus(activityStatus);
                        PMDUtils::DoSleep(250);
                    } while (!(activityStatus & (1<<PMDActivityPhasingInitializedBit)));
                    EngageBrake();
                    DisableAmp();
                }
            }

            // Display detected phase offset (only valid if index was seen).
//            PMDuint16 phaseOffset;
//            GetPhaseOffset(phaseOffset);
//            cout << "phase offset = " << phaseOffset << endl;

        }

        // Set the profile mode and start closed loop control.
        ifDbgCout << "Enable axis (but not closed loop control)\n";
        SetProfileMode(profileMode);
        SetAxisMode(PMDAxisOn);
    	GetMotorMode(motorMode);
    } catch (int cmd) {
        ifDbgCout << "exception while commanding " << cmd << endl; 
    }
    
    ifDbgCout << "Hardware configuration complete\n";

    return (rslt == PMD_ERR_OK);
}

//----------------------------------------------------------------------------
void PMDAxisControl::GetFilter(ServoFilter& sp, bool homing, bool currently_running) {
  if(currently_running)
  {
    UpdateActualActiveFilter();
    sp = activeFilter;

    return;
  }

  if (homing) sp = homingFilter;
  else        sp = activeFilter;
}

void PMDAxisControl::UpdateActualActiveFilter() {
  GetKp(activeFilter.kP);
  GetKd(activeFilter.kD);
  GetKi(activeFilter.kI);
  GetKvff(activeFilter.kVFF);
  GetKaff(activeFilter.kAFF);
  GetKout(activeFilter.kOut);
  GetIntegrationLimit(activeFilter.intLim);
  GetMotorLimit(activeFilter.motorLim);
  GetMotorBias(activeFilter.motorBias);
  GetPositionErrorLimit(activeFilter.errorLim);
}


//----------------------------------------------------------------------------
void PMDAxisControl::SetFilter(const ServoFilter& sp, bool homing) {
    
    char methodStr[] = "SetFilter: ";

    if (homing) {
        
        // For homing, just record the values in the homing structure for 
        // use when homing is executed.
        homingFilter = sp;
        
    }
    else {
        
        // Send the control parameter values directly to the controller.
        try {
            if (activeFilter.kP != sp.kP) SetKp(sp.kP);
            if (activeFilter.kD != sp.kD) SetKd(sp.kD);
            if (activeFilter.kI != sp.kI) SetKi(sp.kI);
            if (activeFilter.kVFF != sp.kVFF) SetKvff(sp.kVFF);
            if (activeFilter.kAFF != sp.kAFF) SetKaff(sp.kAFF);
            if (activeFilter.kOut != sp.kOut) SetKout(sp.kOut);
            if (activeFilter.intLim != sp.intLim) SetIntegrationLimit(sp.intLim);
            if (activeFilter.motorLim != sp.motorLim) SetMotorLimit(sp.motorLim);
            if (activeFilter.motorBias != sp.motorBias) SetMotorBias(sp.motorBias);
            if (activeFilter.errorLim != sp.errorLim) SetPositionErrorLimit(sp.errorLim);
        
            rslt = Update();

            // Record the new values being commanded for future reference.
            activeFilter = sp;
        } catch (int cmd) {
            ifDbgCout << "exception while commanding " << cmd << endl; 
        }

    }
}

//----------------------------------------------------------------------------
void PMDAxisControl::GetProfile(Profile& profile, bool homing) {
    if (homing) {
        profile = homingProfile;
    }
    else {
        profile = cmdProfile;
    }
}

//----------------------------------------------------------------------------
void PMDAxisControl::GetProfile(CountsProfile& profile, bool homing) {
    if (homing) {
        profile = homingProfileCounts;
    }
    else {
        profile = cmdProfileCounts;
    }
}

//----------------------------------------------------------------------------
void PMDAxisControl::SetProfile(const Profile& profile, bool homing,
                                bool doLoad) {
    if (homing) {
        homingProfile = profile;
        UnitsToCounts(profile,homingProfileCounts);
        // Also copy to command profile if expecting to send to controller.
        if (doLoad) {
            cmdProfileCounts = homingProfileCounts;
            SendProfileToController();
        }
    } else {

        // Convert the new profile to counts.
        UnitsToCounts(profile, cmdProfileCounts);

        // Crop profile
        if (CropProfile(cmdProfileCounts)) {

            // Conversion to counts had at least one parameter that does not
            // match the original profile after conversion. Make the reverse
            // conversion and store this as the actual profile being used.
            CountsToUnits(cmdProfile,cmdProfileCounts);

        } else {

            // Store the profile as it was commanded.
            cmdProfile = profile;

        }

        // Transfer the new profile to the controller if desired.
        if (doLoad) SendProfileToController();

    }

}

//----------------------------------------------------------------------------
void PMDAxisControl::SetProfile(const CountsProfile& profile, bool homing,
                                bool doLoad) {

    // Make local copy of profile in case it needs to be cropped.
    CountsProfile countsProfile = profile;

    if (homing) homingProfileCounts = countsProfile;
    else CropProfile(countsProfile);

    // Convert the possibly cropped profile from counts to world units.
    CountsToUnits(cmdProfile, countsProfile);

    if (homing) homingProfile = cmdProfile;
    cmdProfileCounts = countsProfile;

    // Transfer the new profile to the controller if desired.
    if (doLoad) SendProfileToController(countsProfile);

}

//----------------------------------------------------------------------------
bool PMDAxisControl::CropProfile(CountsProfile& profile) {
    
    // For now just check for position limits. 
    // Future versions will have checks for velocity and acceleration.

    char methodStr[] = "CropProfile: ";

    // Don't do cropping unless the limits are different.
    PMDint32 pos = profile.pos;
    if (fwdLimitCounts != revLimitCounts) {
        profile.pos = min(fwdSoftLimitCounts,max(revSoftLimitCounts,profile.pos));
    }
    bool retval = (pos != profile.pos);
    if (retval)
      ifDbgCout << "Profile postion cropped from "<<pos<<" to "<<profile.pos<<"\n";      

    return retval;
}
//----------------------------------------------------------------------------
void PMDAxisControl::UpdateLimits() {
    // Update the soft limits
    if (hasHardLimits) {
        // Hard limits are relative to the home offset and range of motion,
        // while the soft limits are 2% inside of the hard limits.
        if (softLimitPad == 0) {
	        // Set soft limits to default of 2% range of motion
            fwdSoftLimitCounts = fwdLimitCounts - (PMDlong32)rangeOfMotion/2*.02;
            revSoftLimitCounts = revLimitCounts + (PMDlong32)rangeOfMotion/2*.02;
        } else {
            fwdSoftLimitCounts = fwdLimitCounts - softLimitPad;
            revSoftLimitCounts = revLimitCounts + softLimitPad;
        }
    } else {
        // Set artificial soft limits at +- one full rotation and leave the
        // hard limits at their max values since there are no hard limits.
        fwdSoftLimitCounts = (PMDlong32)rangeOfMotion/2;
        revSoftLimitCounts = -fwdSoftLimitCounts;
    }

}

//----------------------------------------------------------------------------
PMDint32 PMDAxisControl::EstimateAbsPosFromAnalogSensor(PMDuint16 analog)
{
    double avf = analog;
    // Calculate angular position using 3rd order polynomial
    double angleInDegrees =
        aLUT[0] + aLUT[1]*avf + aLUT[2]*avf*avf + aLUT[3]*avf*avf*avf;
    // Convert angle into counts
    return UnitsToCounts(PMDUtils::DegsToRevs(angleInDegrees));
}

//----------------------------------------------------------------------------
PMDuint16 PMDAxisControl::SampleAnalog(PMDuint16 analogID, int sampleCount)
{
    PMDuint16 sample = 0;
    PMDuint16 analog;
    for (int i = 0; i < sampleCount; i++)
    {
        ReadAnalog(analogID, analog);
        sample += analog;
        PMDUtils::DoSleep(100);
    }
    sample /= sampleCount;

    return sample;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::CaptureEventTriggered()
{
    GetLastEventStatus();
    	return ((eventStatus & PMDEventCaptureReceivedMask) != 0);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::HomeSensorIsAsserted()
{
	PMDuint16 signalStatus;
	GetSignalStatus(signalStatus);
	return ((signalStatus & PMDSignalEncoderHomeMask) != 0);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::PositionErrorOccured(bool forceGet)
{ 
  if (forceGet)
    GetEventStatus(eventStatus);
  else
    GetLastEventStatus();
  return ((eventStatus & PMDEventMotionErrorMask) != 0);
}

//----------------------------------------------------------------------------
void PMDAxisControl::SetHomingLimits(
                    const PMDlong32 fwdHardLimit, const PMDlong32 revHardLimit,
                    const PMDlong32 fwdSoftLimit, const PMDlong32 revSoftLimit) {
    this->fwdLimitCounts = fwdHardLimit;
    this->revLimitCounts = revHardLimit;
    this->fwdSoftLimitCounts = fwdSoftLimit;
    this->revSoftLimitCounts = revSoftLimit;

    // Record the fact that this axis is considered homed
    SetHomedState(true);

}

//----------------------------------------------------------------------------
void PMDAxisControl::GetHomingLimits(
		PMDlong32& fwdHardLimit, PMDlong32& revHardLimit,
                PMDlong32& fwdSoftLimit, PMDlong32& revSoftLimit) {
    fwdHardLimit = this->fwdLimitCounts;
    revHardLimit = this->revLimitCounts;
    fwdSoftLimit = this->fwdSoftLimitCounts;
    revSoftLimit = this->revSoftLimitCounts;
}

//----------------------------------------------------------------------------
// Control of the motor amp enable pin is implemented through the axisOut 
// pins. By setting the axisOut state to not be tied to any of the status 
// register bits, the application has direct control of the axisOut value by
// setting the signal sense of the AxisOut signal.
// For example, if the signal sense of the pin is set to non-inverted,
// then the axisOut pin will be low. However, if the signal sense is
// set to inverted, then the axisOut pin will go high, thus enabling
// the amp. 
// Setting of the signal sense for the axisOut pin is done by getting 
// the current state of the signal sense mask from the control chip,
// setting the value of the axisOut bit, and then setting the signal
// sense register to the new value.
// WARNING: axis out defaults to high on powerup or reset, so if the amp enable
// control for the amplifier is active high, the amplifier will be powered
// after reset.
void PMDAxisControl::EnableAmp() {
    PMDuint16 signalSenseMask;
    GetSignalSense(signalSenseMask);
    if (ampPolarity == ActiveHigh)
        signalSenseMask &= ~PMDSignalAxisOutMask; 
    else
        signalSenseMask |= PMDSignalAxisOutMask; // Active low so motor defaults to off.
    rslt = SetSignalSense(signalSenseMask);    
    ampIsOn = true;
}
void PMDAxisControl::DisableAmp() {
    PMDuint16 signalSenseMask;
    GetSignalSense(signalSenseMask);
    if (ampPolarity == ActiveHigh)
        signalSenseMask |= PMDSignalAxisOutMask; // Active low so motor defaults to off.
    else
        signalSenseMask &= ~PMDSignalAxisOutMask; 
    rslt = SetSignalSense(signalSenseMask);    
    ampIsOn = false;
}

//----------------------------------------------------------------------------
void PMDAxisControl::SetMotorTypeInHardware(bool isBrushless) {

    char methodStr[] = "SetMotorTypeInHardware: ";
    // First check what kind of controller this is.
    // If this is a magellan controller, set the glue logic flag
    // for brushless motors and tell the chipset that this axis
    // is brushless. Otherwise, do nothing because the older
    // controllers can't change their motor type.
    PMDuint16 userIOData;
    ifDbgCout << (isBrushless ? " is brushless" : "is brushed") << endl;
    ReadIO(UserIOAddress,userIOData);
    if (isBrushless)
        userIOData = userIOData | UIOMaskIsBrushless;
    else
        userIOData = userIOData & ~UIOMaskIsBrushless;
    WriteIO(UserIOAddress,userIOData);

    // SetMotorType resets lots of important parameters, so only do it
    // if necessary.
    if (controller->GetPMDProductFamily() == PMDFamilyMagellan) {
        PMDuint16 motortype = GetMotorType();
        if (isBrushless && motortype != PMDBrushless3Phase) 
            SetMotorType(PMDBrushless3Phase);
        else if (!isBrushless && motortype != PMDDCBrush) SetMotorType(PMDDCBrush);
    }
}

bool PMDAxisControl::IsEStopped()
{
  PMDuint16 userIOData;
  ReadIO(UserIOAddress+GetAxisNumber(),userIOData);
  return !(userIOData & UIOMaskIsEStopped);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::IsBrakeEngaged() {
    if (hasBrake) {
        return brakeIsEngaged;
//        return ((controller->brakeState & (1 << GetAxisNumber())) == 0);
    } else 
        return false;
}

// NOTE: Brakes are currently assumed to be failsafe, meaning that they are 
// engaged when not powered. Setting their state high (on) disengages
// the brake.
// NOTE: Logic for EngageBrake() and DisengageBrake() currently assumes only
// one axis. To expand this, the glue logic registers would need to have 
// bits reserved for each potential axis, up to 4.
//----------------------------------------------------------------------------
void PMDAxisControl::EngageBrake() {
    if (hasBrake) {
        PMDuint16 userIOData;
        ReadIO(UserIOAddress+GetAxisNumber(),userIOData);
        userIOData = userIOData & ~UIOMaskBrake;
        WriteIO(UserIOAddress,userIOData);
        brakeIsEngaged = true;
    }
}

//----------------------------------------------------------------------------
void PMDAxisControl::DisengageBrake() {
    // WARNING: this logic only works for a single axis controller.
    // The controller glue logic needs to be updated to use a separate
    // IO address for each axis so isHomed, isBrushless, and brakeState
    // have their own bit for each axis. Currently the glue logic
    // only implements the register for the first axis.
    if (hasBrake) {
        PMDuint16 userIOData;
        ReadIO(UserIOAddress+GetAxisNumber(),userIOData);
        userIOData = userIOData | UIOMaskBrake;
        WriteIO(UserIOAddress,userIOData);
        brakeIsEngaged = false;
    }
}

//----------------------------------------------------------------------------
PMDuint16 PMDAxisControl::GetMotorType() {
    PMDuint16 motorType;
    if (controller->GetPMDProductFamily() == PMDFamilyMagellan) {
        PMD::GetMotorType(motorType);
        return motorType;
    } else {
        PMDuint16 family,n,s,c,major,minor;
        controller->GetPMDVersion(family,motorType,n,s,c,major,minor);
        return motorType;
    }
}

//----------------------------------------------------------------------------
void PMDAxisControl::SetMotorType(PMDuint16 motorType) {
    if (controller->GetPMDProductFamily() == PMDFamilyMagellan)
        PMD::SetMotorType(motorType);
}

//----------------------------------------------------------------------------
// Lock brake, disable amp, and turn off closed loop control.
void PMDAxisControl::Park() {
    EngageBrake();
    DisableAmp();
    SetMotorMode(PMDMotorOff);
}

//----------------------------------------------------------------------------
// Turn on closed loop control, enable amp, and unlock brake.
void PMDAxisControl::Unpark() {
    ResetEventStatus(~MoveEventMask);
    EnableAmp();
    SetMotorMode(PMDMotorOn);
    DisengageBrake();
}

//----------------------------------------------------------------------------
void PMDAxisControl::GetAxisConfigBuffInfo(PMDuint16 &buffID, PMDuint32 &buffAddr)
{
    switch (GetAxisNumber()) 
    {   case 0: buffID = BuffIDAxis0; buffAddr = BuffAddrAxis0; break;
        case 1: buffID = BuffIDAxis1; buffAddr = BuffAddrAxis1; break;
        case 2: buffID = BuffIDAxis2; buffAddr = BuffAddrAxis2; break;
        case 3: buffID = BuffIDAxis3; buffAddr = BuffAddrAxis3; break;
    }
}

//----------------------------------------------------------------------------
bool PMDAxisControl::ParkAndSave() {

    char methodStr[] = "ParkAndSave: ";
    bool didParkAndSave = false;

    // Only attempt this action if the axis is connected.
    if (controller->IsConnected()) {

        // Secure the axis.
        Park();

        // Only create the park file if this axis has been homed properly.
        if (isHomed) 
        {
            // Collect the axis information to be saved.
            PMDlong32 pos;
            GetActualPosition(pos);
            ifDbgCout << "Park for "<<controller->uniqueID << '-' << uniqueID << " will try to use "<<(controller->HasMemory() ? "MEMORY" : "FILE") << "\n";

            if (controller->HasMemory())
            {
                // Controller has memory, so save park info there.

                // Set up buffer memory access. 
                PMDuint16 buffID=0;
                PMDuint32 buffAddr=0;
                GetAxisConfigBuffInfo(buffID,buffAddr);
                SetBufferStart(buffID,buffAddr+AxisBuffOffsetParkInfo);
                SetBufferLength(buffID,AXIS_CFG_BUFF_SIZE);

                // Write the data to memory in sequence
                WriteBuffer(buffID,PMDTRUEMEMVALUE); // IsParked flag
                WriteBuffer(buffID,pos);    // Parked position

                didParkAndSave = true;

            } else
            {
                // Controller does not have memory, so
                // record park parameters to a file.
                char fName[100];
                stringstream stream(stringstream::in | stringstream::out);
                stream << "etc/"<<controller->uniqueID << '-' << uniqueID << ".park" << ends;
                stream >> fName;
                ofstream fp(fName,ios::out);

                if (fp) {

                    fp << pos << endl;
                    fp.close();
                    PMDUtils::DoSleep(250); // Slow down the process a little.
                    didParkAndSave = true;
                }
            }
        } else {
            ifDbgCout << " No park file is being created because "
                 << " the axis hasn't been homed." << endl;
        }
    }
    ifDbgCout << "Park for "<<controller->uniqueID << '-' << uniqueID << " was "<<(didParkAndSave ? "successful" : "unsuccessful") << "\n";
    return didParkAndSave;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::RecoverFromParkAndSave() {
    char methodStr[] = "RecoverFromParkAndSave: ";
    bool didUnparkAndSave = false;

    UpdateLimits();

    // Only attempt this action if the axis is connected.
    if (controller->IsConnected()) {

        int pos;

	    ifDbgCout << "Unpark for "<<controller->uniqueID << '-' << uniqueID << " will try to use "<<(controller->HasMemory() ? "MEMORY" : "FILE") << "\n";

        if (controller->HasMemory())
        {
                // Controller has memory, so recover park info from there.

                // First, set up access to the memory space. 
                PMDuint16 buffID=0;
                PMDuint32 buffAddr=0;
                GetAxisConfigBuffInfo(buffID,buffAddr);
                SetBufferStart(buffID,buffAddr+AxisBuffOffsetParkInfo); // Park data starts at 2nd buffer location.
                SetBufferLength(buffID,AXIS_CFG_BUFF_SIZE);

                // Read the data from memory
                PMDint32 data;
                ReadBuffer(buffID,data);                // Park flag (skip it)
                if (data == PMDTRUEMEMVALUE) 
                {   // Only continue reading data if park flag is set
                    ReadBuffer(buffID,data); pos = data;    // Parked position
                    
                    // Clear the park flag in memory
                    SetBufferStart(buffID,buffAddr+AxisBuffOffsetParkInfo);
                    WriteBuffer(buffID,0x0);

                    didUnparkAndSave = true;
                } 

        } else
        {
            char fName[256];
            stringstream stream(stringstream::in | stringstream::out);
            stream << "etc/"<<controller->uniqueID << '-' << uniqueID << ".park" << ends;
            stream >> fName;
	    ifDbgCout << "Attempting to open: >" << fName << "<" << std::endl;

            ifstream fp(fName,ios::in);
            if (fp.is_open()) {

                // Get the values from the file.
                fp >> pos;
                fp.close();

			    //RRB - workaround for problems when pos value is -1 or -2
			    if((pos == -1) || (pos == -2))
			    {
				    cout << "Modifying park value of " << pos << " to zero." << endl;
				    pos = 0;
			    }
            
                // Erase the park file since it is now no longer valid.
                if (remove(fName) != 0) {
                    ifDbgCout << "Removal of " << fName << " park file failed\n";
                } else {
                    ifDbgCout << "Removal of " << fName << " park file succeeded\n";
                }
                didUnparkAndSave = true;
            }
        }
        if (didUnparkAndSave) {

            // Record the fact that this axis is considered homed
            SetHomedState(true);

            SetActualPosition(pos);

            // Release any brakes and put the joint in closed loop mode.
            Unpark();

            // Load axis control parameters
            SetFilter(runFilter);
            CountsProfile profile;
            UnitsToCounts(runProfile,profile);
            profile.pos = pos;
            SetProfile(profile);
            Move();

            double softLLdeg,softULdeg;
            softLLdeg = PMDUtils::RevsToDegs(CountsToUnits(revSoftLimitCounts));
            softULdeg = PMDUtils::RevsToDegs(CountsToUnits(fwdSoftLimitCounts));
            ifDbgCout << "Unparked successfully. Range of motion is "
	              << revSoftLimitCounts << " .. " << fwdSoftLimitCounts
	              << " (" << softLLdeg << " .. " << softULdeg << " degrees)\n";

        }
    } 
    ifDbgCout << "Unpark for "<<controller->uniqueID << '-' << uniqueID << " was "<<(didUnparkAndSave ? "successful" : "unsuccessful") << "\n";
    return didUnparkAndSave;
}

//----------------------------------------------------------------------------
// Only need to call this if application is NOT calling a Move command.
void PMDAxisControl::PreMove() {

    // Get latest event status, querying the chipset if the status
    // has not been sampled since the last move was issued.
    GetLastEventStatus();

    // Check for motion error and reset axis if detected.
//        (eventStatus & PMDEventMotionCompleteMask) != 0 )
    if (eventStatus != 0)
        ResetEventStatus(0);

    if ((eventStatus & PMDEventMotionErrorMask) != 0)
      SetMotorMode(PMDMotorOn);

    // Force the profile load logic to load velocity if an autostop event
    // occured.
    if ((eventStatus & AutostopEventMask) != 0)
        loadedProfileCounts.vel = 0;

    eventStatusLoadedSinceLastMove = false;
    eventStatus = 0;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::Move(bool waitForMotionToStop, bool doUpdate) {

    PreMove();

    // Start moving to the new position.
    SendProfileToController();

    if (doUpdate) Update();
    else waitForMotionToStop = false;

    // Wait for motion to complete if desired.
    if (waitForMotionToStop) {
        return WaitForMotionToStop();
    } else
        return true;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::MoveWithCaptureInterrupt(bool waitForMotionToStop) {

    PreMove();
	
    // Create breakpoint to smooth stop motion if capture event occurs.
    // This minimizes motion for a capture event move.
    SetBreakpointValue(PMDBreakpoint1,
        PMDEventCaptureReceivedMask<<16 | PMDEventCaptureReceivedMask);
    SetBreakpoint(PMDBreakpoint1,GetAxisNumber(),
        PMDBreakpointActionSmoothStop,PMDBreakpointEventStatus);

    // Start motion
    SendProfileToController();
    Update();

    // Wait for motion to complete if desired.
    bool moveStatus = false;
    if (waitForMotionToStop) {
//        moveStatus = WaitForMotionToStop(false);
        moveStatus = WaitForMotionToStop(true);
    } else
        moveStatus = true;

    // Clear the breakpoint action so it doesn't trigger again.
    SetBreakpoint(PMDBreakpoint1,GetAxisNumber(),PMDBreakpointNoAction,
        PMDBreakpointDisable);
    
    captureEventTriggered = CaptureEventTriggered();

    return moveStatus;
}

//----------------------------------------------------------------------------
// Returns false if a motion error is detected. Otherwise, returns true when
// motion is complete.
bool PMDAxisControl::WaitForMotionToStop(bool includeCaptureEvent) {
    while (!IsMotionDone(includeCaptureEvent)) {
        PMDUtils::DoSleep(waitPeriod);
    }
    return !(eventStatus & AutostopEventMask);
}

//----------------------------------------------------------------------------
// Only need to call this if application is NOT calling a Move command.
void PMDAxisControl::PostMove() {
    // Re-enable closed loop control if a motion error is asserted.
    if (mustEnableClosedLoopControl) SetMotorMode(PMDMotorOn);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::Move(bool waitForMotionToStop, double pos, 
			  double vel, double acc, double dec, double jerk, 
			  bool doUpdate) {
    Profile profile = cmdProfile;
    if (profileMode != PMDVelocityContouringProfile) {
        profile.pos = pos;
        if (vel != 0.0) profile.vel = vel;
    } else 
        profile.vel = vel;
    if (acc != 0.0) profile.acc = acc;
    if (dec != 0.0) profile.dec = dec;
    if (jerk != 0.0) profile.jerk = jerk;
    SetProfile(profile,false,false);
    return Move(waitForMotionToStop, doUpdate);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::Halt(bool waitForMotionToStop) {
  Profile profile = cmdProfile;
  profile.vel = 0.0;
  SetProfile(profile,false,false);
  return Move(waitForMotionToStop);
}


//----------------------------------------------------------------------------
bool PMDAxisControl::Move(bool waitForMotionToStop, PMDint32 pos, 
			  PMDint32 vel, PMDuint32 acc, PMDuint32 dec, 
			  PMDuint32 jerk,
			  bool doUpdate) {
    CountsProfile newProfile,cmdCountsProfile;
    UnitsToCounts(cmdProfile,cmdCountsProfile);
    newProfile = cmdCountsProfile;
    if (profileMode != PMDVelocityContouringProfile) {
        newProfile.pos = pos;
        if (vel != 0) newProfile.vel = vel;
    } else
        newProfile.vel = vel;
    if (acc != 0) newProfile.acc = acc;
    if (dec != 0) newProfile.dec = dec;
    if (jerk != 0) newProfile.jerk = jerk;
    SetProfile(newProfile,false,false);
    return Move(waitForMotionToStop, doUpdate);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::MovePseudoVel(bool waitForMotionToStop,  
				   PMDint32 vel, PMDuint32 acc, 
				   PMDuint32 dec, bool doUpdate)
{
    // Set the position to one extreme based on velocity sign and remove sign
    // from velocity. 
    // Then just do a normal move with those values.
    PMDint32 pos;

    // A velocity of zero gets translated into a pos cmd to the current pos.
    if   (vel == 0) 
        GetActualPosition(pos);
    else 
        pos = (vel > 0) ? fwdSoftLimitCounts : revSoftLimitCounts;
    return Move(waitForMotionToStop,pos,abs(vel),acc,dec,0,doUpdate);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::MovePseudoVel(bool waitForMotionToStop, 
				   double vel, double acc, double dec, 
				   bool doUpdate)
{
  return MovePseudoVel(waitForMotionToStop, UnitsToCounts(vel),
		       UnitsToCounts(acc), 
		       UnitsToCounts(dec), 
		       doUpdate);
}

//----------------------------------------------------------------------------
void PMDAxisControl::EnableLimitSensorProtection() {
    if (controller->GetPMDProductFamily() == PMDFamilyMagellan)
    {   PMDEventActionActionType action =  
            hasLimitSensors ? PMDEventActionActionAbruptStopWithPosErrorClear :
                              PMDEventActionActionNone;
        SetEventAction(PMDEventActionEventPosLim, action);
        SetEventAction(PMDEventActionEventNegLim, action);
        RestoreOperatingMode();
    } else
    {   tagPMDLimitMode limitSwitchMode = 
            hasLimitSensors ? PMDLimitEnabled: PMDLimitDisabled;
        SetLimitSwitchMode(limitSwitchMode);
    }
}

//----------------------------------------------------------------------------
void PMDAxisControl::DisableLimitSensorProtection() {
    if (controller->GetPMDProductFamily() == PMDFamilyMagellan)
    {   SetEventAction(PMDEventActionEventPosLim,PMDEventActionActionNone);
        SetEventAction(PMDEventActionEventNegLim,PMDEventActionActionNone);
        RestoreOperatingMode();
    } else
        SetLimitSwitchMode(PMDLimitDisabled);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::IsMotionDone(bool includeCaptureEvent) {
    GetEventStatus(eventStatus);
    eventStatusLoadedSinceLastMove = true;
	unsigned short eventMask = MoveEventMask;
	if (includeCaptureEvent) eventMask |= PMDEventCaptureReceivedMask;
	return ((eventStatus & eventMask) != 0);
}

//----------------------------------------------------------------------------
void PMDAxisControl::UnitsToCounts(const Profile& profile, 
                                     CountsProfile& counts) {
    counts.pos  = UnitsToCounts(profile.pos);
    counts.vel  = UnitsToCounts(profile.vel);
    counts.acc  = UnitsToCounts(profile.acc);
    counts.dec  = UnitsToCounts(profile.dec);
    counts.jerk = UnitsToCounts(profile.jerk);
}

//----------------------------------------------------------------------------
void PMDAxisControl::CountsToUnits(Profile& profile, 
                                     const CountsProfile& counts) {
    profile.pos  = CountsToUnits(counts.pos);
    profile.vel  = CountsToUnits(counts.vel);
    profile.acc  = CountsToUnits(counts.acc);
    profile.dec  = CountsToUnits(counts.dec);
    profile.jerk = CountsToUnits(counts.jerk);
}

//----------------------------------------------------------------------------
PMDint32 PMDAxisControl::CountsVelToPMD(PMDint32 countsVel) {
	return (PMDint32) (countsVel*GetCycleTimeInSecs()*PMDVelScaling);
}

//----------------------------------------------------------------------------
PMDuint32 PMDAxisControl::CountsAccToPMD(PMDuint32 countsAcc) {
    return (PMDuint32)(countsAcc*pow(GetCycleTimeInSecs(),2.0)*PMDAccScaling);
}

//----------------------------------------------------------------------------
PMDuint32 PMDAxisControl::CountsJerkToPMD(PMDuint32 countsJerk) {
    return (PMDuint32)(countsJerk*pow(GetCycleTimeInSecs(),3.0)*PMDJerkScaling);
}

//----------------------------------------------------------------------------
PMDint32 PMDAxisControl::PMDToCountsVel(PMDint32 pmdVel) {
	return (PMDint32) (pmdVel/PMDVelScaling/GetCycleTimeInSecs());
}

//----------------------------------------------------------------------------
PMDuint32 PMDAxisControl::PMDToCountsAccToPMD(PMDuint32 pmdAcc) {
    return (PMDuint32)(pmdAcc/PMDAccScaling/pow(GetCycleTimeInSecs(),2.0));
}

//----------------------------------------------------------------------------
PMDuint32 PMDAxisControl::PMDToCountsJerkToPMD(PMDuint32 pmdJerk) {
    return (PMDuint32)(pmdJerk/PMDJerkScaling/pow(GetCycleTimeInSecs(),3.0));
}

//----------------------------------------------------------------------------
void PMDAxisControl::SendProfileToController(const CountsProfile& countsProfile) {
    cmdProfileCounts = countsProfile;
    SendProfileToController();
}

//----------------------------------------------------------------------------
void PMDAxisControl::SendProfileToController() {
    // Go through each of the profile values and "load"
    // only the values that are different from the current settings.
    // This minimizes command traffic by eliminating redundant commands.
  //    bool somethingWasSet = false;
    PMDint32 counts;
    PMDuint32 ucounts;
    if (loadedProfileCounts.vel != cmdProfileCounts.vel || captureEventTriggered) {
        counts = CountsVelToPMD(cmdProfileCounts.vel);
        if (profileMode != PMDVelocityContouringProfile)
            if (counts == 0) counts = 1;    // Can't allow a vel of zero unless in vel mode
        SetVelocity(counts);
        captureEventTriggered = false;
	//        somethingWasSet = true;
    }
    if (loadedProfileCounts.acc != cmdProfileCounts.acc) {
        ucounts = CountsAccToPMD(cmdProfileCounts.acc);
//        if (ucounts == 0) ucounts = 1;  // Can't allow an acc of zero
        if (ucounts < 2) ucounts = 2;  // V2.3 of navigator won't allow acc < 2
        SetAcceleration(ucounts);
	//        somethingWasSet = true;
    }
    if (loadedProfileCounts.dec != cmdProfileCounts.dec) {
        ucounts = CountsAccToPMD(cmdProfileCounts.dec);
//        if (countsProfile.dec != 0 && ucounts == 0) ucounts = 1;
        if (cmdProfileCounts.dec != 0 && ucounts < 2) ucounts = 2;
        SetDeceleration(ucounts);
	//        somethingWasSet = true;
    }
    if (loadedProfileCounts.jerk!= cmdProfileCounts.jerk) {
        ucounts = CountsJerkToPMD(cmdProfileCounts.jerk);
        if (cmdProfileCounts.jerk != 0 && ucounts == 0) ucounts = 1;
        SetJerk(CountsJerkToPMD(ucounts));
	//        somethingWasSet = true;
    }
    if (profileMode != PMDVelocityContouringProfile)
        SetPosition(cmdProfileCounts.pos);

    loadedProfileCounts = cmdProfileCounts;
}

//----------------------------------------------------------------------------
double PMDAxisControl::GetCycleTimeInSecs() { 
    return controller->GetCycleTime()/1.0E6;
}

//----------------------------------------------------------------------------
PMDuint16 PMDAxisControl::GetLastEventStatus() {

    // If the last move did not have any wait, then the event status probably
    // has not been sampled, so sample it now and indicate that the register
    // has been sampled and the most recent data is stored in the local 
    // variable.
    if (!eventStatusLoadedSinceLastMove) {
        eventStatusLoadedSinceLastMove = true;
        GetEventStatus(eventStatus);
    }
    return eventStatus;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::ChangeComm(int baud) {
    if (controller != NULL)  return controller->ChangeComm(baud);
    else                     return false;
}


//----------------------------------------------------------------------------
bool PMDAxisControl::SetProfileMode(tagPMDProfileMode mode) {
    if (PMD::SetProfileMode(mode) == PMD_ERR_OK) {
        profileMode = mode;
        return true;
    } else
        return false;
}

//----------------------------------------------------------------------------
void PMDAxisControl::CalibrateAnalogSensor()
{
  //  char methodStr[] = "CalibrateAnalogSensor: ";

  if (isHomed)
    {   
      Profile profile = homingProfile;
      profile.pos=0;
      SetProfile(profile);
      Move();
      PMDUtils::DoSleep(1000);
      
      PMDuint16 absPosAnalogValue;
      ReadAnalog(absPosAnalogChannel,absPosAnalogValue);

      std::cout << "Value at 0 degrees is: "<<absPosAnalogValue<<"\n";

      double softLLdeg,softULdeg;   
      softLLdeg = PMDUtils::RevsToDegs(CountsToUnits(revSoftLimitCounts));
      softULdeg = PMDUtils::RevsToDegs(CountsToUnits(fwdSoftLimitCounts));

      int mult = (int)softLLdeg / 5;  
      softLLdeg = mult*5;
      mult = (int)softULdeg / 5;
      softULdeg = mult*5;
      std::vector<std::vector<double> > vals(2);
      for (int i = softLLdeg; i <= softULdeg; i+=5) {
        profile = homingProfile;
        profile.pos = PMDUtils::DegsToRevs(i);
        SetProfile(profile);
        Move();
        
        //PMDUtils::DoSleep(1000);
        PMDint32 pos;
        GetActualPosition(pos);

        PMDuint16 absPosAnalogValue;
        ReadAnalog(absPosAnalogChannel,absPosAnalogValue);

        //std::cout << absPosAnalogValue<<" "<<PMDUtils::RevsToDegs(CountsToUnits(pos))<<"\n";

        vals[0].push_back(absPosAnalogValue);
        vals[1].push_back(PMDUtils::RevsToDegs(CountsToUnits(pos)));
      }
      PMDUtils::polyfit(vals,3);
      profile = homingProfile;
      profile.pos = PMDUtils::DegsToRevs(0);
      SetProfile(profile);
      Move();
  }
}

//----------------------------------------------------------------------------
void PMDAxisControl::FindMaxSpeed()
{

  SetProfileMode(PMDTrapezoidalProfile);

  PMDint32 lastpos;
  GetActualPosition(lastpos);

  int dir=1;
  if (PMDUtils::RevsToDegs(CountsToUnits(lastpos)) < 0)
    dir=-1;

  PMDint32 limit;
  if (dir==1)
    limit=revSoftLimitCounts;
  else limit=fwdSoftLimitCounts;

  MovePseudoVel( true, PMDUtils::DegsToRevs(10*dir), PMDUtils::DegsToRevs(20.0), cmdProfile.dec);

  PMDint32 pos;
  GetActualPosition(lastpos);
  double vel,acc;

  double curr_time, last_time;

  double maxvel=-1;
  double maxacc=-1;
  double lastvel=0;
  double start_time = PMDUtils::GetTime();
  last_time=0;
  std::cerr.precision(15);
  do {
    MovePseudoVel( false, PMDUtils::DegsToRevs(-50*dir), PMDUtils::DegsToRevs(80.0),0);
    GetActualPosition(pos);
    curr_time=PMDUtils::GetTime()-start_time;
    vel=fabs((pos-lastpos)/((curr_time-last_time)/1000.0));
    if (vel >= maxvel)
      maxvel = vel;
    if (lastvel != 0) {
      acc=fabs((vel-lastvel)/((curr_time-last_time)/1000.0));
      if (acc >= maxacc)
	maxacc = acc;
    }
    last_time=curr_time;
    lastpos=pos;
    lastvel=vel;
  } while (fabs(PMDUtils::RevsToDegs(CountsToUnits(pos-limit))) > 2);

  PMDUtils::DoSleep(1000);

  double first_max=maxvel;
  double first_max_acc=maxacc;
  maxvel=-1;
  maxacc=-1;
  dir=-dir;
  

  if (dir==1)
    limit=revSoftLimitCounts;
  else limit=fwdSoftLimitCounts;

  GetActualPosition(lastpos);
  start_time = PMDUtils::GetTime();
  last_time=0;
  lastvel = 0;
  do {
    MovePseudoVel( false, PMDUtils::DegsToRevs(-50*dir), PMDUtils::DegsToRevs(80.0),0);
    GetActualPosition(pos);
    curr_time=PMDUtils::GetTime()-start_time;
    vel=fabs((pos-lastpos)/((curr_time-last_time)/1000.0));
    if (vel >= maxvel)
      maxvel = vel;
    if (lastvel != 0) {
    acc=fabs((vel-lastvel)/((curr_time-last_time)/1000.0));
    if (acc >= maxacc)
      maxacc = acc;
    }
    last_time=curr_time;
    lastpos=pos;
    lastvel=vel;
  } while (fabs(PMDUtils::RevsToDegs(CountsToUnits(pos-limit))) > 2);

  maxvel = 0.8*min(maxvel,first_max);
  maxacc = 0.4*min(maxacc,first_max_acc);

  std::cerr << "Max velocity: "
      << floor(PMDUtils::RevsToDegs(CountsToUnits(maxvel)))
      << " degrees\nMax acceleration: "
      << floor(PMDUtils::RevsToDegs(CountsToUnits(maxacc))) << "degrees\n";

  PMDUtils::DoSleep(1000);

  Move(true,0,PMDint32(maxvel),PMDint32(maxacc),0,0);

}
