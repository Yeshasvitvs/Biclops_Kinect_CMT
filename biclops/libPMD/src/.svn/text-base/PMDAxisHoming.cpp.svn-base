#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#else
#include <stdlib.h>
#endif

#include <sstream>
#include <fstream>          // for homing file reading

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

using namespace std;

#undef dbgCout
#define dbgCout cout << classStr << "(" << (int)ctrlID << 'x' << GetAxisNumber() <<  ")::" << methodStr 

//----------------------------------------------------------------------------
void PMDAxisControl::SetHomedState(bool state)
{
    // Record the homing state in the controller.
    if (controller->HasMemory())
    {   PMDuint16 userIOData;
        ReadIO(UserIOAddress+GetAxisNumber(),userIOData);
        if (state)
            userIOData = userIOData | UIOMaskIsHomed;
        else
            userIOData = userIOData & ~UIOMaskIsHomed;
        WriteIO(UserIOAddress+GetAxisNumber(),userIOData);
    } else
    {   // If controller doesn't have memory, just set the capture source
        // to a value that is different from the controller reset default. 
      if (state)
        SetCaptureSource(PMDCaptureSourceHome);
      else 
        SetCaptureSource(!PMDCaptureSourceHome);
    }

    isHomed = state;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::GetHomedState()
{
    bool homedState = false;

    // Record the homing state in the controller.
    if (controller->IsConnected())
    {   
      if (controller->HasMemory())
        {   PMDuint16 userIOData;
            ReadIO(UserIOAddress+GetAxisNumber(),userIOData);
            homedState = (userIOData & UIOMaskIsHomed) != 0;
        } else
        {   // If controller doesn't have memory, just set the capture source
            // to a value that is different from the controller reset default. 
            PMDuint16 captureSource;
            GetCaptureSource(captureSource);
            homedState = ((tagPMDCaptureSource)captureSource) == PMDCaptureSourceHome;
        }
    }

    return homedState;
}

void PMDAxisControl::LoadRunFilter()
{
  SetFilter(runFilter);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::FindHome(bool forceHoming, bool writeHomeFile) {
    char methodStr[] = "FindHome: ";
    bool homingWasSuccessful = false;
    bool homingWithMotion = false;

    UpdateLimits();

    SetProfile(homeCfgCounts, true);

    if (forceHoming || !GetHomedState()) {
        ifDbgCout << "----------BEGIN HOMING----------\n";
        SetHomedState(false);

        // Set the axis gains for homing.
        SetFilter(homingFilter);

        // Since the current axis position is unknown, define it to be at zero.
        PMDint32 position;
        GetActualPosition(position);
        if (position != 0) {
            ifDbgCout << "Current position (" << position << ") being set to zero\n";
            SetActualPosition(0);
        }

        // Ensure the axis will not move when closed loop control is enabled.
	CountsProfile zero_move;
	GetProfile(zero_move,true);
	zero_move.pos=0;
	zero_move.vel=0;
	SetProfile(zero_move);

        // Turn on the motor amplifier.
        EnableAmp();

        // Start closed loop control with the specified profiling mode.
        SetProfileMode(PMDTrapezoidalProfile);
        SetMotorMode(PMDMotorOn);

        // Release the brake (if it exists)
        DisengageBrake();

        homingWithMotion = true;

        // Start the actual homing process
        switch (homingMode) {
        case ReverseLimit:
        case ForwardLimit:
        case BothLimits:
            homingWasSuccessful = FindHomeWithLimits();
            break;
        case HomeAndIndex:
            homingWasSuccessful = FindHomeWithHomeSensor();
            break;
        case BestWay:
            homingWasSuccessful = FindHomeBestWay();
            break;
        default: // case NoHoming
            SetHomingLimits(0,0,0,0);
            homingWasSuccessful = true;
            break;
        }
	
    } else {
        // Since a homing sequence wasn't necessary, just ensure that 
        // closed loop control is enabled.
        SetMotorMode(PMDMotorOn);
        EnableAmp();
        DisengageBrake();
        homingWasSuccessful = true;
    }

    // Record the fact that this axis is homed.
    // This is done by setting the chip's capture source to "home" 
    // because the chip default capture source is index and this signal
    // is not used for normal operation so having it set to a non 
    // default value is a persistent indication that homing was successful.
    if (homingWasSuccessful) {
      
        SetHomedState(true);
	    if (writeHomeFile)
	      WriteAxisStateToFile();
            double softLLdeg,softULdeg;
            softLLdeg = PMDUtils::RevsToDegs(CountsToUnits(revSoftLimitCounts));
            softULdeg = PMDUtils::RevsToDegs(CountsToUnits(fwdSoftLimitCounts));
		    ifDbgCout << "Homed successfully. Motion now limited to "
                      << revSoftLimitCounts << " .. " << fwdSoftLimitCounts
                      << " (" << softLLdeg << " .. " << softULdeg << " degrees)\n";

        // With homing complete, stop movement if a motion error occurs.
        SetAutoStopMode(PMDAutoStopEnabled);

        // At the completion of homing, load the run-time servo filter
        // and velocity profile and move to the run profile's start position.
        SetFilter(runFilter);

        PMDint32 position;
        GetActualPosition(position);
        if (homingWithMotion)
          runProfile.pos = 0;
        else
          runProfile.pos = CountsToUnits(position);
        SetProfile(runProfile);

        u_int32_t oldPeriod = GetMoveWaitPeriod();
        SetMoveWaitPeriod(250);
        Move();
        SetMoveWaitPeriod(oldPeriod);

    }

    ifDbgCout << "----------END HOMING----------\n";

    return homingWasSuccessful;

}

//----------------------------------------------------------------------------
void PMDAxisControl::GoToLimit(CountsProfile &p) {

    char methodStr[] = "GoToLimit: ";
    bool moveCompletedSuccessfully;
    PMDuint16 status;

    // If limit sensors exist then that's what defines the limit sought.
    // Set the capture source
    if (hasLimitSensors) {

        // Move away from the limit sensor if it is inactive.
        WithdrawFromLimit(p);

    }

    // Move toward the goal
    SetProfile(p,true);
    GetActualPosition(actProfileCounts.pos);
    u_int32_t oldPeriod = GetMoveWaitPeriod();
    SetMoveWaitPeriod(500);
    moveCompletedSuccessfully = Move();
    GetEventStatus(status);
    SetMoveWaitPeriod(oldPeriod);

    // Record limit position
    GetActualPosition(actProfileCounts.pos);

    if (moveCompletedSuccessfully) {
        ifDbgCout << "Desired goal attained at " << actProfileCounts.pos 
                  << "; presume limit not found\n";
    } else {
        ifDbgCout << "Motion interrupted at " << actProfileCounts.pos 
                  << "; presume limit was found\n";
    }

    // Back off of the limit
    WithdrawFromLimit(p);
}

//----------------------------------------------------------------------------
void PMDAxisControl::WithdrawFromLimit(CountsProfile &p) {
    char methodStr[] = "WithdrawFromLimit: ";

    u_int32_t oldPeriod = GetMoveWaitPeriod();
    SetMoveWaitPeriod(500);
    if (hasLimitSensors) {

        // Determine if both limits are asserted (an indication that both
        // limits are attached to the same physical switch/sensor).
        GetSignalStatus(signalStatus);
        bool bothLimitsAreAsserted = 
            !(signalStatus & PMDSignalPositiveLimitMask) &&
            !(signalStatus & PMDSignalNegativeLimitMask);
        if (bothLimitsAreAsserted) {
            // Disable limit sensor mode until after the move so that neither
            // sensors is active. Otherwise, the controller will not allow a 
            // move in either direction.
            SetLimitSwitchMode(PMDLimitDisabled);
        }

        // Record commanded pos for later replacement.
        PMDint32 lastCommandedPos = p.pos;

        // Compute direction and distance to "withdrawl" from limit sensors.
        GetActualPosition(p.pos);
        int posDelta = rangeOfMotion/10;    // move 10% of full range away from sensor
        if (lastCommandedPos > p.pos) posDelta = -posDelta;

        // Only withdrawl from limit sensor if the active sensor is in the
        // opposite direction of the desired withdrawl.
        if ((posDelta > 0 && !(signalStatus & PMDSignalNegativeLimitMask)) ||
            (posDelta < 0 && !(signalStatus & PMDSignalPositiveLimitMask))) {
            ifDbgCout << "Withdrawling from current pos because a limit sensor" 
                      << " is asserted.\n";
            p.pos += posDelta;
            SetProfile(p,true);
            Move();
        }

        // Turn back on limit sensor mode.
        if (bothLimitsAreAsserted) SetLimitSwitchMode(PMDLimitEnabled);

        // Restore the original goal position
        p.pos = lastCommandedPos;

    } else if (limitsMayBind) {

        // Set a more agressive output and error limit to overcome 
        // sticky limits while backing off the hard limit.
        const int MaxOL = 32767;
        SetMotorLimit(MaxOL);
        SetPositionErrorLimit(homingFilter.errorLim*2);

        // Back off the hard-stop by 10%
        PMDint32 actPos;
        GetActualPosition(actPos);
        if (p.pos < 0)
            p.pos = actPos + rangeOfMotion/10;
        else
            p.pos = actPos - rangeOfMotion/10;
        SetProfile(p,true);
        ifDbgCout << "Withdrawling from potentially binding limit...\n";
        if (Move()) {
            ifDbgCout << "Withdrawl succeeded\n";
        }
        else {
            ifDbgCout << "Withdrawl failed (limit too sticky?)\n";
        }

        // Reset gains
        SetMotorLimit(homingFilter.motorLim);
        SetPositionErrorLimit(homingFilter.errorLim);

    }
    SetMoveWaitPeriod(oldPeriod);

}

//----------------------------------------------------------------------------
bool PMDAxisControl::FindHomeWithLimits() {

    char methodStr[] = "FindHomeWithLimits: ";
    const unsigned int moveBoost = 2;
    PMDuint16 boostedMotorLimit = min((unsigned int)32767,homingFilter.motorLim*moveBoost);

    u_int32_t oldPeriod = GetMoveWaitPeriod();
    SetMoveWaitPeriod(500);

    // Force homing to hold position against hard stops to get a more accurate
    // measure of where the hard stop is.
    SetAutoStopMode(PMDAutoStopDisabled);

    // Load homing profile
    CountsProfile countsProfile = homingProfileCounts;

    // Find reverse limit & record its position
    if (homingMode == ReverseLimit || homingMode == BothLimits) {

        // If this limit is asserted, the axis must back off of it.
        // This is done by moving 10% in the opposite direction.
        GetSignalStatus(signalStatus);
        if (!(signalStatus & PMDSignalNegativeLimitMask)) {
            countsProfile.pos = rangeOfMotion/10;
            WithdrawFromLimit(countsProfile);
        }

        // Tell profiler to hold position against hard limit (if encountered)
        // until position can be recorded.
        SetAutoStopMode(PMDAutoStopDisabled);

        // Want to move enough in the limit direction to ensure that the limit
        // is surpassed so that either a hard limit is reached or the reverse
        // limit switch is reached. This is done by moving the specified 
        // range of motion PLUS twice the error limit.
        countsProfile.pos = -rangeOfMotion - 2*homingFilter.errorLim;
        ifDbgCout << "Attempting to move the entire range of motion (" 
                  << countsProfile.pos
                  << ") towards the -limit in expectation of hitting the limit\n";
        GoToLimit(countsProfile);
        revLimitCounts = actProfileCounts.pos;
        ifDbgCout << "Reverse limit encountered at " << revLimitCounts << endl;

        // Stop profiler from trying to drive up against hard limit.
        SetAutoStopMode(PMDAutoStopEnabled);
    }

    // Move more quickly most of the way toward the expected 
    // forward limit if both limits are used for homing.
    // Find positive limit (if desired)
    if (homingMode == BothLimits) {

        countsProfile.pos = revLimitCounts + rangeOfMotion*4/5;
        countsProfile.vel *= moveBoost;
        ifDbgCout << "Moving partially (to " << countsProfile.pos
                  << ") back towards expected +limit a little faster\n";
        SetProfile(countsProfile,true);

        // Boost components of profile and filter to facilitate the faster move.
        SetPositionErrorLimit(homingFilter.errorLim*moveBoost);
        SetMotorLimit(boostedMotorLimit);

        // Initiate move
        if (Move()) {
            ifDbgCout << "Quick partial forward move done\n";
        }
        else {
            ifDbgCout << "Quick partial forward move ended prematurely\n";
        }

        // Restore original motion parameters
        countsProfile.vel = homingProfileCounts.vel;
        SetPositionErrorLimit(homingFilter.errorLim);
        SetMotorLimit(homingFilter.motorLim);
    }

    // Find forward limit & record its position
    if (homingMode == ForwardLimit || homingMode == BothLimits) {

        // If this limit is asserted, the axis must back off of it.
        // This is done by moving 10% in the opposite direction.
        GetSignalStatus(signalStatus);
        if (!(signalStatus & PMDSignalPositiveLimitMask)) {
            GetActualPosition(countsProfile.pos);
            countsProfile.pos -= rangeOfMotion/10;
            WithdrawFromLimit(countsProfile);
        }

        // Tell profiler to hold position against hard limit (if encountered)
        // until position can be recorded.
        SetAutoStopMode(PMDAutoStopDisabled);

        // Want to go past the limit by at least the error limit to ensure
        // that the hard stop is reached and a motion error is triggered.
        countsProfile.pos = rangeOfMotion + 2*homingFilter.errorLim;
        ifDbgCout << "Attempting to move past the expected +limit (to " 
                  << countsProfile.pos
                  << ") in expectation of hitting a hard limit\n";
        GoToLimit(countsProfile);
        fwdLimitCounts = actProfileCounts.pos;
        ifDbgCout << "Forward limit detected at " << fwdLimitCounts << endl;

        // Stop profiler from trying to drive up against hard limit.
        SetAutoStopMode(PMDAutoStopEnabled);

    }

    // Compute the new home/zero position
    if (homingMode == ReverseLimit)
        homeDelta = revLimitCounts + calibratedHomeOffset;
    else if (homingMode == ForwardLimit)
        homeDelta = fwdLimitCounts + calibratedHomeOffset;
    else {
        // Homing fails if measured range of motion is not within a %age
        // of the predicted range of motion.
        PMDlong32 measuredRangeOfMotion = fwdLimitCounts - revLimitCounts;
        if (measuredRangeOfMotion > rangeOfMotion*1.1 || 
            measuredRangeOfMotion < rangeOfMotion*0.9) 
            return false;
        homeDelta = (revLimitCounts + fwdLimitCounts)/2 + calibratedHomeOffset;
    }

    // Make the actual position relative to the new home position.
    const float AdjustActualPositionReleaseVersion = 1.6f;
    if (controller->GetControllerVersion() < AdjustActualPositionReleaseVersion) {
        GetActualPosition(actProfileCounts.pos);
        SetActualPosition(actProfileCounts.pos-homeDelta);
    } else
        AdjustActualPosition(-homeDelta);

    // Move to the axis zero (home) position.
    countsProfile.pos = 0;
    countsProfile.vel *= moveBoost;
    ifDbgCout << "Moving to zero (home) position, offset by " << calibratedHomeOffset << endl;
    SetProfile(countsProfile,true);

    // Boost components of profile and filter to facilitate the faster move.
    SetPositionErrorLimit(homingFilter.errorLim*moveBoost);
    SetMotorLimit(boostedMotorLimit);
    SetPositionErrorLimit(homingFilter.errorLim*moveBoost);

    // Start moving.
    if (Move()) {
        GetActualPosition(actProfileCounts.pos);
        ifDbgCout << "final home position acheived, stopped at " 
                  << actProfileCounts.pos << endl;
    }
    else {
        GetActualPosition(actProfileCounts.pos);
        ifDbgCout << "final home position not acheived, stopped at" 
                  << actProfileCounts.pos << endl;
        ifDbgCout << "However, homing was still successful." << endl;
    }

    // Restore original homing rates
    countsProfile.vel = homingProfileCounts.vel;
    SetPositionErrorLimit(homingFilter.errorLim);
    SetMotorLimit(homingFilter.motorLim);

    // Recompute the hard limits relative to what is now the zero position.
    if (homingMode == ReverseLimit) {
        revLimitCounts = revLimitCounts - homeDelta;
        fwdLimitCounts = revLimitCounts + rangeOfMotion;
    } else if (homingMode == ForwardLimit) {
        fwdLimitCounts = fwdLimitCounts - homeDelta;
        revLimitCounts = fwdLimitCounts - rangeOfMotion;
    } else {
        revLimitCounts = revLimitCounts - homeDelta;
        fwdLimitCounts = fwdLimitCounts - homeDelta;
    }

    // Restore original move wait period
    SetMoveWaitPeriod(oldPeriod);

    return true;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::FindHomeWithHomeSensor() {

    char methodStr[] = "FindHomeWithHomeSensor: ";
    PMDint32 captureValue;
	bool captureValueIsValid = false;

    u_int32_t oldPeriod = GetMoveWaitPeriod();
    SetMoveWaitPeriod(500);

    // Load homing profile
    CountsProfile countsProfile = homingProfileCounts;

    // Set high speed capture for home signal and re-arm capture.
    SetCaptureSource(PMDCaptureSourceHome);
    ResetEventStatus(~PMDEventCaptureReceivedMask);
    GetCaptureValue(captureValue);   // re-arm

    // May want to change the following logic to look for hard limits while
    // looking for home sensor and recording their positions when encountered.
    // Could also make Move() command accept mask parameter to tell it what
    // to look for when deciding whether a move is complete or not. The move
    // command would default to motion complete and motion error.

    // Find hard limits if they are indicated to exist.
    // Since hard limits are not explorable, exercise axis to find
    // home signal by sweeping over the defined range of motion.

	// Is the home sensor active?
	PMDuint16 signalStatus;
	GetSignalStatus(signalStatus);
	if (signalStatus & PMDSignalEncoderHomeMask) {

		// Home sensor is active.
        // Back off the sensor so it's edge can be located.
		GetActualPosition(captureValue);
        countsProfile.pos = captureValue - countsPerEncoderCycle;
	SetProfile(countsProfile, true);
		ifDbgCout << "Home sensor active, so move off it by 1 encoder cycle ("
            << (int) countsPerEncoderCycle << " counts = "
            << PMDUtils::RevsToDegs(CountsToUnits(countsPerEncoderCycle)) 
            << "deg)\n";
		Move();

    }

    // Start positive rotation in search of the home sensor
    countsProfile.pos = (PMDint32)rangeOfMotion;
	SetProfile(countsProfile,true,false);
	ifDbgCout << "Sweeping axis in + direction until home is seen "
        << " or a hard limit is reached\n";
    GetCaptureValue(captureValue);
	MoveWithCaptureInterrupt();

	// Was the home signal captured during positive rotation?
	GetEventStatus(eventStatus);
	if (!CaptureEventTriggered()) {

		// Did not see the home signal.
        GetActualPosition(captureValue);
		ifDbgCout << "Didn't find home signal but encountered hard limit at "
            << captureValue << endl;

		// Reverse direction and aim for hard limit with the expectation of
        // seeing the home signal first.
		if (hasHardLimits && revSoftLimitCounts > -countsPerAxisCycle/2)
			countsProfile.pos = -revSoftLimitCounts;
		else
			countsProfile.pos = -(int)rangeOfMotion/2;
		ifDbgCout << "Looking for home in - direction\n";
		SetProfile(countsProfile,true,false);
		MoveWithCaptureInterrupt();

		// Was the home signal captured during negative rotation?
		GetEventStatus(eventStatus);
		if (!CaptureEventTriggered()) {
			ifDbgCout 
				<< "Didn't find home signal in - direction\n"
				<< "Home sensor isn't working or axis didn't move!\n";
			SetMoveWaitPeriod(oldPeriod);
		} else {
			GetCaptureValue(captureValue);
			captureValueIsValid = true;
			ifDbgCout << "Saw the home signal at " << captureValue 
                << " counts (" << PMDUtils::RevsToDegs(CountsToUnits(captureValue)) 
                << "deg) with negative rotation\n";
		}

	} else {
		GetCaptureValue(captureValue);
		captureValueIsValid = true;
		ifDbgCout << "Saw the home signal at " << captureValue 
            << " counts (" << PMDUtils::RevsToDegs(CountsToUnits(captureValue)) 
            << "deg) with positive rotation\n";
	}

	// If the home signal was detected, find the nearest encoder index pulse.
	if (captureValueIsValid) {

		// Clear local capture flag for the index search.
		captureValueIsValid = false;

		// Rotate negative half an encoder revolution from the capture position.
		countsProfile.pos = captureValue - countsPerEncoderCycle/2;
		countsProfile.acc += 1;
		SetProfile(countsProfile, true);
		ifDbgCout << "Moving back to 1/2 encoder rev  (" << countsProfile.pos 
            << " counts (" << PMDUtils::RevsToDegs(CountsToUnits(countsProfile.pos)) 
            << "deg) from home sensor\n";
		Move();

		// Set high speed capture for index signal and re-arm capture.
		SetCaptureSource(PMDCaptureSourceIndex);
		GetCaptureValue(captureValue);
		ResetEventStatus(~PMDEventCaptureReceivedMask);

		// Rotate one whole encoder cycle in the forward direction.
		ifDbgCout << "Sweeping 1 encoder cycle (" << (int)countsPerEncoderCycle
            << "counts = " << PMDUtils::RevsToDegs(CountsToUnits(countsPerEncoderCycle))
            << "deg) in + dir to detect index pulse\n";
		countsProfile.pos += countsPerEncoderCycle;
		SetProfile(countsProfile, true);
		MoveWithCaptureInterrupt();

		// Was the index signal detected?
		GetEventStatus(eventStatus);
		if (!CaptureEventTriggered()) {

			// Did not see the index signal.
			ifDbgCout << "Didn't find index signal near the home sensor.\n";
			SetMoveWaitPeriod(oldPeriod);

		} else {
			// Record the captured index value
			GetCaptureValue(captureValue);
			captureValueIsValid = true;
			ifDbgCout << "Saw the index pulse at " << captureValue 
                << " counts (" << PMDUtils::RevsToDegs(CountsToUnits(captureValue))
                << "deg)\n";

			// Change the actual position to be relative to the captured 
            // location. For example, if the index is detected at 50, then 
            // the zero position must be adjusted by -50 so that position is 
            // the new zero. Also adjust the position by the known error 
            // between the index pulse and true home.
			AdjustActualPosition(-captureValue-calibratedHomeOffset);

		}
	} else {
		ifDbgCout << "Homing failed\n";
	}

    return captureValueIsValid;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::FindHomeBestWay()
{
    char methodStr[] = "FindHomeBestWay: ";
    bool foundHome = false;
    PMDint32 zeroPos = 0;
    PMDint32 motionLL,motionUL;

    // Set the initial homing move direction towards the negative limit
    // unless an analog sensor exists.
    PMDint32 absPosEstimate;
    int moveDir;
    if (hasAnalogSensor)
    {   // Set the initial direction of motion towards zero.

        //First move closer to where home may be (end of analog strip
        //may be noisy)
        PMDuint16 absPosAnalogValue;
        ReadAnalog(absPosAnalogChannel,absPosAnalogValue);
        absPosEstimate = EstimateAbsPosFromAnalogSensor(absPosAnalogValue);
        SetActualPosition(absPosEstimate);

        Profile profile = homingProfile;
        profile.pos=0;
        SetProfile(profile, true);
        ifDbgCout << "Abs pos sensor value of " << absPosAnalogValue
                  << " implies position of " << absPosEstimate
                  << " (" << PMDUtils::RevsToDegs(CountsToUnits(absPosEstimate))
                  << " degs). Move to expected zero before homing\n";
        Move();
      
        ReadAnalog(absPosAnalogChannel,absPosAnalogValue);
        absPosEstimate = EstimateAbsPosFromAnalogSensor(absPosAnalogValue);
        //moveDir = (absPosAnalogValue > analogZero) ? -1 : 1;
        moveDir = (PMDUtils::RevsToDegs(CountsToUnits(absPosEstimate)) > 0) ? -1 : 1;
        ifDbgCout << "Abs pos sensor value of " << absPosAnalogValue
                  << " implies position of " << absPosEstimate
                  << " (" << PMDUtils::RevsToDegs(CountsToUnits(absPosEstimate)) 
                  << " degs) so start homing in " 
                  << ((moveDir > 0) ? '+':'-') << " direction\n";

        // Limit homing motion based on analog estimated position
        if (abs(absPosEstimate) < (rangeOfMotion/10))
        {   motionUL =  rangeOfMotion/10;
            motionLL = -motionUL;
        } else if (absPosEstimate > 0)
        {   motionLL = -absPosEstimate - rangeOfMotion/20;
            motionUL = rangeOfMotion/20;
        } else
        {   motionUL = -absPosEstimate + rangeOfMotion/20;
            motionLL = -((int)rangeOfMotion)/20;
        }
    } else
    {   ifDbgCout << "No abs pos sensor so start homing in - direction\n";
        moveDir = -1;
        motionUL = rangeOfMotion;
        motionLL = -(int)rangeOfMotion;
    }
    ifDbgCout << "Homing range of motion limited to "
        << motionLL << ".." << motionUL << " counts ("
        << PMDUtils::RevsToDegs(CountsToUnits(motionLL)) << ".."
        << PMDUtils::RevsToDegs(CountsToUnits(motionUL)) << " deg)\n";

    // Choosing the best homing method starts with examining the encoder
    // index characteristics (basically, is there zero, one, or more?).
    int indexCount = (hasEncoderIndex) ? encoderCyclesPerAxisCycle : 0;
    ifDbgCout << indexCount << " index" << ((indexCount == 1) ? ", ":"es, ")
        << (hasHomeSensor   ? "has " : " no ") << "homeSensor, "
        << (hasHardLimits   ? "has " : " no ") << "hardLimits, "
        << (hasLimitSensors ? "has " : " no ") << "limitSensors\n";
    switch (indexCount)
    {   case 0:     // no encoder index
            if (hasHomeSensor)          // 0 index, have home
            {   foundHome = FindHomeCenter(moveDir,zeroPos,motionLL,motionUL);
            } else if (hasHardLimits)   // 0 index, no home, yes limits
            {   moveDir = -1;           // Force move toward neg limit
                PMDint32 relativeGoal = moveDir*(fwdLimitCounts - revLimitCounts);
                foundHome = FindLimit(relativeGoal,zeroPos);
            } else                      // 0 index, no home, no limits
            {   GetActualPosition(zeroPos);
                foundHome = true;
            }
            break;
        case 1:     // one encoder index per axis rotation
            foundHome = FindIndex(moveDir,zeroPos,motionLL,motionUL);
            break;
        default:     // multiple encoder indexes per axis rotation
            PMDint32 homeCenter,indexPos;
            if (hasHomeSensor)         // yes index, yes home
            {   if (HomeSensorIsAsserted()) // find home first since on it already
                {   ifDbgCout << "Home sensor is asserted, so find "
                        << "home center then index\n";
                    if (FindHomeCenter(moveDir,homeCenter,motionLL,motionUL))
                        foundHome = FindIndex(moveDir,indexPos,motionLL,motionUL);
                    else
                        foundHome = false;
                } else                  // find index, then home
                {   ifDbgCout << "Home sensor is not asserted, so find "
                        << "index then home center\n";
                    if (FindIndex(moveDir,indexPos,motionLL,motionUL))
                        foundHome = FindHomeCenter(moveDir,homeCenter,motionLL,motionUL);
                    else
                        foundHome = false;
                }

                // Determine zero position by calculating the index that is
                // nearest home center.
                if (foundHome) 
                {   double encRevsFromHomeToIndex =
                        (double)(homeCenter-indexPos)/countsPerEncoderCycle;
                    zeroPos = indexPos + 
                        PMDUtils::Rnd(encRevsFromHomeToIndex) * countsPerEncoderCycle;
                    ifDbg 
                    {   int indexOffsetFromHome = abs(homeCenter-zeroPos);
                        dbgCout << "Home/index separation = " 
                            << indexOffsetFromHome;

                        if (indexOffsetFromHome > (int)countsPerEncoderCycle/3)
                        {   cout << " > 1/3 (" << int(countsPerEncoderCycle/3)
                                << ") encoder cycle offset.\n";
                        } else cout << endl;

                    }
                }

            } else
            {   foundHome = false;
                //multiple indexes without home sensor is not useful
                // could just default to using hard limits, assuming they
                // exist
            }
            break;
    }

    if (foundHome) 
    {
        // Modify the measured zero position with the offset measured during
        // calibration to get the true zero position and modify the axis zero.
        zeroPos += calibratedHomeOffset;
        AdjustActualPosition(-zeroPos);

        ifDbgCout << "New home is " << zeroPos << " counts away from start pos\n";
    } else
        ifDbgCout << "Unable to find home\n";
    return foundHome;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::FindIndex(int moveDir,PMDint32 &indexPos, 
                               PMDint32 motionLL, PMDint32 motionUL) 
{
    char methodStr[] = "FindIndex: ";
    PMDint32 pos;
    PMDint32 localMotionLL,localMotionUL;
    PMDint32 captureValue;

    // The home sensor is redundant with only one index, but we can
    // minimize motion if the sensor is asserted at the current
    // position because we know the index pulse is nearby.
    GetActualPosition(pos);
//    if (hasHomeSensor && HomeSensorIsAsserted())
//    {   // Limit index search motion to a fraction of full range.
//        ifDbgCout << "Search for index started with home sensor asserted\n";
//        localMotionLL = pos - homeSensorWidth;
//        localMotionUL = pos + homeSensorWidth;
//    } else
    {   localMotionLL = pos - countsPerEncoderCycle*2;
        localMotionUL = pos + countsPerEncoderCycle*2;
    }
    SetCaptureSource(PMDCaptureSourceIndex);

    // Get the motion profile parameters for homing.
    CountsProfile countsProfile;
    GetProfile(countsProfile,true);

    countsProfile.pos = (moveDir > 0) ? localMotionUL : localMotionLL;
    SetProfile(countsProfile, true);
    ifDbgCout << "Move towards " << countsProfile.pos 
              << " starting @ " << pos << endl;

    // Arm the event trigger
    GetCaptureValue(captureValue);   // re-arm
    MoveWithCaptureInterrupt();

    // Did the capture event trigger?
    if (!CaptureEventTriggered())
      {   // Capture event didn't trigger
	// Try going other direction
	GetActualPosition(pos);
	ifDbgCout << "Index not found in current direction (currently @ "<<pos<<")\n"; 
	moveDir *= -1;
	countsProfile.pos = (moveDir > 0) ? localMotionUL : localMotionLL;
	ifDbgCout << "Move towards " << countsProfile.pos 
		  << " starting @ " << pos << endl;
	SetProfile(countsProfile, true);
	MoveWithCaptureInterrupt();
	if (!CaptureEventTriggered()) {
	  GetActualPosition(pos); 
	  ifDbgCout << "Index not found in current direction (currently @ "<<pos<<")\n"; 
	  cout << "+++INDEX NOT FOUND in range (" << localMotionLL <<
	    ".." << localMotionUL << ")\n";
	  return false;
	}
      }
    // Record the captured value as the actual index position.

    SetProfile(countsProfile, true);
    GetActualPosition(pos);
    GetCaptureValue(indexPos);
    ifDbgCout << "+++INDEX FOUND @ " << indexPos << " (motion stopped at " 
        << pos << ")\n";
	return true;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::FindHomeCenter(int &moveDir, PMDint32 &homeCenter,
                                    PMDint32 motionLL, PMDint32 motionUL)
{
    char methodStr[] = "FindHomeCenter: ";
    CountsProfile profile;      // Used to command moves

    ifDbgCout << "Home sensor width is expected to be " 
              << (int) homeSensorWidth << endl;

    GetProfile(profile,true);

    // Since we're searching for home signal transitions, set the motion 
    // capture trigger to reflect that.
    SetCaptureSource(PMDCaptureSourceHome);

    // Find both home sensor edges
    PMDint32 home[2];
    const int edgeFindAttemptLimit = 2;
    int edgeFindAttempts = 0;
    bool validEdgesFound = false;

    // Invert the home sensor signal sense so the capture event always
    // detects the sensor edge as motion moves past the sensor, eliminating
    // the chance that motion will stop on the sensor, which requires extra
    // logic to figure out which direction to go next.
    PMDuint16 normalHomeSignalSense;
    GetSignalSense(normalHomeSignalSense);
//    SetSignalSense(normalHomeSignalSense ^ PMDSignalEncoderHomeMask);
    PMDint32 startPos;
    GetActualPosition(startPos);
    profile.pos = startPos;
    PMDint32 localLL,localUL;
    bool homeAssertedAtStart = HomeSensorIsAsserted();
    if (homeAssertedAtStart) {
        localLL = startPos - homeSensorWidth;
        localUL = startPos + homeSensorWidth;
    } else {
        localLL = motionLL;
        localUL = motionUL;
    }
    
    // Start looking for edges.
    do
    {
        for (int edge = 0; edge < 2; edge++)
        {
            if (HomeSensorIsAsserted()) 
            {   // Get off the home sensor before searching for the edge

                // Determine the move goal position (actually moving the 
                // opposite direction to get off the home sensor).
//                GetActualPosition(profile.pos);
                profile.pos = (moveDir > 0) ? localLL : localUL;
                ifDbgCout << "Home sensor is asserted, so moving off (other direction) towards "
			  << profile.pos << " before continuing homing" << endl;
                SetProfile(profile,true);
                Move();

//                GetPosition(profile.pos);
                ifDbgCout << "Commanded to "<<profile.pos<<".  ";
                GetActualPosition(profile.pos);
                ifDbgCout << "Stopped at "<<profile.pos<<"\n";

            }


            // Determine the move goal position.
            profile.pos =  (moveDir < 0) ? localLL : localUL;
            ifDbgCout << "Moving towards " << profile.pos << " (" <<
                PMDUtils::RevsToDegs(CountsToUnits(profile.pos)) << " deg) in search of " 
                << ((edge == 0) ? "first" : "second") << " edge \n";

            SetProfile(profile, true);

            // Arm the home sensor trigger
            PMDint32 captureValue;
            GetCaptureValue(captureValue);   // re-arm

            // Start motion and stop if a capture event occurs.
            MoveWithCaptureInterrupt();

//            GetPosition(profile.pos);
            ifDbgCout << "Commanded to "<<profile.pos<<".  ";
            GetActualPosition(profile.pos);
            ifDbgCout << "Stopped at "<<profile.pos<<"\n";

	    
            // Did we see the edge?
            if (CaptureEventTriggered()) {
                GetCaptureValue(home[edge]);   // Yay, record its position.
            }
            else
            {   // Didn't see the edge, so reverse direction and try again
                moveDir *= -1;
                profile.pos =  (moveDir < 0) ? localLL : localUL;
                ifDbgCout << ((edge == 0) ? "First":"Second") 
                          << " edge not found. Try search toward "
                          << profile.pos << endl;
                SetProfile(profile, true);
                MoveWithCaptureInterrupt();

//                GetPosition(profile.pos);
	            ifDbgCout << "Commanded to "<<profile.pos<<".  ";
                GetActualPosition(profile.pos);
                ifDbgCout << "Stopped at "<<profile.pos<<"\n";

                if (!CaptureEventTriggered())   // Did we see the edge this time?
                {   ifDbgCout << "Unable to find "
                        << ((edge == 0) ? "first":"second") << " edge\n";
                    SetSignalSense(normalHomeSignalSense);
                    return false;
                }
                GetCaptureValue(home[edge]);
            }
            GetActualPosition(profile.pos);
            ifDbgCout << "Found " << ((edge == 0) ? "first":"second") 
                << " edge at " << home[edge] << " (stopped at " << profile.pos 
                << ")\n";

            // Switch directions to find other edge.
            moveDir *= -1;
        }

        // Validate home edges by verifying that they are separated by at
        // no less than half the expected sensor width.
        validEdgesFound = (abs(home[0] - home[1]) > homeSensorWidth/2);
        if (!validEdgesFound) {
            edgeFindAttempts++;
            ifDbgCout << "Two home sensor edges (" << home[0] << ", " 
            << home[1] << ") are closer than half the expected sensor width(" 
            << (int)homeSensorWidth/2 << "). Repeat edge search.\n";

	    localLL = localLL - homeSensorWidth;
	    localUL = localUL + homeSensorWidth;
	    
	    if (localLL < motionLL)
	      localLL = motionLL;

	    if (localUL > motionUL)
	      localUL = motionUL;


        }

    } while (!validEdgesFound && edgeFindAttempts <= edgeFindAttemptLimit);

    // Return signal sense to it's original setting.
    SetSignalSense(normalHomeSignalSense);

    // Compute the center between the two home edges.
    if (validEdgesFound)
    {   homeCenter = (home[0] + home[1])/2;
        int homeMin = min(home[0],home[1]);
        int homeMax = max(home[0],home[1]);
        ifDbgCout << "+++HOME FOUND @ " << homeCenter  
            <<" (min,max,delta) = (" << homeMin << ',' << homeMax 
            << ',' << homeMax-homeMin << ")\n";

    }
    return validEdgesFound;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::FindLimit(PMDint32 relativeGoal, PMDint32 &foundLimitPos)
{
    char methodStr[] = "FindLimit: ";
    // Make sure that if limit sensors exist, they're set up to trigger
    // motion stop.

    // Move in desired direction (moveDir) until a position error occurs.
    CountsProfile profile;
    GetProfile(profile,true);   // get homing move parameters
    GetActualPosition(profile.pos);
    profile.pos += relativeGoal;    // Move full range of motion
    SetProfile(profile, true);
    ifDbgCout << "Searching for limit by moving to goal " << profile.pos << endl;
    Move();

    // If limit sensors exist, motion should have stopped at one of them.
    // Repeat move towards limit at slower speed with limits sensor protection off.
    if (hasLimitSensors) {
        // Verify that limit sensor was found
        PMDuint16 status;
        GetSignalStatus(status);
        bool posLimAsserted = (status & PMDSignalPositiveLimitMask) == 0;
        bool negLimAsserted = (status & PMDSignalNegativeLimitMask) == 0;
        GetEventStatus(status); //for temp debug BTW
        if ((relativeGoal < 0 && negLimAsserted) || 
            (relativeGoal > 0 && posLimAsserted)) {
            DisableLimitSensorProtection();
            GetActualPosition(profile.pos);
            ifDbgCout << "Motion stopped at " << profile.pos 
                      << " due to assertion of " 
                      << (posLimAsserted ? "pos" : "neg")
                      << " limit sensor\n";
            ifDbgCout << "Moving more slowly toward hard limit\n";
            profile.vel /=2;
            SetProfile(profile, true);
            ResetEventStatus(0);
            Move();
            GetActualPosition(profile.pos);
            EnableLimitSensorProtection();
        }
    }


    GetPosition(profile.pos);
    ifDbgCout << "Commanded to "<<profile.pos<<".  ";
    GetActualPosition(profile.pos);
    ifDbgCout << "Stopped at " << profile.pos 
              << "(" << PMDUtils::RevsToDegs(CountsToUnits(profile.pos))
              << " deg)" << endl; 
    if (PositionErrorOccured()) {
        ifDbgCout << "Found limit\n";
        foundLimitPos = profile.pos;
        return true;
    } else {
        ifDbgCout << "Did not find limit" << endl;
        return false;
    }
}

