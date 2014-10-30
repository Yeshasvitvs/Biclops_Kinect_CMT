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
const char TokenController[] = "controller";
const char TokenMotor[] = "[motor]";
const char TokenEncoder[] = "[encoder]";
const char TokenAnalog[] = "[analog sensor]";
const char TokenHomingParams[] = "[homing parameters]";
const char TokenHomingProfile[] = "[homing profile]";
const char TokenHomingFilter[] = "[homing servo filter]";
const char TokenProfile[] = "[run profile]";
const char TokenFilter[] = "[run servo filter]";
const char TokenPhysical[] = "[physical]";
const char AxisToken[] = "axisNumber";
const char TokenDebugLevel[] = "debugLevel";
bool PMDAxisControl::ReadConfig(FILE *file, char *token) {
    bool tokenRecognized = true;
    unsigned int uval;

    // Need to look for a stop token if parser saw start token upon entry.
    bool lookForStopToken = (strcmp(token,StartToken) == 0);

    // Prime token stream.
    Parser::ReadToken(file,token);

    // Parse the stream.
    do {
        if (!strcmp(token,PMDUtils::TokenUniqueID)) {
            Parser::ReadToken(file,token);
            strcpy(uniqueID,token);
            Parser::ReadToken(file,token);
        } else if (!strcmp(token,TokenDebugLevel)) {
            Parser::ReadToken(file,token);
            if (sscanf(token,"%u",&uval) == 1) {
                SetDebugLevel(uval);
            } else tokenRecognized = false;
            Parser::ReadToken(file,token);
        } else if (!strcmp(token,TokenController)) {

            // Get the name of the associated controller.
            Parser::ReadToken(file,token);

            // Get access to the named controller and attach this axis to it.
            ControllerCollection *controllers = PMDControllers::Instance();
            controller = (*controllers)[token];
            if (controller != NULL) {
                controller->AddAxis(this);
            } else {
                tokenRecognized = false;
                cerr << "PMDAxisControl::ReadConfig unrecognized controller " << token << endl;
            }
            Parser::ReadToken(file,token);

        } else if (!strcmp(token,TokenMotor)) {
            ReadMotorParameters(file,token);
        } else if (!strcmp(token,TokenEncoder)) {
            ReadEncoderParameters(file,token);
        } else if (!strcmp(token,TokenAnalog)) {
            ReadAbsPosParameters(file,token);
        } else if (!strcmp(token,TokenHomingParams)) {
            ReadHomingParameters(file,token);
        } else if (!strcmp(token,TokenHomingProfile)) {
            ReadProfile(file,token,homingProfile);
            UnitsToCounts(homingProfile,homingProfileCounts);
	    homeCfgCounts= homingProfileCounts;
        } else if (!strcmp(token,TokenHomingFilter)) {
            ReadFilter(file,token,homingFilter);
//            activeFilter = homingFilter;
        } else if (!strcmp(token,TokenProfile)) {
            ReadProfile(file,token,runProfile);
            cmdProfile = runProfile;
        } else if (!strcmp(token,TokenFilter)) {
            ReadFilter(file,token,runFilter);
        } else if (!strcmp(token,TokenPhysical)) {
            ReadPhysical(file,token);
        } else if (!strcmp(token,AxisToken)) {
            Parser::ReadToken(file,token);
            if (sscanf(token,"%u",&uval) == 1 && 
                uval >= PMDAxis1 && uval <= PMDAxis4) {
                SetAxis(uval);
                Parser::ReadToken(file,token);
            } else
                tokenRecognized = false;
        } else tokenRecognized = false;

    } while (!feof(file) && tokenRecognized);

    // Record the results of whether configuration data was successfully 
    // loaded for this axis. Only an axis that has been successfully loaded
    // may be controlled.
    if (lookForStopToken)
        configurationWasLoaded = !strcmp(token,EndToken);
    else
        configurationWasLoaded = feof(file) != 0;
    return configurationWasLoaded;
}


//----------------------------------------------------------------------------
const char TokenActiveHigh[] = "ActiveHigh";
const char TokenActiveLow[] = "ActiveLow";
bool PMDAxisControl::ReadPolarity(FILE *file, char *token, Polarity &polarity) {
    bool tokenRecognized = true;
    Parser::ReadToken(file,token);
    if (!strcmp(token,TokenActiveHigh))
        polarity = ActiveHigh;
    else if (!strcmp(token,TokenActiveLow))
        polarity = ActiveLow;
    else
        tokenRecognized = false;
    return tokenRecognized;
}

//----------------------------------------------------------------------------
const char TokenCounts[] = "counts";
bool PMDAxisControl::ReadAngle(FILE *file, char *token, double &angle) {
    bool tokenRecognized = true;

    // Get the numeric value and save it in a local buffer.
    Parser::ReadToken(file,token);
    char valToken[10];
    strcpy(valToken,token);

    // Get the units of measure for the angle value and convert it to revolutions.
    float fangle;
    Parser::ReadToken(file,token);
    if (strcmp(token,PMDUtils::TokenDegrees) == 0) {
        sscanf(valToken,"%f",&fangle);
        angle = PMDUtils::DegsToRevs((double)fangle);
    } else if (strcmp(token,PMDUtils::TokenRadians) == 0) {
        sscanf(valToken,"%f",&fangle);
        angle = PMDUtils::RadsToRevs((double)fangle);
    } else if (strcmp(token,PMDUtils::TokenRevolutions) == 0) {
        sscanf(valToken,"%f",&fangle);
        angle = (double)fangle;
    } else if (strcmp(token,TokenCounts) == 0) {
        int countAngle;
        sscanf(valToken,"%d",&countAngle);
        angle = CountsToUnits(countAngle);
    } else
        tokenRecognized = false;
    return tokenRecognized;
}

//----------------------------------------------------------------------------
const char TokenMotorType[] = "motortype";
const char TokenBrushed[] = "brushed";
const char TokenBrushless[] = "brushless";
const char TokenPoleCount[] = "polecount";
const char TokenAmpType[] = "amptype";
const char TokenAmpPolarity[] = "ampPolarity";
const char TokenPWMSignMag[] = "PWMSignMag";
const char TokenPWM5050[] = "PWM50/50";
const char TokenHasBrake[] = "hasBrake";
const char TokenPhaseInit[] = "phaseInit";
const char TokenAlgorithmic[] = "Algorithmic";
const char TokenHallBased[] = "HallBased";
const char TokenInvertHalls[] = "invertHalls";
const char TokenCommutationMode[] = "commutationMode";
const char TokenSinusoidal[] = "Sinusoidal";
const char TokenMicrostepping[] = "Microstepping";
const char TokenInvertMotorOut[] = "invertMotorOut";
void PMDAxisControl::ReadMotorParameters(FILE *file, char *token) {
    bool tokenRecognized = true;
    Parser::ReadToken(file,token);
    do {
        if (!strcmp(token,TokenMotorType)) {
            Parser::ReadToken(file,token);
            if (!strcmp(token,TokenBrushed))
                isBrushless = false;
            else if (!strcmp(token,TokenBrushless))
                isBrushless = true;
            else
                tokenRecognized = false;
        } else if (!strcmp(token,TokenPoleCount)) {
            Parser::ReadToken(file,token);
            unsigned int ui;
            tokenRecognized = sscanf(token,"%u",&ui) == 1;
            poleCount = (unsigned short) ui;
        } else if (!strcmp(token,TokenAmpType)) {
            Parser::ReadToken(file,token);
            if (!strcmp(token,TokenPWMSignMag))
                ampType = PMDMotorOutputPWMSignMagnitude;
            else if (!strcmp(token,TokenPWM5050))
                ampType = PMDMotorOutputPWM5050Magnitude;
            else
                tokenRecognized = false;
        } else if (!strcmp(token,TokenAmpPolarity)) {
            tokenRecognized = ReadPolarity(file,token,ampPolarity);
        } else if (!strcmp(token,TokenHasBrake)) {
            tokenRecognized = Parser::ReadBool(file,token,hasBrake);
        } else if (!strcmp(token,TokenPhaseInit)) {
            Parser::ReadToken(file,token);
            if (!strcmp(token,TokenAlgorithmic))
                phaseInitMode = PMDPhaseInitAlgorithmic;
            else if (!strcmp(token,TokenHallBased))
                phaseInitMode = PMDPhaseInitHallBased;
            else
                tokenRecognized = false;
        } else if (!strcmp(token,TokenCommutationMode)) {
            Parser::ReadToken(file,token);
            if (!strcmp(token,TokenHallBased))
                commutationMode = PMDCommutationModeHallBased;
            else if (!strcmp(token,TokenSinusoidal))
                commutationMode = PMDCommutationModeSinusoidal;
            else if (!strcmp(token,TokenMicrostepping))
                commutationMode = PMDCommutationModeMicrostepping;
            else
                tokenRecognized = false;
        } else if (!strcmp(token,TokenInvertHalls)) {
            tokenRecognized = Parser::ReadBool(file,token,invertHalls);
        } else if (!strcmp(token,TokenInvertMotorOut)) {
            tokenRecognized = Parser::ReadBool(file,token,invertMotorOut);
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);
}

//----------------------------------------------------------------------------
const char TokenHasEncoderIndex[] = "hasEncoderIndex";
const char TokenCountsPerEncoderCycle[] = "counts/encodercycle";
const char TokenEncoderCyclesPerAxisCycle[] = "encodercycles/axiscycle";
const char TokenMotorCyclesPerEncoderCycle[] = "motorcycles/encodercycle";
void PMDAxisControl::ReadEncoderParameters(FILE *file, char *token) {
    bool tokenRecognized = true;
    Parser::ReadToken(file,token);
    float ftemp;
    do {
        if (!strcmp(token,TokenHasEncoderIndex)) {
            tokenRecognized = Parser::ReadBool(file,token,hasEncoderIndex);
        } else if (!strcmp(token,TokenCountsPerEncoderCycle)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&countsPerEncoderCycle) == 1;
        } else if (!strcmp(token,TokenEncoderCyclesPerAxisCycle)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%f",&ftemp) == 1;
            encoderCyclesPerAxisCycle = (double)ftemp;
        } else if (!strcmp(token,TokenMotorCyclesPerEncoderCycle)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%f",&ftemp) == 1;
            motorCyclesPerEncoderCycle = (double)ftemp;
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);

    // Recompute derived values
    countsPerAxisCycle = countsPerEncoderCycle*encoderCyclesPerAxisCycle;
    countsPerMotorCycle = countsPerEncoderCycle/motorCyclesPerEncoderCycle;
    rangeOfMotion = (u_int32_t)countsPerAxisCycle;  // default to 1 axis cycle

}

//----------------------------------------------------------------------------
const char TokenCoefficients[] = "coefficients";
const char TokenAbsPosAnalogChannel[] = "analogChannel";
void PMDAxisControl::ReadAbsPosParameters(FILE *file, char *token) {
    bool tokenRecognized = true;
    Parser::ReadToken(file,token);
    bool gotChannel = false;
    bool gotCoefficients = false;
    do {
        if (!strcmp(token,TokenAbsPosAnalogChannel)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%hu",&absPosAnalogChannel) == 1;
            gotChannel = true;
        } else if (!strcmp(token,TokenCoefficients)) {
            int coeffNdx = 0;
            do {
                Parser::ReadToken(file,token);
                tokenRecognized = sscanf(token,"%lf",&aLUT[coeffNdx++]) == 1;
            } while (coeffNdx < 4 && tokenRecognized);
            gotCoefficients = coeffNdx == 4 && tokenRecognized;
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);

    // Only able to use analog sensor if all the parameters are valid.
    // NOTE FOR FUTURE ENHANCEMENT: Logic should test coefficients with the 
    // range of analog inputs to verify that they give values within the 
    // joint's range of motion. (BTW)
    hasAnalogSensor = gotChannel && gotCoefficients;

}

//----------------------------------------------------------------------------
const char TokenHomingMode[] = "homingMode";
const char TokenNoHoming[] = "noHoming";
const char TokenReverseLimit[] = "reverseLimit";
const char TokenForwardLimit[] = "forwardLimit";
const char TokenBothLimits[] = "bothLimits";
const char TokenHomeAndIndex[] = "homeAndIndex";
const char TokenBestWay[] = "bestWay";
const char TokenRangeOfMotion[] = "rangeOfMotion";
const char TokenHasHardLimits[] = "hasHardLimits";
const char TokenHasHomeSensor[] = "hasHomeSensor";
const char TokenHomeSensorWidth[] = "homeSensorWidth";
const char TokenHasLimitSensors[] = "hasLimitSensors";
const char TokenLimitsMayBind[] = "limitsMayBind";
const char TokenHomeOffset[] = "homeOffset";
const char TokenHomeSignalPolarity[] = "homeSignalPolarity";
const char TokenLimitSignalPolarity[] = "limitSignalPolarity";
const char TokenSoftLimitPad[] = "softLimitPad";
void PMDAxisControl::ReadHomingParameters(FILE *file, char *token) {
    bool tokenRecognized = true;
    double angle; 
    Parser::ReadToken(file,token);
    do {
        if (!strcmp(token,TokenHomeOffset)) {
            tokenRecognized = ReadAngle(file,token,angle);
            if (tokenRecognized) calibratedHomeOffset = UnitsToCounts(angle);
        } else if (!strcmp(token,TokenHomeSensorWidth)) {
            tokenRecognized = ReadAngle(file,token,angle);
            if (tokenRecognized) {
                homeSensorWidth = UnitsToCounts(angle);
                hasHomeSensor = true;
            }
        } else if (!strcmp(token,TokenLimitsMayBind)) {
            tokenRecognized = Parser::ReadBool(file,token, limitsMayBind);
        } else if (!strcmp(token,TokenHomingMode)) {
            Parser::ReadToken(file,token);
            if (!strcmp(token,TokenNoHoming))           homingMode = NoHoming;
            else if (!strcmp(token,TokenReverseLimit))  homingMode = ReverseLimit;
            else if (!strcmp(token,TokenForwardLimit))  homingMode = ForwardLimit;
            else if (!strcmp(token,TokenBothLimits))    homingMode = BothLimits;
            else if (!strcmp(token,TokenHomeAndIndex))  homingMode = HomeAndIndex;
            else if (!strcmp(token,TokenBestWay))  homingMode = BestWay;
            else tokenRecognized = false;
        } else if (!strcmp(token,TokenHasLimitSensors)) {
            tokenRecognized = Parser::ReadBool(file,token,hasLimitSensors);
        } else if (!strcmp(token,TokenHomeSignalPolarity)) {
            tokenRecognized = ReadPolarity(file,token,homeSignalPolarity);
        } else if (!strcmp(token,TokenLimitSignalPolarity)) {
            tokenRecognized = ReadPolarity(file,token,limitSignalPolarity);
        } else if (!strcmp(token,TokenSoftLimitPad)) {
            if (ReadAngle(file,token,angle)) 
                softLimitPad = UnitsToCounts(angle);
            else tokenRecognized = false;
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);
}

//----------------------------------------------------------------------------
const char TokenPos[] = "pos";
const char TokenVel[] = "vel";
const char TokenAcc[] = "acc";
const char TokenDec[] = "dec";
const char TokenJerk[] = "jerk";
void PMDAxisControl::ReadProfile(FILE *file, char *token, Profile &profile) {
    bool tokenRecognized = true;
    profile.pos = profile.vel = profile.acc = profile.dec = profile.jerk = 0.0;
    Parser::ReadToken(file,token);
    do {
        if (!strcmp(token,TokenPos))
            tokenRecognized = ReadAngle(file,token,profile.pos);
        else if (!strcmp(token,TokenVel))
            tokenRecognized = ReadAngle(file,token,profile.vel);
        else if (!strcmp(token,TokenAcc))
            tokenRecognized = ReadAngle(file,token,profile.acc);
        else if (!strcmp(token,TokenDec))
            tokenRecognized = ReadAngle(file,token,profile.dec);
        else if (!strcmp(token,TokenJerk))
            tokenRecognized = ReadAngle(file,token,profile.jerk);
        else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);
}

//----------------------------------------------------------------------------
const char TokenKp[] = "kp";
const char TokenKd[] = "kd";
const char TokenKi[] = "ki";
const char TokenKvff[] = "kvff";
const char TokenKaff[] = "kaff";
const char TokenKout[] = "kout";
const char TokenIl[] = "il";
const char TokenOl[] = "ol";
const char TokenBias[] = "bias";
const char TokenElim[] = "elim";
void PMDAxisControl::ReadFilter(FILE *file, char *token, ServoFilter &filter) {
    bool tokenRecognized = true;
    unsigned int val;
    Parser::ReadToken(file,token);
    do {
        if (!strcmp(token,TokenKp)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            filter.kP = val;
        } else if (!strcmp(token,TokenKd)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            filter.kD = val;
        } else if (!strcmp(token,TokenKi)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            filter.kI = val;
        } else if (!strcmp(token,TokenKvff)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            filter.kVFF = val;
        } else if (!strcmp(token,TokenKaff)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            filter.kAFF = val;
        } else if (!strcmp(token,TokenKout)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            filter.kOut = val;
        } else if (!strcmp(token,TokenIl)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&filter.intLim) == 1;
        } else if (!strcmp(token,TokenOl)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            filter.motorLim = val;
        } else if (!strcmp(token,TokenBias)) {
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            filter.motorBias = val;
        } else if (!strcmp(token,TokenElim)) {
            double elim;
            tokenRecognized = ReadAngle(file,token,elim);
            if (tokenRecognized)
                filter.errorLim = PMDAxisControl::UnitsToCounts(elim);
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);
}

//----------------------------------------------------------------------------
const char TokenLowerHardLimit[] = "lowerHardLimit";
const char TokenUpperHardLimit[] = "upperHardLimit";
const char TokenRadius[] = "radius";
const char TokenLength[] = "length";
const char TokenAlpha[] = "alpha";
const char TokenproximalD[] = "proximalD";
const char TokenproximalA[] = "proximalA";
const char TokendistalD[] = "distalD";
const char TokendistalA[] = "distalA";

void PMDAxisControl::ReadPhysical(FILE *file, char *token) {

    bool tokenRecognized = true;
    double angle;

    // Prime token stream.
    Parser::ReadToken(file,token);

    // Parse the stream.
    do {
        if (!strcmp(token,TokenRadius)) {
            tokenRecognized = PMDUtils::ReadLength(file,token,radiusMeters);
        } else if (!strcmp(token,TokenLength)) {
            tokenRecognized = PMDUtils::ReadLength(file,token,lengthMeters);
        } else if (!strcmp(token,TokenAlpha)) {
            tokenRecognized = PMDUtils::ReadAngle(file,token,dhAlphaRadians);
            dhAlphaRadians = PMDUtils::RevsToRads(dhAlphaRadians);
        } else if (!strcmp(token,TokenLowerHardLimit)) {
            if (ReadAngle(file,token,angle)) { 
                revLimitCounts = UnitsToCounts(angle);
                if (hasHardLimits && fwdLimitCounts < (INT_MAX-1)) 
                    rangeOfMotion = fwdLimitCounts-revLimitCounts;
                hasHardLimits = true;
            } else tokenRecognized = false;
        } else if (!strcmp(token,TokenUpperHardLimit)) {
            if (ReadAngle(file,token,angle)) {
                fwdLimitCounts = UnitsToCounts(angle);
                if (hasHardLimits && revLimitCounts > (INT_MIN+1)) 
                    rangeOfMotion = fwdLimitCounts-revLimitCounts;
                hasHardLimits = true;
            } else tokenRecognized = false;
        } else if (!strcmp(token,TokenproximalD)) {
            tokenRecognized = PMDUtils::ReadLength(file,token,proximalDMeters);
        } else if (!strcmp(token,TokenproximalA)) {
            tokenRecognized = PMDUtils::ReadLength(file,token,proximalAMeters);
        } else if (!strcmp(token,TokendistalD)) {
            tokenRecognized = PMDUtils::ReadLength(file,token,distalDMeters);
        } else if (!strcmp(token,TokendistalA)) {
            tokenRecognized = PMDUtils::ReadLength(file,token,distalAMeters);
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);

}

//----------------------------------------------------------------------------
void PMDAxisControl::WriteConfigToFile(ostream& fp, char* indent)
{
    const char TokenTrue[] = "true";
    const char TokenFalse[] = "false";
    char lclInd[50];
    strcpy(lclInd,indent);
    strcat(lclInd,"\t");

    fp << indent << PMDUtils::TokenUniqueID << '\t' << uniqueID << endl;
    fp << indent << TokenDebugLevel << '\t' << debugLevel << endl;

    // Motor parameters
    // MotorType,PoleCount,AmpType,AmpPolarity,HasBrake,PhaseInitMode
    // CommutationMode,InvertHalls,InvertMotor
    fp << endl << indent << TokenMotor << endl;
    fp << lclInd << TokenMotorType << "       ";
    fp << (isBrushless ? TokenBrushless : TokenBrushed)<< endl;
//    if (isBrushless)    fp << TokenBrushless << endl;
//    else                fp << TokenBrushed << endl;
    if (isBrushless) 
    {   fp << lclInd << TokenPoleCount << "       " << poleCount << endl;
        fp << lclInd << TokenPhaseInit << "       ";
        if (phaseInitMode == PMDPhaseInitHallBased) 
                fp << TokenHallBased << endl;
        else    fp << TokenAlgorithmic << endl;
        fp << lclInd << TokenCommutationMode << " ";
        switch (commutationMode)
        {   case PMDCommutationModeHallBased  : fp << TokenHallBased << endl; break;
            case PMDCommutationModeSinusoidal : fp << TokenSinusoidal << endl; break;
            default                           : fp << TokenMicrostepping << endl; break;
        }
        fp << lclInd << TokenInvertHalls << "     "
           << (invertHalls ? TokenTrue : TokenFalse) << endl;
    }
    fp << lclInd << TokenAmpType << "         ";
    if (ampType == PMDMotorOutputPWMSignMagnitude)
        fp << TokenPWMSignMag << endl;
    else
        fp << TokenPWM5050 << endl;
    fp << lclInd << TokenAmpPolarity << "     ";
    if (ampPolarity == ActiveHigh) fp << TokenActiveHigh << endl;
    else                           fp << TokenActiveLow << endl;
    fp << lclInd << TokenHasBrake << "        "
       << (hasBrake ? TokenTrue : TokenFalse) << endl;
    fp << lclInd << TokenInvertMotorOut << "  "
       << (invertMotorOut ? TokenTrue : TokenFalse) << endl;

    // Encoder parameters
    // HasIndex,Counts/EncCycle,EncCycles/AxisCycle,MotorCycles/EncCycle
    fp << endl << indent << TokenEncoder << endl;
    fp << lclInd << TokenHasEncoderIndex << "          "
       << (hasEncoderIndex ? TokenTrue : TokenFalse) << endl;
    fp << lclInd << TokenCountsPerEncoderCycle << "      " << (int)countsPerEncoderCycle << endl;
    fp << lclInd << TokenEncoderCyclesPerAxisCycle << "  " << encoderCyclesPerAxisCycle  << endl;
    fp << lclInd << TokenMotorCyclesPerEncoderCycle << " " << motorCyclesPerEncoderCycle << endl;

    // Physical parameters
    // HLL,HUL,radius, len, alpha, proxD, proxA, distD, distA
    fp << endl << indent << TokenPhysical << endl;
    fp << lclInd << TokenLowerHardLimit << " " << revLimitCounts   << '\t' << TokenCounts << endl;
    fp << lclInd << TokenUpperHardLimit << " " << fwdLimitCounts   << '\t' << TokenCounts << endl;
    fp << lclInd << TokenRadius << "         " << radiusMeters     << '\t' << PMDUtils::TokenMeters << endl;
    fp << lclInd << TokenLength << "         " << lengthMeters     << '\t' << PMDUtils::TokenMeters << endl;
    fp << lclInd << TokenAlpha << "          " << dhAlphaRadians   << '\t' << PMDUtils::TokenRadians << endl;
    fp << lclInd << TokenproximalD << "      " << proximalDMeters  << '\t' << PMDUtils::TokenMeters << endl;
    fp << lclInd << TokenproximalA << "      " << proximalAMeters  << '\t' << PMDUtils::TokenMeters << endl;
    fp << lclInd << TokendistalD << "        " << distalDMeters    << '\t' << PMDUtils::TokenMeters << endl;
    fp << lclInd << TokendistalA << "        " << distalAMeters    << '\t' << PMDUtils::TokenMeters << endl;

    // Analog parameters
    // AbsPosAnalogChannel,0thTerm,1stTerm,2ndTerm,3rdTerm
    if (hasAnalogSensor)
    {
        fp << endl << indent << TokenAnalog << endl;
        fp << lclInd << TokenAbsPosAnalogChannel << " " << absPosAnalogChannel << endl;
        for (int i = 0; i<4;i++) 
        {   fp << lclInd << aLUT[i] << '\t' << "#coefficient order " << i << endl;  
        }
    }

    // Homing parameters
    // HomingMode,HomeOffset,HomeSensorWidth,LimitsMayBind,HasLimitSensors
    // HomeSignalPolarity,LimitSignalPolarity,SoftLimitPad
    fp << endl << indent << TokenHomingParams << endl;
    fp << lclInd << TokenHomingMode << "          ";
    switch (homingMode)
    {   case NoHoming     : fp << TokenNoHoming << endl; break;
        case ReverseLimit : fp << TokenReverseLimit << endl; break;
        case ForwardLimit : fp << TokenForwardLimit << endl; break;
        case BothLimits   : fp << TokenBothLimits << endl; break;
        case HomeAndIndex : fp << TokenHomeAndIndex << endl; break;
        default           : fp << TokenBestWay << endl; break;
    }
    fp << lclInd << TokenHomeOffset << "          " << calibratedHomeOffset << '\t' << TokenCounts << endl;
    fp << lclInd << TokenHomeSensorWidth << "     " << (int)homeSensorWidth << '\t' << TokenCounts << endl;
    fp << lclInd << TokenLimitsMayBind << "       "
       << (limitsMayBind ? TokenTrue : TokenFalse) << endl;
    fp << lclInd << TokenHasLimitSensors << "     "
       << (hasLimitSensors ? TokenTrue : TokenFalse) << endl;
    fp << lclInd << TokenHomeSignalPolarity << "  ";
    if (homeSignalPolarity == ActiveHigh)   fp << TokenActiveHigh << endl;
    else                                    fp << TokenActiveLow << endl;
    fp << lclInd << TokenLimitSignalPolarity << " ";
    if (limitSignalPolarity == ActiveHigh)  fp << TokenActiveHigh << endl;
    else                                    fp << TokenActiveLow << endl;
    fp << lclInd << TokenSoftLimitPad << "        " << softLimitPad << '\t' << TokenCounts << endl;

    // Homing profile
    // Pos,Vel,Acc,Dec,Jerk
    fp << endl << indent << TokenHomingProfile << endl;
    WriteProfileToFile(fp,lclInd,homingProfileCounts);

    // Homing profile
    // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    fp << endl << indent << TokenHomingFilter << endl;
    WriteServoFilterToFile(fp,lclInd,homingFilter);

    // Run profile
    // Pos,Vel,Acc,Dec,Jerk
    CountsProfile runProfileCounts; // This class doesn't have this variable, why?
    UnitsToCounts(runProfile,runProfileCounts);
    fp << endl << indent << TokenProfile << endl;
    WriteProfileToFile(fp,lclInd,runProfileCounts);

    // Run profile
    // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    fp << endl << indent << TokenFilter << endl;
    WriteServoFilterToFile(fp,lclInd,runFilter);

}

//----------------------------------------------------------------------------
// Clear the isParked and configDataValid flags for all axes
void PMDAxisControl::InvalidateMemoryContents()
{
    char methodStr[] = "InvalidateMemoryContents: ";
    if (controller->HasMemory())
    {   PMDuint16 buffID;
        PMDuint32 buffAddr;
        GetAxisConfigBuffInfo(buffID,buffAddr);
        SetBufferStart(buffID,buffAddr);
        SetBufferLength(BuffIDMisc,3); // only 3 mem locs needed
        WriteBuffer(buffID,0);  // isParked flag
        WriteBuffer(buffID,0);  // Parked position
        WriteBuffer(buffID,0);  // Configuration data is valid flag
        ifDbgCout << "Memory invalidated\n";
    }
}

//----------------------------------------------------------------------------
bool PMDAxisControl::ReadAxisStateFromFile() {
    char fName[100];
    stringstream stream(stringstream::in | stringstream::out);
    stream << controller->uniqueID << '-' << uniqueID << ".home" << ends;
    stream >> fName;

    ifstream fp(fName,ios::in);
    if (fp.is_open()) {
        int pos;
        int lim[4];

        // Get the values from the file.
        fp >> pos >> lim[0] >> lim[1] >> lim[2] >> lim[3];

        SetHomingLimits(lim[0],lim[1],lim[2],lim[3]);
        EnableAmp();
        fp.close();
        return true;
    } else {
        return false;
    }
}
//----------------------------------------------------------------------------
bool PMDAxisControl::WriteAxisStateToFile() {
    char fName[100];
    stringstream stream(stringstream::in | stringstream::out);
    stream << controller->uniqueID << '-' << uniqueID << ".home" << ends;
    stream >> fName;
    ofstream fp(fName,ios::out);

    if (!fp) {
        return false;
    } else {
        PMDlong32 pos,lim[4];
        GetActualPosition(pos);
        GetHomingLimits(lim[0],lim[1],lim[2],lim[3]);
        fp << pos << ' ' << lim[0] << ' ' << lim[1] << ' ' << lim[2] << ' ' << lim[3] << endl;
        fp.close();
        return true;
    }
}

//----------------------------------------------------------------------------
void PMDAxisControl::ReadCountsProfileFromMemory(int buff, CountsProfile &prof)
{
    PMDint32 data;
    ReadBuffer(buff,data); prof.pos = data;
    ReadBuffer(buff,data); prof.vel = data;
    ReadBuffer(buff,data); prof.acc = data;
    ReadBuffer(buff,data); prof.dec = data;
    ReadBuffer(buff,data); prof.jerk = data;
}

//----------------------------------------------------------------------------
void PMDAxisControl::ReadServoFilterFromMemory(int buff, ServoFilter &filter)
{
    PMDint32 data;
    ReadBuffer(buff,data); filter.kP = data;
    ReadBuffer(buff,data); filter.kD = data;
    ReadBuffer(buff,data); filter.kI = data;
    ReadBuffer(buff,data); filter.kVFF = data;
    ReadBuffer(buff,data); filter.kAFF = data;
    ReadBuffer(buff,data); filter.kOut = data;
    ReadBuffer(buff,data); filter.intLim = data;
    ReadBuffer(buff,data); filter.motorLim = data;
    ReadBuffer(buff,data); filter.motorBias = data;
    ReadBuffer(buff,data); filter.errorLim = data;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::CheckRead(int buff, int sum) {
    PMDint32 data;
    ReadBuffer(buff,data);
    return (data != sum);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::ReadConfigDataFromMemory(int buff)
{
    char methodStr[] = "ReadConfigDataFromMemory: ";
    PMDMemoryFloatOverlayType dataf;
    PMDint32 data = 0;
    PMDint32 sum = 0;

    // Motor parameters
    // MotorType,PoleCount,AmpType,AmpPolarity,HasBrake,PhaseInitMode
    // CommutationMode,InvertHalls,InvertMotor
    ReadBuffer(buff,data); isBrushless = (data == PMDTRUEMEMVALUE); sum++;
    ReadBuffer(buff,data); poleCount = data; sum++;
    ReadBuffer(buff,data); ampType = (tagPMDMotorOutputMode)data; sum++;
    ReadBuffer(buff,data); ampPolarity = (Polarity)data; sum++;
    ReadBuffer(buff,data); hasBrake = (data == PMDTRUEMEMVALUE); sum++;
    ReadBuffer(buff,data); phaseInitMode = (tagPMDPhaseInitializeMode)data; sum++;
    ReadBuffer(buff,data); commutationMode = (tagPMDCommutationMode)data; sum++;
    ReadBuffer(buff,data); invertHalls = (data == PMDTRUEMEMVALUE); sum++;
    ReadBuffer(buff,data); invertMotorOut = (data == PMDTRUEMEMVALUE); sum++;
    if (CheckRead(buff,sum)) {ifDbgCout << "Motor read fail\n"; return false; }

    // Encoder parameters
    // HasIndex,Counts/EncCycle,EncCycles/AxisCycle,MotorCycles/EncCycle
    ReadBuffer(buff,data); hasEncoderIndex = (data == PMDTRUEMEMVALUE); sum++;
    ReadBuffer(buff,dataf.i); countsPerEncoderCycle = dataf.f; sum++;
    ReadBuffer(buff,dataf.i); encoderCyclesPerAxisCycle = dataf.f; sum++;
    ReadBuffer(buff,dataf.i); motorCyclesPerEncoderCycle = dataf.f; sum++;
    countsPerAxisCycle = countsPerEncoderCycle*encoderCyclesPerAxisCycle;
    countsPerMotorCycle = countsPerEncoderCycle/motorCyclesPerEncoderCycle;
    if (CheckRead(buff,sum)) {ifDbgCout << "Encoder read fail\n"; return false; }

    // Physical parameters
    // HLL,HUL,softLimPad, radius, len, alpha, proxD, proxA, distD, distA
    ReadBuffer(buff,data); revLimitCounts = data; sum++;
    ReadBuffer(buff,data); fwdLimitCounts = data; sum++;
    hasHardLimits = (revLimitCounts != 0 || fwdLimitCounts != 0);
    rangeOfMotion = fwdLimitCounts-revLimitCounts;
    ReadBuffer(buff,dataf.i); radiusMeters = dataf.f; sum++;
    ReadBuffer(buff,dataf.i); lengthMeters = dataf.f; sum++;
    ReadBuffer(buff,dataf.i); dhAlphaRadians = PMDUtils::RevsToRads((double)dataf.f); sum++;
    ReadBuffer(buff,dataf.i); proximalDMeters = dataf.f; sum++;
    ReadBuffer(buff,dataf.i); proximalAMeters = dataf.f; sum++;
    ReadBuffer(buff,dataf.i); distalDMeters = dataf.f; sum++;
    ReadBuffer(buff,dataf.i); distalAMeters = dataf.f; sum++;
    if (CheckRead(buff,sum)) {ifDbgCout << "Physical read fail\n"; return false; }

    // Analog parameters
    // AbsPosAnalogChannel,0thTerm,1stTerm,2ndTerm,3rdTerm
    ReadBuffer(buff,data); hasAnalogSensor = (data == PMDTRUEMEMVALUE); sum++;
    ReadBuffer(buff,data); absPosAnalogChannel = data; sum++;
    for (int i = 0; i<4;i++) 
    {   ReadBuffer(buff,dataf.i); aLUT[i] = dataf.f; sum++;
    }
    if (CheckRead(buff,sum)) {ifDbgCout << "Analog read fail\n"; return false; }

    // Homing parameters
    // HomingMode,HomeOffset,HomeSensorWidth,LimitsMayBind,HasLimitSensors
    // HomeSignalPolarity,LimitSignalPolarity,SoftLimitPad
    ReadBuffer(buff,data); homingMode = (HomingMode) data; sum++;
    ReadBuffer(buff,data); calibratedHomeOffset = data; sum++;
    ReadBuffer(buff,data); homeSensorWidth = data; sum++;
    hasHomeSensor = (homeSensorWidth != 0);
    ReadBuffer(buff,data); limitsMayBind = (data == PMDTRUEMEMVALUE); sum++;
    ReadBuffer(buff,data); hasLimitSensors = (data == PMDTRUEMEMVALUE); sum++;
    ReadBuffer(buff,data); homeSignalPolarity = (Polarity)data; sum++;
    ReadBuffer(buff,data); limitSignalPolarity = (Polarity)data; sum++;
    ReadBuffer(buff,data); softLimitPad = data; sum++;
    fwdSoftLimitCounts = fwdLimitCounts - softLimitPad;
    revSoftLimitCounts = revLimitCounts + softLimitPad;
    if (CheckRead(buff,sum)) {ifDbgCout << "Homing read fail\n"; return false; }

    // Homing profile
    // Pos,Vel,Acc,Dec,Jerk
    ReadCountsProfileFromMemory(buff,homingProfileCounts); sum++;
    CountsToUnits(homingProfile,homingProfileCounts);
    homeCfgCounts=homingProfileCounts;
    if (CheckRead(buff,sum)) {ifDbgCout << "Home profile read fail\n"; return false; }

    // Homing profile
    // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    ReadServoFilterFromMemory(buff,homingFilter); sum++;
    if (CheckRead(buff,sum)) {ifDbgCout << "Home filter read fail\n"; return false; }

    // Run profile
    // Pos,Vel,Acc,Dec,Jerk
    CountsProfile runProfileCounts; // This class doesn't have this variable, why?
    ReadCountsProfileFromMemory(buff,runProfileCounts); sum++;
    CountsToUnits(runProfile,runProfileCounts);
    if (CheckRead(buff,sum)) {ifDbgCout << "Run profile read fail\n"; return false; }

    // Run profile
    // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    ReadServoFilterFromMemory(buff,runFilter); sum++;
    if (CheckRead(buff,sum)) {ifDbgCout << "Run filter read fail\n"; return false; }

    // unique ID
    PMDMemoryCharsOverlayType datac;
    int uidNdx = 0;
    bool endOfString = false;
    do 
    {
        ReadBuffer(buff,datac.i);  sum++;
        for (int i = 0; i < 4 && !endOfString; i++)
        {   uniqueID[uidNdx++] = datac.str[i];
            endOfString = (datac.str[i] == 0);
        }
    } while (!endOfString);
    if (CheckRead(buff,sum)) {ifDbgCout << "UID read fail\n"; return false; }

    return true;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::BurnAxisConfig()
{
    int buff = BuffIDMisc;
    const int buffStart[] = 
        {BuffAddrAxis0,BuffAddrAxis1,BuffAddrAxis2,BuffAddrAxis3};
    SetBufferStart(buff,buffStart[GetAxisNumber()] +
                               AxisBuffOffsetMotorParams);
    SetBufferLength(buff,AXIS_CFG_BUFF_SIZE);
    if (WriteConfigDataToMemory(buff))
    {   cout << "Write successful? Checking...\n";
      if (ValidateConfigDataInMemory(buff))
      {   
	      cout << "Write was successful. Setting validity flag in mem\n";
	      SetBufferStart(buff,
			     buffStart[GetAxisNumber()] +
			     AxisBuffOffsetCfgDataValid);
	      WriteBuffer(buff,PMDTRUEMEMVALUE);
        }
      else {
    cout << "Write was not successful\n";
    return false;
      }
    }
    else return false;
    return true;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::WriteConfigDataToMemory(int buff)
{
    PMDMemoryFloatOverlayType dataf;
    PMDint32 data = 0;
    PMDint32 sum = 0;

    // Motor parameters
    // MotorType,PoleCount,AmpType,AmpPolarity,HasBrake,PhaseInitMode
    // CommutationMode,InvertHalls,InvertMotor
    WriteBuffer(buff,isBrushless ? PMDTRUEMEMVALUE : 0); sum++;
    data = poleCount; WriteBuffer(buff,data); sum++;
    data = ampType; WriteBuffer(buff,data); sum++;
    data = ampPolarity; WriteBuffer(buff,data); sum++;
    WriteBuffer(buff,hasBrake ? PMDTRUEMEMVALUE : 0); sum++;
    data = phaseInitMode; WriteBuffer(buff,data); sum++;
    data = commutationMode; WriteBuffer(buff,data); sum++;
    WriteBuffer(buff,invertHalls ? PMDTRUEMEMVALUE : 0); sum++;
    WriteBuffer(buff,invertMotorOut ? PMDTRUEMEMVALUE : 0); sum++;
    WriteBuffer(buff,sum);  // Write checksum

    // Encoder parameters
    // HasIndex,Counts/EncCycle,EncCycles/AxisCycle,MotorCycles/EncCycle
    WriteBuffer(buff,hasEncoderIndex ? PMDTRUEMEMVALUE : 0); sum++;
    dataf.f = countsPerEncoderCycle; WriteBuffer(buff,dataf.i); sum++;
    dataf.f = encoderCyclesPerAxisCycle; WriteBuffer(buff,dataf.i); sum++;
    dataf.f = motorCyclesPerEncoderCycle; WriteBuffer(buff,dataf.i); sum++;
    WriteBuffer(buff,sum);  // Write checksum

    // Physical parameters
    // HLL,HUL, radius, len, alpha, proxD, proxA, distD, distA
    data = revLimitCounts; WriteBuffer(buff,data); sum++;
    data = fwdLimitCounts; WriteBuffer(buff,data); sum++;
    dataf.f = radiusMeters; WriteBuffer(buff,dataf.i); sum++;
    dataf.f = lengthMeters; WriteBuffer(buff,dataf.i); sum++;
    dataf.f = PMDUtils::RadsToRevs(dhAlphaRadians); WriteBuffer(buff,dataf.i); sum++;
    dataf.f = proximalDMeters; WriteBuffer(buff,dataf.i); sum++;
    dataf.f = proximalAMeters; WriteBuffer(buff,dataf.i); sum++;
    dataf.f = distalDMeters; WriteBuffer(buff,dataf.i); sum++;
    dataf.f = distalAMeters; WriteBuffer(buff,dataf.i); sum++;
    WriteBuffer(buff,sum);  // Write checksum

    // Analog parameters
    // AbsPosAnalogChannel,0thTerm,1stTerm,2ndTerm,3rdTerm
    WriteBuffer(buff,hasAnalogSensor ? PMDTRUEMEMVALUE : 0); sum++;
    data = absPosAnalogChannel; WriteBuffer(buff,data); sum++;
    for (int i = 0; i<4;i++) 
    {   dataf.f = aLUT[i]; WriteBuffer(buff,dataf.i); sum++;
    }
    WriteBuffer(buff,sum);  // Write checksum

    // Homing parameters
    // HomingMode,HomeOffset,SoftLimitPad,HomeSensorWidth,LimitsMayBind,HasLimitSensors
    // HomeSignalPolarity,LimitSignalPolarity,SoftLimitPad
    data = homingMode; WriteBuffer(buff,data); sum++;
    data = calibratedHomeOffset; WriteBuffer(buff,data); sum++;
    data = homeSensorWidth; WriteBuffer(buff,data); sum++;
    WriteBuffer(buff,limitsMayBind ? PMDTRUEMEMVALUE : 0); sum++;
    WriteBuffer(buff,hasLimitSensors ? PMDTRUEMEMVALUE : 0); sum++;
    data = homeSignalPolarity; WriteBuffer(buff,data); sum++;
    data = limitSignalPolarity; WriteBuffer(buff,data); sum++;
    data = softLimitPad; WriteBuffer(buff,data); sum++;
    WriteBuffer(buff,sum);  // Write checksum

    // Homing profile
    // Pos,Vel,Acc,Dec,Jerk
    WriteCountsProfileToMemory(buff,homingProfileCounts); sum++;
    WriteBuffer(buff,sum);  // Write checksum

    // Homing profile
    // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    WriteServoFilterToMemory(buff,homingFilter); sum++;
    WriteBuffer(buff,sum);  // Write checksum

    // Run profile
    // Pos,Vel,Acc,Dec,Jerk
    CountsProfile runProfileCounts; // This class doesn't have this variable, why?
    UnitsToCounts(runProfile,runProfileCounts);
    WriteCountsProfileToMemory(buff,runProfileCounts); sum++;
    WriteBuffer(buff,sum);  // Write checksum

    // Run profile
    // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    WriteServoFilterToMemory(buff,runFilter); sum++;
    WriteBuffer(buff,sum);  // Write checksum

    // unique ID
    PMDMemoryCharsOverlayType datac;
    int uidNdx = 0;
    bool endOfString = false;
    do 
    {
        for (int i = 0; i < 4 && !endOfString; i++)
        {   datac.str[i] = uniqueID[uidNdx++];
            endOfString = (datac.str[i] == 0);
        }
        WriteBuffer(buff,datac.i);  sum++;
    } while (!endOfString);
    WriteBuffer(buff,sum);  // Write checksum

    return true;
}

//----------------------------------------------------------------------------
void PMDAxisControl::WriteCountsProfileToMemory(int buff, CountsProfile prof)
{
    PMDint32 data;
    data = prof.pos; WriteBuffer(buff,data);
    data = prof.vel; WriteBuffer(buff,data);
    data = prof.acc; WriteBuffer(buff,data);
    data = prof.dec; WriteBuffer(buff,data);
    data = prof.jerk; WriteBuffer(buff,data);
}

//----------------------------------------------------------------------------
void PMDAxisControl::WriteServoFilterToMemory(int buff, ServoFilter filter)
{
    PMDint32 data;
    data = filter.kP; WriteBuffer(buff,data);
    data = filter.kD; WriteBuffer(buff,data);
    data = filter.kI; WriteBuffer(buff,data);
    data = filter.kVFF; WriteBuffer(buff,data);
    data = filter.kAFF; WriteBuffer(buff,data);
    data = filter.kOut; WriteBuffer(buff,data);
    data = filter.intLim; WriteBuffer(buff,data);
    data = filter.motorLim; WriteBuffer(buff,data);
    data = filter.motorBias; WriteBuffer(buff,data);
    data = filter.errorLim; WriteBuffer(buff,data);
}

//----------------------------------------------------------------------------
bool PMDAxisControl::ValidateConfigDataInMemory(int buff)
{
    PMDMemoryFloatOverlayType dataf;
    PMDint32 data = 0;
    bool isValid = true;
    const double eps = 0.0001;
    PMDint32 sum = 0;

    // Motor parameters
    // MotorType,PoleCount,AmpType,AmpPolarity,HasBrake,PhaseInitMode
    // CommutationMode,InvertHalls,InvertMotor
    ReadBuffer(buff,data); isValid &= (isBrushless == (data == PMDTRUEMEMVALUE)); sum++;
    ReadBuffer(buff,data); isValid &= (poleCount == data); sum++;
    ReadBuffer(buff,data); isValid &= (ampType == (tagPMDMotorOutputMode)data); sum++;
    ReadBuffer(buff,data); isValid &= (ampPolarity == (Polarity)data); sum++;
    ReadBuffer(buff,data); isValid &= (hasBrake == (data == PMDTRUEMEMVALUE)); sum++;
    ReadBuffer(buff,data); isValid &= (phaseInitMode == (tagPMDPhaseInitializeMode)data); sum++;
    ReadBuffer(buff,data); isValid &= (commutationMode == (tagPMDCommutationMode)data); sum++;
    ReadBuffer(buff,data); isValid &= (invertHalls == (data == PMDTRUEMEMVALUE)); sum++;
    ReadBuffer(buff,data); isValid &= (invertMotorOut == (data == PMDTRUEMEMVALUE)); sum++;
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // Encoder parameters
    // HasIndex,Counts/EncCycle,EncCycles/AxisCycle,MotorCycles/EncCycle
    ReadBuffer(buff,data); isValid &= (hasEncoderIndex == (data == PMDTRUEMEMVALUE)); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(countsPerEncoderCycle,dataf.f,eps); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(encoderCyclesPerAxisCycle,dataf.f,eps); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(motorCyclesPerEncoderCycle,dataf.f,eps); sum++;
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // Physical parameters
    // HLL,HUL,softLimPad, radius, len, alpha, proxD, proxA, distD, distA
    ReadBuffer(buff,data); isValid &= (revLimitCounts == data); sum++;
    ReadBuffer(buff,data); isValid &= (fwdLimitCounts == data); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(radiusMeters,dataf.f,eps); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(lengthMeters,dataf.f,eps); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(dhAlphaRadians,PMDUtils::RevsToRads((double)dataf.f),eps); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(proximalDMeters,dataf.f,eps); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(proximalAMeters,dataf.f,eps); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(distalDMeters,dataf.f,eps); sum++;
    ReadBuffer(buff,dataf.i); isValid &= PMDUtils::AlmostEqual(distalAMeters,dataf.f,eps); sum++;
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // Analog parameters
    // AbsPosAnalogChannel,0thTerm,1stTerm,2ndTerm,3rdTerm
    ReadBuffer(buff,data); isValid &= (hasAnalogSensor == (data == PMDTRUEMEMVALUE)); sum++;
    ReadBuffer(buff,data); absPosAnalogChannel = data; sum++;
    for (int i = 0; i<4;i++) 
    {   ReadBuffer(buff,dataf.i);  sum++;
        isValid &= PMDUtils::AlmostEqual(aLUT[i],dataf.f,eps);
    }
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // Homing parameters
    // HomingMode,HomeOffset,HomeSensorWidth,LimitsMayBind,HasLimitSensors
    // HomeSignalPolarity,LimitSignalPolarity,SoftLimitPad
    ReadBuffer(buff,data); isValid &= (homingMode == (HomingMode) data); sum++;
    ReadBuffer(buff,data); isValid &= (calibratedHomeOffset == data); sum++;
    ReadBuffer(buff,data); isValid &= (homeSensorWidth == (PMDuint32)data); sum++;
    ReadBuffer(buff,data); isValid &= (limitsMayBind == (data == PMDTRUEMEMVALUE)); sum++;
    ReadBuffer(buff,data); isValid &= (hasLimitSensors == (data == PMDTRUEMEMVALUE)); sum++;
    ReadBuffer(buff,data); isValid &= (homeSignalPolarity == (Polarity)data); sum++;
    ReadBuffer(buff,data); isValid &= (limitSignalPolarity == (Polarity)data); sum++;
    ReadBuffer(buff,data); isValid &= (softLimitPad == data); sum++;
//    fwdSoftLimitCounts = fwdLimitCounts - softLimitPad;
//    revSoftLimitCounts = revLimitCounts + softLimitPad;
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // Homing profile
    // Pos,Vel,Acc,Dec,Jerk
    isValid &= ValidateCountsProfileInMemory(buff,homingProfileCounts); sum++;
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // Homing profile
    // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    isValid &= ValidateServoFilterInMemory(buff,homingFilter); sum++;
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // Run profile
    // Pos,Vel,Acc,Dec,Jerk
    CountsProfile runProfileCounts; // This class doesn't have this variable, why?
    UnitsToCounts(runProfile,runProfileCounts);
    isValid &= ValidateCountsProfileInMemory(buff,runProfileCounts); sum++;
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // Run profile
    // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    isValid &= ValidateServoFilterInMemory(buff,runFilter); sum++;
    ReadBuffer(buff,data); isValid &= (data == sum); if (!isValid) return false;

    // unique ID
    PMDMemoryCharsOverlayType datac;
    int uidNdx = 0;
    bool endOfString = false;
    do 
    {
        ReadBuffer(buff,datac.i);  sum++;
        for (int i = 0; i < 4 && !endOfString; i++)
        {   isValid &= (uniqueID[uidNdx] == datac.str[i]);
            endOfString = (uniqueID[uidNdx] == 0);
            uidNdx++;
        }
    } while (!endOfString && isValid);
    ReadBuffer(buff,data); isValid &= (data == sum);

    return isValid;
}
//----------------------------------------------------------------------------
bool PMDAxisControl::ValidateCountsProfileInMemory(int buff, CountsProfile prof)
{
    PMDint32 data;
    bool isValid = true;

    ReadBuffer(buff,data); isValid &= (prof.pos == data);
    ReadBuffer(buff,data); isValid &= (prof.vel == data);
    ReadBuffer(buff,data); isValid &= (prof.acc == (PMDuint32)data);
    ReadBuffer(buff,data); isValid &= (prof.dec == (PMDuint32)data);
    ReadBuffer(buff,data); isValid &= (prof.jerk == (PMDuint32)data);

    return isValid;
}

//----------------------------------------------------------------------------
bool PMDAxisControl::ValidateServoFilterInMemory(int buff, ServoFilter filter)
{
    PMDint32 data;
    bool isValid = true;
    ReadBuffer(buff,data); isValid &= (filter.kP == data);
    ReadBuffer(buff,data); isValid &= (filter.kD == data);
    ReadBuffer(buff,data); isValid &= (filter.kI == data);
    ReadBuffer(buff,data); isValid &= (filter.kVFF == data);
    ReadBuffer(buff,data); isValid &= (filter.kAFF == data);
    ReadBuffer(buff,data); isValid &= (filter.kOut == data);
    ReadBuffer(buff,data); isValid &= (filter.intLim == (PMDuint32)data);
    ReadBuffer(buff,data); isValid &= (filter.motorLim == data);
    ReadBuffer(buff,data); isValid &= (filter.motorBias == data);
    ReadBuffer(buff,data); isValid &= (filter.errorLim == (PMDuint32)data);

    return isValid;
}

//----------------------------------------------------------------------------
void PMDAxisControl::WriteProfileToFile(ostream& fp, char* indent,CountsProfile prof)
{
    if (prof.pos != 0)  fp  << indent << TokenPos  << '\t' << prof.pos  
                            << '\t' << TokenCounts << endl;
    if (prof.vel != 0)  fp  << indent << TokenVel << '\t' << prof.vel  
                            << '\t' << TokenCounts << endl;
    if (prof.acc != 0)  fp  << indent << TokenAcc  << '\t' << (int)prof.acc  
                            << '\t' << TokenCounts << endl;
    if (prof.dec != 0)  fp  << indent << TokenDec  << '\t' << (int)prof.dec  
                            << '\t' << TokenCounts << endl;
    if (prof.jerk != 0) fp  << indent << TokenJerk << '\t' << (int)prof.jerk 
                            << '\t' << TokenCounts << endl;
}

//----------------------------------------------------------------------------
void PMDAxisControl::WriteServoFilterToFile(ostream& fp, char* indent, ServoFilter filter)
{
    fp  << indent << TokenKp   << '\t' << filter.kP << endl;
    fp  << indent << TokenKd   << '\t' << filter.kD << endl;
    fp  << indent << TokenKi   << '\t' << filter.kI << endl;
    fp  << indent << TokenKvff << '\t' << filter.kVFF << endl;
    fp  << indent << TokenKaff << '\t' << filter.kAFF << endl;
    fp  << indent << TokenKout << '\t' << filter.kOut << endl;
    fp  << indent << TokenIl   << '\t' << (int)filter.intLim << endl;
    fp  << indent << TokenOl   << '\t' << filter.motorLim << endl;
    fp  << indent << TokenBias << '\t' << filter.motorBias << endl;
    fp  << indent << TokenElim << '\t' << (int)filter.errorLim  
                               << '\t' << TokenCounts << endl;
}
