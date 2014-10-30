#ifndef PMD_CommonTypes
#define PMD_CommonTypes

//  PMDtypes.h -- common type declarations
//
//  Performance Motion Devices, Inc.
//

#include <cmath>       // for pow() 
#include <sys/types.h>

// common types
#define PMDnull 0

#ifdef WIN32
typedef unsigned __int32 u_int32_t;
#ifndef __MINGW32__
typedef __int32 int32_t;
#define OS_ENUM_TYPEDEF typedef
#else
#define OS_ENUM_TYPEDEF
#include <stdint.h>
#endif
#else
#define OS_ENUM_TYPEDEF
#endif


typedef int32_t PMDint32;
typedef u_int32_t  PMDuint32;
typedef int32_t PMDlong32;
typedef unsigned short PMDuint16;
typedef short PMDint16;
typedef unsigned char  PMDuint8;

typedef PMDuint16 PMDAxis;

OS_ENUM_TYPEDEF enum {nibbleMask=0x0F, byteMask=0xFF};

const int PMDMaxAxes = 4;
const double PMDVelScaling = 1<<16;
const double PMDAccScaling = 1<<16;
const double PMDJerkScaling = pow(2.0,32.0);//1<<32;


//OS_ENUM_TYPEDEF enum {
typedef enum {
        PMDAxis1=0,
        PMDAxis2=1,
        PMDAxis3=2,
        PMDAxis4=3
} PMDAxisEnum;

OS_ENUM_TYPEDEF enum {
        PMDNoAxisMask=0x00,
        PMDAxis1Mask=0x01,
        PMDAxis2Mask=0x02,
        PMDAxis3Mask=0x04,
        PMDAxis4Mask=0x08
};

// PMD Controller chip types
//OS_ENUM_TYPEDEF enum {
//    PMDGenerationNavigator = 2,
//    PMDGenerationPilot = 3,
//    PMDGenerationMagellan = 5
//};
typedef enum {
    PMDFamilyNavigator    = 2,
    PMDFamilyPilot        = 3,
    PMDFamilyMagellan     = 5,
    PMDFamilyION          = 9
} tagPMDProductFamily;


// Profile Generation
typedef enum {
        PMDTrapezoidalProfile=0,
        PMDVelocityContouringProfile=1,
        PMDSCurveProfile=2,
        PMDElectronicGearProfile=3,
        PMDExternalProfile=4
} tagPMDProfileMode;

typedef enum {
        PMDPositionSourceActual=0, 
        PMDPositionSourceCommanded=1
} tagPMDPositionSource;

typedef enum {
        PMDNoStopMode=0, 
        PMDAbruptStopMode=1, 
        PMDSmoothStopMode=2
} tagPMDStopMode;

OS_ENUM_TYPEDEF enum {
        PMDOperatingModeAxisEnBit=0,
        PMDOperatingModeMotorOutputEnBit=1,
        PMDOperatingModeCurrentCtlEnBit=2,
        PMDOperatingModePosLoopEnBit=4,
        PMDOperatingModeTrajectoryEnBit=5
};
OS_ENUM_TYPEDEF enum {
        PMDOperatingModeAxisEnMask=0x0001,
        PMDOperatingModeMotorOutputEnMask=0x0002,
        PMDOperatingModeCurrentCtlEnMask=0x0004,
        PMDOperatingModePosLoopEnMask=0x0010,
        PMDOperatingModeTrajectoryEnMask=0x0020
};

typedef enum {
        PMDOperatingMode
} tagPMDOperatingMode;

// Servo Filter
typedef enum {
        PMDLimitDisabled=0, 
        PMDLimitEnabled=1
} tagPMDLimitMode;

typedef enum {
        PMDAutoStopDisabled=0, 
        PMDAutoStopEnabled=1
} tagPMDAutoStopMode;

typedef enum {
        PMDMotionCompleteCommandedPosition=0, 
        PMDMotionCompleteActualPosition=1
} tagPMDMotionCompleteMode;

typedef enum {
    PMDPositionLoopPIDKp=0,
    PMDPositionLoopPIDKi=1,
    PMDPositionLoopPIDLimit=2,
    PMDPositionLoopPIDKd=3,
    PMDPositionLoopPIDKdTime=4,
    PMDPositionLoopPIDKout=5,
    PMDPositionLoopKvff=6,
    PMDPositionLoopKaff=7,
    PMDPositionLoopBiquad1Enable=8,
    PMDPositionLoopBiquad1B0=9,
    PMDPositionLoopBiquad1B1=10,
    PMDPositionLoopBiquad1B2=11,
    PMDPositionLoopBiquad1A1=12,
    PMDPositionLoopBiquad1A2=13,
    PMDPositionLoopBiquad1K=14,
    PMDPositionLoopBiquad2Enable=15,
    PMDPositionLoopBiquad2B0=16,
    PMDPositionLoopBiquad2B1=17,
    PMDPositionLoopBiquad2B2=18,
    PMDPositionLoopBiquad2A1=19,
    PMDPositionLoopBiquad2A2=20,
    PMDPositionLoopBiquad2K=21
} tagPMDPositionLoopParameter;

typedef enum {
    PMDPositionLoopValueIntegratorSum=0,
    PMDPositionLoopValueIntegralContribution=1,
    PMDPositionLoopValueDerivative=2,
    PMDPositionLoopValueBiquad1Input=3,
    PMDPositionLoopValueBiquad2Input=4
} tagPMDPositionLoopValueParameter;

// Parameter Update & Breakpoints
OS_ENUM_TYPEDEF enum {
        PMDBreakpoint1=0,
        PMDBreakpoint2=1
};

typedef enum {
        PMDBreakpointDisable=0,
        PMDBreakpointGreaterOrEqualCommandedPosition=1,
        PMDBreakpointLessOrEqualCommandedPosition=2,
        PMDBreakpointGreaterOrEqualActualPosition=3,
        PMDBreakpointLessOrEqualActualPosition=4,
        PMDBreakpointCommandedPositionCrossed=5,
        PMDBreakpointActualPositionCrossed=6,
        PMDBreakpointTime=7,
        PMDBreakpointEventStatus=8,
        PMDBreakpointActivityStatus=9,
        PMDBreakpointSignalStatus=10
} tagPMDBreakpointTrigger;

typedef enum {
        PMDBreakpointNoAction=0,
        PMDBreakpointActionUpdate=1,
        PMDBreakpointActionAbruptStop=2,
        PMDBreakpointActionSmoothStop=3,
        PMDBreakpointActionMotorOff=4
} tagPMDBreakpointAction;

// Status Register Control
OS_ENUM_TYPEDEF enum {
        PMDActivityPhasingInitializedMask=0x0001,
        PMDActivityAtMaximumVelocityMask=0x0002,
        PMDActivityTrackingMask=0x0004,
        PMDActivityProfileModeMask=0x0038,
        PMDActivityAxisSettledMask=0x0080,
        PMDActivityMotorOnMask=0x0100,
        PMDActivityPositionCaptureMask=0x0200,
        PMDActivityInMotionMask=0x0400,
        PMDActivityInPositiveLimitMask=0x0800,
        PMDActivityInNegativeLimitMask=0x1000,
        PMDActivityProfileSegment=0xE000,
        PMDActivityStatusMask=0xFF7F //previously 0x1FBF, which I think is just wrong...
};

OS_ENUM_TYPEDEF enum {
        PMDEventMotionCompleteMask=0x0001,
        PMDEventWrapAroundMask=0x0002,
        PMDEventBreakpoint1Mask=0x0004,
        PMDEventCaptureReceivedMask=0x0008,
        PMDEventMotionErrorMask=0x0010,
        PMDEventInPositiveLimitMask=0x0020,
        PMDEventInNegativeLimitMask=0x0040,
        PMDEventInstructionErrorMask=0x0080,
        PMDEventCommutationErrorMask=0x0800,
        PMDEventBreakpoint2Mask=0x4000,
        PMDEventStatusMask=0x48FF
};

OS_ENUM_TYPEDEF enum {
        PMDSignalEncoderAMask=0x0001,
        PMDSignalEncoderBMask=0x0002,
        PMDSignalEncoderIndexMask=0x0004,
        PMDSignalEncoderHomeMask=0x0008,
        PMDSignalPositiveLimitMask=0x0010,
        PMDSignalNegativeLimitMask=0x0020,
        PMDSignalAxisInMask=0x0040,
        PMDSignalHallAMask=0x0080,
        PMDSignalHallBMask=0x0100,
        PMDSignalHallCMask=0x0200,
        PMDSignalAxisOutMask=0x0400,
        PMDSignalStepOutMask=0x0800,
        PMDSignalMotorDirectionMask=0x1000,
        PMDSignalMask=0x07FF
};

typedef enum {
    PMDEventActionEventImmediate=0,
    PMDEventActionEventPosLim=1,
    PMDEventActionEventNegLim=2,
    PMDEventActionEventMotionError=3,
    PMDEventActionEventCurrentFoldback=4
} PMDEventActionEventType;

typedef enum {
    PMDEventActionActionNone=0,
    PMDEventActionActionAbruptStop=2,
    PMDEventActionActionSmoothStop=3,
    PMDEventActionActionDisablePosLoopAndHigher=5,
    PMDEventActionActionDisableCurrLoopAndHigher=6,
    PMDEventActionActionDisableMotorOutAndHigher=7,
    PMDEventActionActionAbruptStopWithPosErrorClear=8
} PMDEventActionActionType;

// Encoder
typedef enum {
        PMDCaptureSourceIndex=0,
        PMDCaptureSourceHome=1,
        PMDCaptureSourceHighSpeed=2 // Magellan only
} tagPMDCaptureSource;

typedef enum {
        PMDEncoderSourceIncremental=0,
        PMDEncoderSourceParallel=1
} tagPMDEncoderSource;

// Motor
typedef enum {
        PMDMotorOutputDAC=0,
        PMDMotorOutputPWMSignMagnitude=1,
        PMDMotorOutputPWM5050Magnitude=2
} tagPMDMotorOutputMode;

typedef enum {
        PMDMotorOff=0,
        PMDMotorOn=1
} tagPMDMotorMode;

// Commutation
typedef enum {
        PMDCommutationModeSinusoidal=0,
        PMDCommutationModeHallBased=1,
        PMDCommutationModeMicrostepping=2
} tagPMDCommutationMode;

typedef enum {
        PMDPhaseInitAlgorithmic=0,
        PMDPhaseInitHallBased=1
} tagPMDPhaseInitializeMode;

typedef enum {
        PMDPhaseCorrectionDisabled=0,
        PMDPhaseCorrectionEnabled=1
} tagPMDPhaseCorrectionMode;

typedef enum {
        PMDPhasePrescaleOff=0,
        PMDPhasePrescaleOn=1
} tagPMDPhasePrescaleMode;

typedef enum {
        PMDPhaseA=0,
        PMDPhaseB=1,
        PMDPhaseC=2
} tagPMDPhaseNumber;

// Trace Operations
typedef enum {
        PMDTrace1=0,
        PMDTrace2=1,
        PMDTrace3=2,
        PMDTrace4=3
} tagPMDTraceNumber;

typedef enum {
        PMDTraceOneTime=0,
        PMDTraceRollingBuffer=1
} tagPMDTraceMode;

typedef enum {
        PMDTraceNoVariable=0,
        PMDTracePositionError=1,
        PMDTraceCommandedPosition=2,
        PMDTraceCommandedVelocity=3,
        PMDTraceCommandedAcceleration=4,
        PMDTraceActualPosition=5,
        PMDTraceActualVelocity=6,
        PMDTraceCurrentMotorCommand=7,
        PMDTraceTime=8,
        PMDTraceCaptureValue=9,
        PMDTraceIntegral=10,
        PMDTraceDerivative=11,
        PMDTraceEventStatus=12,
        PMDTraceActivityStatus=13,
        PMDTraceSignalStatus=14,
        PMDTracePhaseAngle=15,
        PMDTracePhaseOffset=16,
        PMDTracePhaseACommand=17,
        PMDTracePhaseBCommand=18,
        PMDTracePhaseCCommand=19,
        PMDTraceAnalogInput1=20,
        PMDTraceAnalogInput2=21,
        PMDTraceAnalogInput3=22,
        PMDTraceAnalogInput4=23,
        PMDTraceAnalogInput5=24,
        PMDTraceAnalogInput6=25,
        PMDTraceAnalogInput7=26,
        PMDTraceAnalogInput8=27
} tagPMDTraceVariable;

typedef enum {
        PMDTraceConditionImmediate=0,
        PMDTraceConditionUpdate=1,
        PMDTraceConditionEventStatus=2,
        PMDTraceConditionActivityStatus=3,
        PMDTraceConditionSignalStatus=4
} tagPMDTraceCondition;

typedef enum {
        PMDTraceTriggerStateLow=0,
        PMDTraceTriggerStateHigh=1
} tagPMDTraceTriggerState;

OS_ENUM_TYPEDEF enum {
        PMDTraceStatusMode=0x0001,
        PMDTraceStatusActivity=0x0002,
        PMDTraceStatusDataWrap=0x0004,
        PMDTraceStatusMask=0x07
};

// Miscellaneous
OS_ENUM_TYPEDEF enum {
        PMDActivityPhasingInitializedBit=0,
        PMDActivityAtMaximumVelocityBit=1,
        PMDActivityTrackingBit=2,
        PMDActivityAxisSettledBit=7,
        PMDActivityMotorOnBit=8,
        PMDActivityPositionCaptureBit=9,
        PMDActivityInMotionBit=10,
        PMDActivityInPositiveLimitBit=11,
        PMDActivityInNegitiveLimitBit=12
};

OS_ENUM_TYPEDEF enum {
        PMDEventMotionCompleteBit=0,
        PMDEventWrapAroundBit=1,
        PMDEventBreakpoint1Bit=2,
        PMDEventCaptureReceivedBit=3,
        PMDEventMotionErrorBit=4,
        PMDEventInPositiveLimitBit=5,
        PMDEventInNegativeLimitBit=6,
        PMDEventInstructionErrorBit=7,
        PMDEventCommutationErrorBit=11,
        PMDEventBreakpoint2Bit=14
};

OS_ENUM_TYPEDEF enum {
        PMDSignalEncoderABit=0,
        PMDSignalEncoderBBit=1,
        PMDSignalEncoderIndexBit=2,
        PMDSignalEncoderHomeBit=3,
        PMDSignalPositiveLimitBit=4,
        PMDSignalNegativeLimitBit=5,
        PMDSignalAxisInBit=6,
        PMDSignalHallABit=7,
        PMDSignalHallBBit=8,
        PMDSignalHallCBit=9,
        PMDSignalAxisOutBit=10,
        PMDSignalMotorOutputInvertBit=12
};

typedef enum {
        PMDAxisOutSourceNone=0,
        PMDAxisOutSourceEventStatus=1,
        PMDAxisOutSourceActivityStatus=2,
        PMDAxisOutSourceSignalStatus=3
} tagPMDAxisOutSource;

typedef enum {
        PMDDiagnosticPortModeLimited=0,
        PMDDiagnosticPortModeFull=1
} tagPMDDiagnosticPortMode;

typedef enum {
        PMDAxisOff=0,
        PMDAxisOn=1
} tagPMDAxisMode;

typedef enum {
        PMDSerialBaud1200=0,
        PMDSerialBaud2400=1,
        PMDSerialBaud9600=2,
        PMDSerialBaud19200=3,
        PMDSerialBaud57600=4,
        PMDSerialBaud115200=5,
        PMDSerialBaud230400=6,  // Magellan and newer chipsets only
        PMDSerialBaud250000=6,  // Navigator and Pilot chipsets only
        PMDSerialBaud416667=7,  // Navigator and Pilot chipsets only
        PMDSerialBaud460800=7   // Magellan and newer chipsets only
} tagPMDSerialBaudRate;

typedef enum {
        PMDSerial1StopBit=0,
        PMDSerial2StopBits=1
} tagPMDSerialStopBits;

typedef enum {
        PMDSerialProtocolPoint2Point=0,
        PMDSerialProtocolMultidropMagellan=1,
        PMDSerialProtocolMultiDropUsingAddressBit=2,
		PMDSerialProtocolMultiDropUsingIdleLineDetection=3
} tagPMDSerialProtocol;

typedef enum {
        PMDSerialParityNone=0,
        PMDSerialParityOdd=1,
        PMDSerialParityEven=2
} tagPMDSerialParity;

typedef enum {
        PMDBrushedServo=1,
        PMDBrushlessServo=3,
        PMDMicroStepping=4,
        PMDStepping=5
} tagPMDMotorType;

typedef enum {
    PMDBrushless3Phase = 0,
    PMDBrushless2Phase = 1,
    PMDMicrostepping3Phase = 2,
    PMDMicrostepping2Phase = 3,
    PMDPulseAndDirection = 4,
    PMDDCBrush = 7
} tagMagellanMotorType;

#endif
