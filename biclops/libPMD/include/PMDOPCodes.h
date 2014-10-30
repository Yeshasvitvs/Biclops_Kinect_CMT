#ifndef PMD_OPCodes
#define PMD_OPCodes

//  Navigator command code declarations
//
//  Performance Motion Devices, Inc.
//

#if defined(__cplusplus)
extern "C" {
#endif

enum {

	PMDOPNoOperation=				0x00,
    PMDOPSetMotorType=              0x02,   // Magellan only
    PMDOPGetMotorType=              0x03,   // Magellan only
    PMDOPSetBiQuadCoefficient=      0x04,   // Magellan v1.x, deprecated for v2.x by SetPosLoop
    PMDOPGetBiQuadCoefficient=      0x05,   // Magellan v1.x, deprecated for v2.x by GetPosLoop
	PMDOPSetMotorLimit=				0x06,
	PMDOPGetMotorLimit=				0x07,
    PMDOPSetAuxilliaryEncoderSource=0x08,   // Magellan only
    PMDOPGetAuxilliaryEncoderSource=0x09,   // Magellan only
    PMDOPSetSPIMode=                0x0A,   // Magellan only
    PMDOPGetSPIMode=                0x0B,   // Magellan only
    PMDOPSetPWMFrequency=           0x0C,   // Magellan only
    PMDOPGetPWMFrequency=           0x0D,   // Magellan only
	PMDOPSetMotorBias=				0x0F,

	PMDOPSetPosition=				0x10,
	PMDOPSetVelocity=				0x11,
    PMDOPSetCANMode=                0x12,   // Magellan only
	PMDOPSetJerk=					0x13,
	PMDOPSetGearRatio=				0x14,
    PMDOPGetCANMode=                0x15,   // Magellan only
	PMDOPUpdate=					0x1A,
    PMDOPSetOvertemperatureLimit=   0x1B,   // Magellan only
    PMDOPGetOvertemperatureLimit=   0x1C,   // Magellan only
	PMDOPGetCommandedPosition=		0x1D,
	PMDOPGetCommandedVelocity=		0x1E,

	PMDOPSetKp=						0x25,   // Deprecated for Magellan v2.x by SetPosLoop
	PMDOPSetKi=						0x26,   // Deprecated for Magellan v2.x by SetPosLoop
	PMDOPSetKd=						0x27,   // Deprecated for Magellan v2.x by SetPosLoop
	PMDOPSetKvff=					0x2B,   // Deprecated for Magellan v2.x by SetPosLoop
	PMDOPGetPhaseAngle=				0x2C,
	PMDOPGetMotorBias=				0x2D,
    PMDOPRestoreOperatingMode=      0x2E,   // Magellan only
	PMDOPSetInterruptMask=			0x2F,

	PMDOPGetEventStatus=			0x31,
    PMDOPSetBreakpointUpdateMask=   0x32,   // Magellan only
    PMDOPGetBreakpointUpdateMask=   0x33,   // Magellan only
	PMDOPResetEventStatus=			0x34,
	PMDOPGetCaptureValue=			0x36,
	PMDOPGetActualPosition=			0x37,
	PMDOPSetSampleTime=				0x38,   // Replaced with 0x3B on Magellan
	PMDOPReset=						0x39,
	PMDOPGetCurrentMotorCommand=	0x3A,
    PMDOPSetMagellanSampleTime=     0x3B,   // Magellan only (replacement for 0x38)
    PMDOPGetMagellanSampleTime=     0x3C,   // Magellan only (replacement for 0x61)
	PMDOPGetTime=					0x3E,

    PMDOPGetBusVoltage=             0x40,   // Magellan only
    PMDOPSetCurrentFoldback=        0x41,   // Magellan only
    PMDOPGetCurrentFoldback=        0x42,   // Magellan only
    PMDOPSetCurrentControlMode=     0x43,   // Magellan only
    PMDOPGetCurrentControlMode=     0x44,   // Magellan only
    PMDOPSetAxisOutMask=            0x45,   // Magellan only
    PMDOPGetAxisOutMask=            0x46,   // Magellan only
	PMDOPClearPositionError=		0x47,
    PMDOPSetEventAction=            0x48,   // Magellan only
    PMDOPGetEventAction=            0x49,   // Magellan only
	PMDOPGetPosition=				0x4A,
	PMDOPGetVelocity=				0x4B,
	PMDOPGetAcceleration=			0x4C,
	PMDOPSetActualPosition=			0x4D,

	PMDOPGetKp=						0x50,   // Deprecated for Magellan v2.x by GetPosLoop
	PMDOPGetKi=						0x51,   // Deprecated for Magellan v2.x by GetPosLoop
	PMDOPGetKd=						0x52,   // Deprecated for Magellan v2.x by GetPosLoop
    PMDOPGetTemperature=            0x53,   // Magellan only
	PMDOPGetKvff=					0x54,   // Deprecated for Magellan v2.x by SetPosLoop
    PMDOPGetPositionLoopValue=      0x55,   // Magellan only
	PMDOPGetInterruptMask=			0x56,
    PMDOPGetActiveOperatingMode=    0x57,   // Magellan only
	PMDOPGetJerk=					0x58,
	PMDOPGetGearRatio=				0x59,
    PMDOPGetFOCValue=               0x5A,   // Magellan only
	PMDOPMultiUpdate=				0x5B,
    PMDOPSetHoldingCurrent=         0x5E,   // Magellan only
    PMDOPGetHoldingCurrent=         0x5F,   // Magellan only

    PMDOPGetBusVoltageLimits=       0x60,   // Magellan only
	PMDOPGetSampleTime=				0x61,   // Replaced with 0x3C on Magellan
    PMDOPSetBusVoltageLimits=       0x62,   // Magellan only
    PMDOPSetOperatingMode=          0x65,   // Magellan only (replacement for many)
    PMDOPGetOperatingMode=          0x66,   // Magellan only (replacement for many)
    PMDOPSetPositionLoop=           0x67,   // Magellan only
    PMDOPGetPositionLoop=           0x68,   // Magellan only
	PMDOPGetMotorCommand=			0x69,
	PMDOPSetStartVelocity=			0x6A,
	PMDOPGetStartVelocity=			0x6B,
    PMDOPClearDriveFaultStatus=     0x6C,   // Magellan only
    PMDOPGetDriveFaultStatus=       0x6D,   // Magellan only
	PMDOPGetOutputMode=				0x6E,

    PMDOPGetCurrentLoopValue=       0x71,   // Magellan only
	PMDOPSetPhaseInitializeTime=	0x72,
    PMDOPSetCurrentLoop=            0x73,   // Magellan only
    PMDOPGetCurrentLoop=            0x74,   // Magellan only
	PMDOPSetPhaseCounts=			0x75,
	PMDOPSetPhaseOffset=			0x76,
	PMDOPSetMotorCommand=			0x77,
	PMDOPInitializePhase=			0x7A,
	PMDOPGetPhaseOffset=			0x7B,
	PMDOPGetPhaseInitializeTime=	0x7C,
	PMDOPGetPhaseCounts=			0x7D,

	PMDOPSetLimitSwitchMode=		0x80,   // Deprecated for Magellan v2.x by SetEventAction
	PMDOPGetLimitSwitchMode=		0x81,   // Deprecated for Magellan v2.x by GetEventAction
	PMDOPWriteIO=					0x82,
	PMDOPReadIO=					0x83,
	PMDOPSetPhaseAngle=				0x84,
	PMDOPSetNumberPhases=			0x85,   // NOT ON Magellan
	PMDOPGetNumberPhases=			0x86,   // NOT ON Magellan
	PMDOPSetAxisMode=				0x87,   // Deprecated for Magellan v2.x by SetOperatingMode
	PMDOPGetAxisMode=				0x88,   // Deprecated for Magellan v2.x by GetOperatingMode
	PMDOPSetDiagnosticPortMode=		0x89,   // NOT ON Magellan
	PMDOPGetDiagnosticPortMode=		0x8A,   // NOT ON Magellan
	PMDOPSetSerialPortMode=			0x8B,
	PMDOPGetSerialPortMode=			0x8C,
	PMDOPSetEncoderModulus=			0x8D,
	PMDOPGetEncoderModulus=			0x8E,
	PMDOPGetVersion=				0x8F,

	PMDOPSetAcceleration=			0x90,
	PMDOPSetDeceleration=			0x91,
	PMDOPGetDeceleration=			0x92,
	PMDOPSetKaff=					0x93,   // Magellan v1.x, deprecated for v2.x by SetPosLoop
	PMDOPGetKaff=					0x94,   // Magellan v1.x, deprecated for v2.x by GetPosLoop
	PMDOPSetIntegrationLimit=		0x95,   // Magellan v1.x, deprecated for v2.x by SetPosLoop
	PMDOPGetIntegrationLimit=		0x96,   // Magellan v1.x, deprecated for v2.x by GetPosLoop
	PMDOPSetPositionErrorLimit=		0x97,
	PMDOPGetPositionErrorLimit=		0x98,
	PMDOPGetPositionError=			0x99,
	PMDOPGetIntegral=				0x9A,   // Magellan v1.x, deprecated for v2.x by GetPosLoopValue
	PMDOPGetDerivative=				0x9B,   // Magellan v1.x, deprecated for v2.x by GetPosLoopValue
	PMDOPSetDerivativeTime=			0x9C,   // Magellan v1.x, deprecated for v2.x by SetPosLoop
	PMDOPGetDerivativeTime=			0x9D,   // Magellan v1.x, deprecated for v2.x by GetPosLoop
	PMDOPSetKout=					0x9E,
	PMDOPGetKout=					0x9F,

	PMDOPSetProfileMode=			0xA0,
	PMDOPGetProfileMode=			0xA1,
	PMDOPSetSignalSense=			0xA2,
	PMDOPGetSignalSense=			0xA3,
	PMDOPGetSignalStatus=			0xA4,
	PMDOPGetHostIOError=			0xA5,
	PMDOPGetActivityStatus=			0xA6,
	PMDOPGetCommandedAcceleration=	0xA7,
	PMDOPSetTrackingWindow=			0xA8,
	PMDOPGetTrackingWindow=			0xA9,
	PMDOPSetSettleTime=				0xAA,
	PMDOPGetSettleTime=				0xAB,
	PMDOPClearInterrupt=			0xAC,
	PMDOPGetActualVelocity=			0xAD,
	PMDOPSetGearMaster=				0xAE,
	PMDOPGetGearMaster=				0xAF,

	PMDOPSetTraceMode=				0xB0,
	PMDOPGetTraceMode=				0xB1,
	PMDOPSetTraceStart=				0xB2,
	PMDOPGetTraceStart=				0xB3,
	PMDOPSetTraceStop=				0xB4,
	PMDOPGetTraceStop=				0xB5,
	PMDOPSetTraceVariable=			0xB6,
	PMDOPGetTraceVariable=			0xB7,
	PMDOPSetTracePeriod=			0xB8,
	PMDOPGetTracePeriod=			0xB9,
	PMDOPGetTraceStatus=			0xBA,
	PMDOPGetTraceCount=				0xBB,
	PMDOPSetSettleWindow=			0xBC,
	PMDOPGetSettleWindow=			0xBD,
	PMDOPSetActualPositionUnits=	0xBE,
	PMDOPGetActualPositionUnits=	0xBF,

	PMDOPSetBufferStart=			0xC0,
	PMDOPGetBufferStart=			0xC1,
	PMDOPSetBufferLength=			0xC2,
	PMDOPGetBufferLength=			0xC3,
	PMDOPSetBufferWriteIndex=		0xC4,
	PMDOPGetBufferWriteIndex=		0xC5,
	PMDOPSetBufferReadIndex=		0xC6,
	PMDOPGetBufferReadIndex=		0xC7,
	PMDOPWriteBuffer=				0xC8,
	PMDOPReadBuffer=				0xC9,
	PMDOPSetBufferFunction=			0xCA,   // NOT ON Magellan
	PMDOPGetBufferFunction=			0xCB,   // NOT ON Magellan
	PMDOPGetStepRange=				0xCE,
	PMDOPSetStepRange=				0xCF,

	PMDOPSetStopMode=				0xD0,
	PMDOPGetStopMode=				0xD1,
	PMDOPSetAutoStopMode=			0xD2,   // Deprecated for Magellan v2.x by SetEventAction
	PMDOPGetAutoStopMode=			0xD3,   // Deprecated for Magellan v2.x by GetEventAction
	PMDOPSetBreakpoint=				0xD4,
	PMDOPGetBreakpoint=				0xD5,
	PMDOPSetBreakpointValue=		0xD6,
	PMDOPGetBreakpointValue=		0xD7,
	PMDOPSetCaptureSource=			0xD8,
	PMDOPGetCaptureSource=			0xD9,
	PMDOPSetEncoderSource=			0xDA,
	PMDOPGetEncoderSource=			0xDB,
	PMDOPSetMotorMode=				0xDC,   // Deprecated for Magellan v2.x by SetOperatingMode
	PMDOPGetMotorMode=				0xDD,   // Depricated for Magellan v2.x by GetOperatingMode
	PMDOPSetEncoderToStepRatio=		0xDE,
	PMDOPGetEncoderToStepRatio=		0xDF,

	PMDOPSetOutputMode=				0xE0,
	PMDOPGetInterruptAxis=			0xE1,
	PMDOPSetCommutationMode=		0xE2,
	PMDOPGetCommutationMode=		0xE3,
	PMDOPSetPhaseInitializeMode=	0xE4,
	PMDOPGetPhaseInitializeMode=	0xE5,
	PMDOPSetPhasePrescale=			0xE6,
	PMDOPGetPhasePrescale=			0xE7,
	PMDOPSetPhaseCorrectionMode=	0xE8,
	PMDOPGetPhaseCorrectionMode=	0xE9,
	PMDOPGetPhaseCommand=			0xEA,
	PMDOPSetMotionCompleteMode=		0xEB,
	PMDOPGetMotionCompleteMode=		0xEC,
	PMDOPSetAxisOutSource=			0xED,   // Deprecated for Magellan v2.x by SetAxisOutMask
	PMDOPGetAxisOutSource=			0xEE,   // Deprecated for Magellan v2.x by GetAxisOutMask
	PMDOPReadAnalog=				0xEF,

    PMDOPSetSynchronizationMode=    0xF2,   // Magellan only
    PMDOPGetSynchronizationMode=    0xF3,   // Magellan only
	PMDOPAdjustActualPosition=		0xF5,
    PMDOPSetFOC=                    0xF6,   // Magellan only
    PMDOPGetFOC=                    0xF7,   // Magellan only
	PMDOPGetChecksum=				0xF8,
    PMDOPSetUpdateMask=             0xF9,   // Magellan only
    PMDOPGetUpdateMask=             0xFA,   // Magellan only
    PMDOPSetFaultOutMask=           0xFB,   // Magellan only
    PMDOPGetFaultOutMask=           0xFC   // Magellan only

};

#if defined(__cplusplus)
}
#endif

#endif

