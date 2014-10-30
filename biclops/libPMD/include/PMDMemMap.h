#ifndef PMDMEMMAP_H
#define PMDMEMMAP_H

//////////////////////////////////////////////////////////////////////////
// Copyright � 2011 by Bryn Wolfe and TRACLabs Inc.                     //
// All rights reserved. No part of this software or source code may be  //
// used or reproduced in any form or by any means, or stored in a       //
// database or retrieval system, without prior written approval from    //
// TRACLabs. This notice must not be removed or modified under          //
// penalty of United States copyright law.                              //
//////////////////////////////////////////////////////////////////////////
//  PMDMemMap.h

// PROGRAMMERS NOTE:
// This header file defines the memory address space for TRACLabs PMD
// controllers having memory.
/*
 * PMDMemMap.h
 *
 *  Created on: Jun 14, 2011
 *      Author: bwolfe
 */

// PMD memory buffer IDs
typedef enum {
    BuffIDAxis0     =  0,   // 0x100 32-bit locations
    BuffIDAxis1     =  1,   // 0x100 32-bit locations
    BuffIDAxis2     =  2,   // 0x100 32-bit locations
    BuffIDAxis3     =  3,   // 0x100 32-bit locations
    BuffIDCtlr      =  4,   // 0x100-4 32-bit locations
    BuffIDMisc      =  5,   // use for any buffer access
    BuffIDMemCfg    = 30,   // 2 32-bit location
    BuffIDConfigReg = 31    // 2 32-bit locations
} PMDMemBuffID;

// PMD memory map
typedef enum {
    BuffAddrConfigRegs  = 0x10000,
    BuffAddrMemCfg      = 0x10002,
    BuffAddrCtlr        = 0x10004,
    BuffAddrAxis0       = 0x10100,
    BuffAddrAxis1       = 0x10200,
    BuffAddrAxis2       = 0x10300,
    BuffAddrAxis3       = 0x10400,
    BuffAddrFreeSpace   = 0x10500
} PMDMemMap;

// Memory configuration offsets
typedef enum {
    MemCfgBuffOffsetHasMem          = 0,    // read/write to verify mem exists
    MemCfgBuffOffsetMemSize         = 1     // Size of memory
} PMDMemCfgBuffOffsets;

// Configuration register memory offsets
typedef enum {
    CfgRegBuffOffsetMotorType       = 0,    // motor type and serial port
    CfgRegBuffOffsetCANPort         = 1     // CAN port cfg register
} PMDCfgRegBuffOffsets;

// PMD axis buffer parameter memory offsets
//OS_ENUM_TYPEDEF enum {
//    CtlrBuffOffsetUniqueName        = 1 // null terminated string
//} PMDCtlrBuffOffsets;

// PMD axis buffer parameter memory offsets
typedef enum {
    AxisBuffOffsetParkInfo          =  0, // 2 values
        // isParked, pos
    AxisBuffOffsetCfgDataValid      =  2, // 1 value
    AxisBuffOffsetMotorParams       =  3, // 9 values
        // MotorType,PoleCount,AmpType,AmpPolarity,HasBrake,PhaseInitMode
        // CommutationMode,InvertHalls,InvertMotor
    AxisBuffOffsetEncoderParams     = 12, // 4 values
        // HasIndex,Counts/EncCycle,EncCycles/AxisCycle,MotorCycles/EncCycle
    AxisBuffOffsetPhysicalParams    = 16, // 9 values 
        // HLL,HUL,radius, len, alpha, proxD, proxA, distD, distA
    AxisBuffOffsetAnalogParams      = 25, // 6 values
        // HasAnalog,AbsPosAnalogChannel,0thTerm,1stTerm,2ndTerm,3rdTerm
    AxisBuffOffsetHomingParams      = 31, // 8 values
        // HomingMode,HomeOffset,HomeSensorWidth,LimitsMayBind,HasLimitSensors
        // HomeSignalPolarity,LimitSignalPolarity,SoftLimitPad
    AxisBuffOffsetHomingProfile     = 39, // 5 values
        // Pos,Vel,Acc,Dec,Jerk
    AxisBuffOffsetHomingFilter      = 44, // 10 values
        // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    AxisBuffOffsetRunProfile        = 54, // 5 values
        // Pos,Vel,Acc,Dec,Jerk
    AxisBuffOffsetRunFilter         = 59, // 10 values
        // Kp,Kd,Ki,Kvff,Kaff,Kout,IntLim,OutLim,Bias,ErrLim
    AxisBuffOffsetAxisNumber        = 69, // 1 value
    AxisBuffOffsetUniqueName        = 70  // null terminated string
} PMDAxisBuffOffsets;

// PMD memory buffer sizes
#define UMA_MEM_SIZE         0x20000
#define CONFIG_REG_BUFF_SIZE 2
#define AXIS_CFG_BUFF_SIZE   0x100
#define CONTROLLER_BUFF_SIZE 0x100
#define MEMORY_CFG_BUFF_SIZE 2

// Memory value used to represent a boolean value of TRUE
#define PMDTRUEMEMVALUE      0xB001EA9

// Peripheral I/O map
// Only the first byte of IO space is defined.
typedef enum {
    UIOMaskBrake        = 0x0001,
    UIOMaskIsBrushless  = 0x0002,
    UIOMaskIsHomed      = 0x0004,
    UIOMaskIsEStopped   = 0x0010

} UIORegAMasks;

#endif /* PMDMEMMAP_H_ */
