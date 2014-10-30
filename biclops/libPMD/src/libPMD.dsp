# Microsoft Developer Studio Project File - Name="libPMD" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libPMD - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libPMD.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libPMD.mak" CFG="libPMD - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libPMD - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "libPMD - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libPMD - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "../lib"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GR /GX /O2 /I "../include" /I "$(MOTIONHOME)/Development/libUtils/include" /I "$(POSIXHOME)/include" /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /FR /FD /c
# SUBTRACT CPP /YX
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ELSEIF  "$(CFG)" == "libPMD - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../lib"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../include" /I "$(MOTIONHOME)/Development/libUtils/include" /I "$(POSIXHOME)/include" /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /FR /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"../lib\libPMDD.lib"

!ENDIF 

# Begin Target

# Name "libPMD - Win32 Release"
# Name "libPMD - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\Compass.cpp
# End Source File
# Begin Source File

SOURCE=.\PMD.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDAxisControl.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDAxisHoming.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDAxisUtils.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDController.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDGet.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDNetwork.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDParallel.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDSerial.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDTransport.cpp
# End Source File
# Begin Source File

SOURCE=.\PMDUtils.cpp
# End Source File
# Begin Source File

SOURCE=.\Sequencing.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=..\include\Compass.h
# End Source File
# Begin Source File

SOURCE=..\include\PMD.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDAxisControl.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDCollections.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDController.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDDiagnostics.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDErrorCodes.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDGet.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDIOTransport.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDMemMap.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDNetwork.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDOPCodes.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDParallel.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDSerial.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDTransport.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDTypes.h
# End Source File
# Begin Source File

SOURCE=..\include\PMDUtils.h
# End Source File
# Begin Source File

SOURCE=..\include\Sequencing.h
# End Source File
# End Group
# End Target
# End Project
