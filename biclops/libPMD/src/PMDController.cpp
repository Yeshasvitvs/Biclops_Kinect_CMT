#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#endif

#include <iostream>         // for cout
using namespace std;
#include <string.h>         // for strcpy
#include <cstdio>          // For sccanf definition
#include <typeinfo>

#include "PMDCollections.h" // for access to Axes collection
#include "PMDController.h"  // class being implemented here
#include "PMDMemMap.h"      // Definitions for accessing memory
#include "Parser.h"         // for parsing config files
#include "DebugMacros.h"

// Debug declarations
const char* PMDController::classStr = "PMDController::";

const char *PMDController::StartToken = "[controller]";
const char *PMDController::EndToken = "[/controller]";

int itoa(int val, char* buf)
{
  const int radix = 10;
  char* p;
  int a;        //every digit
  int len;
  char* b;    //start of the digit char
  char temp;
  
  p = buf;
  if (val < 0) {
    *p++ = '-';
    val = 0 - val;
  }
  
  b = p;
  
  do {
    a = val % radix;
    val /= radix;
    *p++ = a + '0';
  } while (val > 0);
  
  len = (int)(p - buf);
  *p-- = 0;  
  //swap
  do {
    temp = *p;
    *p = *b;
    *b = temp;
    --p;
    ++b;
  } while (b < p);
  
  return len;
}

//----------------------------------------------------------------------------
PMDController::PMDController() {
    SetDefaults();

    // Use of the default constructor implies desire to use the default 
    // communication mechanism as well, which is currently a serial port using
    // the same defaults that the PMD controller uses.
    ioTransport = NULL;
} 

//----------------------------------------------------------------------------
PMDController::PMDController(PMDIOTransport *ioTransport,char *uniqueID) {

    SetDefaults();

    // Record the transport mechanism and indicate that it came from
    // and external source.
    this->ioTransport = ioTransport;
    strcpy(this->uniqueID,uniqueID);

    for (int axis = 0; axis < PMDMaxAxes; axis++) axes[axis] = NULL;
}

//----------------------------------------------------------------------------
void PMDController::SetDefaults() {
    debugLevel = 0;
    ioTransport = NULL;
    primaryAxis = NULL;
    axisCount = 0;
    axisCollection = new AxisCollection;

    memoryExistenceHasBeenChecked = false;
    hasMemory = false;
    memorySize = 0;

    // Clear the digital outputs
    for (int i = 0; i < 256; i++) digitalOutput[i] = 0;

    // Make sure all axis references start out empty.
    for (int axis = 0; axis < PMDMaxAxes; axis++) axes[axis] = NULL;

//    brakeState = 0;

    // Set cycle time to the slowest necessary rate assuming only 
    // 1 axis is running and the longest minimum cycle period.
    // Once the controller is accessible this value will be overwritten
    // with the default value pulled from the controller chip itself.
    cyclePeriodInUSec = (PMDuint16)(
                        PMDAxisControl::ExtendedControllerCycleMin *
                        PMDAxisControl::CyclePeriodStepSizeInUSec);

    gotVersionAlready = false;
}

//----------------------------------------------------------------------------
void PMDController::GenerateUniqueID(
    char* uniqueID, char* networkName, int controllerNumber)
{
    strcpy(uniqueID,networkName);
    strcat(uniqueID,"-");
    char astr[3];
    itoa(controllerNumber,astr);
    if (controllerNumber < 10) strcat(uniqueID,"0");
    strcat(uniqueID,astr);
}

//----------------------------------------------------------------------------
PMDController::PMDController(FILE *file, char *token) {

    // Initialize this instance with the defaults.
    SetDefaults();

    // Load values from the configuration file.
    ReadConfig(file,token);
}

//----------------------------------------------------------------------------
//PMDIOTransport *PMDController::GetIOTransport() {
//    return ioTransport;
//}

//----------------------------------------------------------------------------
PMDAxisControl* PMDController::GetAxis(char *axis) {
    AxisCollection::iterator it = axisCollection->find(axis);
    if (it == axisCollection->end()) return NULL;
    else return it->second;
//        return (*axisCollection)[axis];
}

//----------------------------------------------------------------------------
void PMDController::AddAxis(PMDAxisControl* axis) {

    // Record this axis in the indexable axis table.
    int axisNumber = axis->GetAxisNumber();
    axes[axisNumber] = axis;

    // Add the axis to the string-searchable axis collection.
    (*axisCollection)[axis->uniqueID] = axis;

    // Connect the axis to this controller's transport mechanism.
    axis->SetIOTransport(ioTransport);

    // Create a shortcut to the primary axis if this is it.
    if (axisNumber == 0) {
        primaryAxis = axis;
        if (typeid(PMDSerial) == typeid(*ioTransport))
            ((PMDSerial *)ioTransport)->SetPrimaryAxisInterface(axis);
    }
}

//----------------------------------------------------------------------------
void PMDController::SetIOTransport(PMDIOTransport *ioTransport) {
    this->ioTransport = ioTransport;
    for (int axis = 0; axis < PMDMaxAxes; axis++)
        if (axes[axis] != NULL) 
            axes[axis]->SetIOTransport(ioTransport);
}

//----------------------------------------------------------------------------
void PMDController::ReadConfig(FILE *file, char *token) {

    // Define the tokens used here.
    const char TokenDebugLevel[] = "debugLevel";

    bool tokenRecognized = true;
    unsigned int val;

    // Prime token stream.
    Parser::ReadToken(file,token);

    // Parse the stream.
    do {
        if (!strcmp(token,PMDUtils::TokenUniqueID)) {
            Parser::ReadToken(file,token);
            strcpy(uniqueID,token);
        } else if (!strcmp(token,TokenDebugLevel)) {
            Parser::ReadToken(file,token);
            if (sscanf(token,"%u",&val) == 1) {
                SetDebugLevel(val);
            } else tokenRecognized = false;
        } else if (!strcmp(token,PMDSerial::StartToken)) {
            ioTransport = new PMDSerial(file,token);
            if (!strcmp(token,PMDSerial::StopToken)) {
//                cout << "network = " << ((PMDSerial *)ioTransport)->GetNetwork();
                ((PMDSerial *)ioTransport)->GetNetwork()->
                    AddController(uniqueID,this);
            } else 
                tokenRecognized = false;
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);
}

//----------------------------------------------------------------------------
bool PMDController::Connect() {
//    char methodStr[] = "Connect: ";

    return ioTransport->Open();
}

//----------------------------------------------------------------------------
void PMDController::Disconnect() {
    //%%%%%%%%%%%%%%%%%%%%%% not sure what to do here
//    primaryAxis->transport->mTransport->Close();
}

//----------------------------------------------------------------------------
bool PMDController::IsConnected() {
    return ioTransport->IsReady();
}

//----------------------------------------------------------------------------
bool PMDController::ChangeComm(int baud) {

    // Change the host interface settings to match.
    PortableSerial *comm = 
        ((PMDSerial *)ioTransport)->GetNetwork()->GetSerial();
    char portName[32];
    int oldBaud;
    PortableSerial::Parity parity;
    PortableSerial:: StopBits stopBits;
    comm->GetConfig(portName,oldBaud,parity,stopBits);
    comm->SetConfig(portName,baud,parity,stopBits);
    return ((PMDSerial *)ioTransport)->Open();
}

//----------------------------------------------------------------------------
bool PMDController::ResetController() {

    char methodStr[] = "ResetController: ";
    // A reset command to any axis of the chipset affects all axes of that
    // chipset, so arbitrarily send the reset command to the first axis.
	// If more than one chip set is present, all of them should be
	// reset here
	PMDuint16 result = primaryAxis->Reset();

	// after the reset the chip will be in the PMDChipsetReset state
	PMDuint16 error;
	result = primaryAxis->GetHostIOError(error);

	// the above command should execute without error but we need to check
	if ( (result != PMD_ERR_OK) || (error != PMD_ERR_ChipsetReset) ) {
        ifDbgCout << "Error: " << PMDUtils::GetErrorMessage(result) << endl;
		return false;
	}
	return true;
}

//----------------------------------------------------------------------------
bool PMDController::GetPMDVersion(PMDuint16 &productFamily,
        PMDuint16 &motorType,         PMDuint16 &axisCount, 
        PMDuint16 &specialAttributes, PMDuint16 &customizationCode,
        PMDuint16 &majorSWVersion,    PMDuint16 &minorSWVersion) {
    // Get version info from chipset and record success for future reference.
    if (gotVersionAlready) {
        productFamily = this->productFamily;
        motorType = this->motorType;
        axisCount = this->axisCount;
        specialAttributes = this->specialAttributes;
        customizationCode = this->customizationCode;
        majorSWVersion = this->majorSWVersion;
        minorSWVersion = this->minorSWVersion;
    } else {
        gotVersionAlready = (primaryAxis->GetVersion(productFamily,motorType,
            axisCount,specialAttributes,customizationCode,majorSWVersion,
            minorSWVersion) == PMD_ERR_OK);
    }
    return gotVersionAlready;
}

//----------------------------------------------------------------------------
tagPMDProductFamily PMDController::GetPMDProductFamily() {
    if (!gotVersionAlready) {
        PMDuint16 pf, mt, ac, sa, cc, majorV, minorV;
        GetPMDVersion (pf, mt, ac, sa, cc, majorV, minorV);
    }
    return (tagPMDProductFamily) productFamily;
}

//----------------------------------------------------------------------------
bool PMDController::HasMemory()
{
    char methodStr[] = "HasMemory: ";
    if (!memoryExistenceHasBeenChecked)
    {
        ifDbgCout << "Checking controller for existence of memory\n";
        primaryAxis->SetBufferStart(BuffIDMemCfg,BuffAddrMemCfg);
        primaryAxis->SetBufferLength(BuffIDMemCfg,MEMORY_CFG_BUFF_SIZE);

        // Write a value to memory, read it back, and make sure it's the same.
        PMDint32 dataWritten = 0x87654321;
        PMDint32 dataRead = 0;
        primaryAxis->WriteBuffer(BuffIDMemCfg,dataWritten);
        primaryAxis->ReadBuffer(BuffIDMemCfg,dataRead);

        hasMemory = (dataWritten == dataRead);
        memoryExistenceHasBeenChecked = true;

        // Get the controller's memory size
        if (hasMemory) 
        {
            primaryAxis->ReadBuffer(BuffIDMemCfg,dataRead);
            memorySize = dataRead;
            ifDbgCout << "Controller has " << memorySize << 
                " 32-bit words of memory\n";
        } else ifDbgCout << "Controller has no memory\n";
   }

    return hasMemory;
}

//----------------------------------------------------------------------------
bool PMDController::ConfigureController() {

    // Determine the number of axes on the controller
    GetPMDVersion(productFamily,motorType,axisCount,
            specialAttributes,customizationCode,majorSWVersion,
            minorSWVersion);

    // Create additional axes if the axis count for this controller is greater
    // than one.
    PMDAxis axis;

    // Create the number of axis instances consistent with the quantity the
    // current controller supports. Avoid overwriting instances that have
    // already been populated.
    for (axis = 0; axis < axisCount; axis++) 
        if (axes[axis] == NULL)
	  axes[axis] = new PMDAxisControl((char*)"DefaultAxisName",this,axis);


    // Clear all axes that are unallowable
    for (; axis <= PMDAxis4; axis++)
        if (axes[axis] != NULL) {
            // Eliminate the axis from all collections.
            PMDAxes::Instance()->erase(axes[axis]->uniqueID);
            axisCollection->erase(axes[axis]->uniqueID);
            axes[axis] = NULL;
        }

    // Get the true sample time from the chip and record it's value for 
    // future calculations.
    primaryAxis->GetSampleTime(cyclePeriodInUSec);

    return ConfigureAxes();

}

//----------------------------------------------------------------------------
bool PMDController::ConfigureAxes() {
    bool success = true;
    for (int axis = PMDAxis1; axis <= axisCount; axis++)
        if (axes[axis] != NULL)
            success = success && axes[axis]->ConfigureHardware();
    return success;
}

//----------------------------------------------------------------------------
void PMDController::SetDebugLevel(int level) {
    debugLevel = level;
    for (int axis = PMDAxis1; axis <= PMDAxis4; axis++)
        if (axes[axis] != NULL)
            axes[axis]->SetDebugLevel(level);
}

//----------------------------------------------------------------------------
unsigned short PMDController::GetDigitalOutput(unsigned char address) {
    return digitalOutput[address];
}

//----------------------------------------------------------------------------
void PMDController::SetDigitalOutput(unsigned char address, unsigned short data) {
    primaryAxis->WriteIO(address,data);
}

