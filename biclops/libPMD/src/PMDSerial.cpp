//  PMDSerial.cpp -- (Adapted from PMDw32ser.cpp) serial IO
//
//  TRACLabs -- A division of Metrica, Inc.
//
#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#endif

#include <cstdio>          // For sccanf definition
//#include <stdio.h>
#include <string.h>

#include <iostream>     // for cout
#include <iomanip>      // for setw() method
using namespace std;    // pull in std namespace (for VC++)

#include "PMDTypes.h"
#include "PMDOPCodes.h"
#include "PMDUtils.h"       // for error code conversion
#include "PMDCollections.h" // for access to network collection
#include "PMDSerial.h"      // This class
#include "Parser.h"         // for parsing config files

// Debugging declarations
const char* PMDSerial::classStr = "PMDSerial::";
#define dbgCout cout << classStr << methodStr 
#define ifDbg if (GetDebugLevel() > 0) 
#define ifDbg1 if (GetDebugLevel() > 1) 
#define ifDbgCout ifDbg dbgCout
#define ifDbgCout1 ifDbg1 dbgCout

// only include this if we are running in diagnostics mode
#include "PMDDiagnostics.h"

const tagPMDSerialBaudRate  PMDSerial::defaultPMDBaud = PMDSerialBaud9600;
const tagPMDSerialParity    PMDSerial::defaultPMDParity = PMDSerialParityNone;
const tagPMDSerialStopBits  PMDSerial::defaultPMDStopBits = PMDSerial1StopBit;
const tagPMDSerialProtocol  PMDSerial::defaultPMDProtocol = 
                                PMDSerialProtocolPoint2Point;
const PMDuint8              PMDSerial::defaultPMDMulidropID = 0;

const char *PMDSerial::StartToken = "[comm]";
const char *PMDSerial::StopToken = "[/comm]";

// ------------------------------------------------------------------------
PMDSerial::PMDSerial(  PMDNetwork *network,
                tagPMDSerialProtocol protocol, 
                PMDuint8             multiDropID) {

    SetDefaults();

    this->network = network;
    this->protocol = protocol;
    this->multiDropID = multiDropID;
}

// ------------------------------------------------------------------------
PMDSerial::PMDSerial(FILE *file, char *token) {
    SetDefaults();
    ReadConfig(file,token);
}

// ------------------------------------------------------------------------
void PMDSerial::SetDefaults() {
	verifyChecksum = 1;    // by default always verify the checksum
	diagnostics = 0;	// by default disable diagnostics
    isConnected = false;
    SetDebugLevel(0);
    SetTransportType (SERIAL);
    protocol = defaultPMDProtocol;
    multiDropID = defaultPMDMulidropID;
    commandAttemptLimit = 4;
    network = NULL;
//    prodFamily = PMDFamilyNavigator;
}

// ------------------------------------------------------------------------
PMDSerial::~PMDSerial() {
    Close();
}

// ------------------------------------------------------------------------
void PMDSerial::SetProtocol(tagPMDSerialProtocol pmdProtocol, 
                            PMDuint8 pmdMultiDropID) {
    protocol = pmdProtocol;
    multiDropID = pmdMultiDropID;
}

// ------------------------------------------------------------------------
void PMDSerial::GetProtocol(tagPMDSerialProtocol &pmdProtocol, 
                            PMDuint8 &pmdMultiDropID) {
    pmdProtocol = protocol;
    pmdMultiDropID = multiDropID;
}
   
// ------------------------------------------------------------------------
bool PMDSerial::Open() {

    char methodStr[] = "Open: ";    // Debug defn
    PortableSerial *comm = network->GetSerial();

    bool allowedToChangeComm = network->OnlyOneDeviceIsOnNetwork();

    // Get the host port settings
    char portName[32];
    int baud;
    PortableSerial::Parity parity;
    PortableSerial:: StopBits stopBits;
    comm->GetConfig(portName,baud,parity,stopBits);

    // Validate the host port settings against PMD restrictions.
    bool constrained = ConstrainSerialSettings(baud,parity,stopBits);

    // Is comm currently established with this controller?
    if (isConnected) {

        // Comm is already established, so the implication is that it is
        // desired to reestablish communication at a different comm setting.
        // Only do this if it is allowed.
        if (allowedToChangeComm) {
            if (constrained) comm->SetConfig(portName,baud,parity,stopBits);
            ChangeComm();
        }

    } else {

        // Comm is not established with this controller.

        if (constrained) ifDbgCout << "Host serial settings constrained\n";

        if (allowedToChangeComm) {
            // Try connecting to controller even if it means changing the
            // host port to do so.
            comm->SetConfig(portName,baud,parity,stopBits);
            OpenPortWithSearch();
        } else if (!constrained) {
            // Not allowed to change the host port, but since the host
            // port settings are within PMD constraints, try opening anyway.
            if (!comm->IsOpen()) comm->Open(); // Only open if not already
        } else {
            // Not allowed to change port and the host port settings are
            // invalid. Abort!
            ifDbgCout << "Serial settings invalid for this comm channel\n";
        }
    }

    // Verify that communication with the controller works.
    if (comm->IsOpen()) {

        // This only tells us if there is something on the other end of the wire.
        if (Sync() != PMD_ERR_OK) {
            ifDbgCout << "comm channel open but sync failed\n";
            isConnected = false;
        } else {
            ifDbgCout << "comm channel open and sychronized\n";
            isConnected = true;
        }

    } else isConnected = false;

	return isConnected;
}

// ------------------------------------------------------------------------
int PMDSerial::NextBaud(int baud) {
    switch (baud) {
    case   9600: return  19200;
    case  19200: return  57600;
    case  57600: return 115200;
    case 115200: return 230400;
    case 230400: return 250000;
    case 250000: return 416667;
    case 416667: return 460800;
    default:     return   9600;
    }
}

// ------------------------------------------------------------------------
void PMDSerial::OpenPortWithSearch() {

    char methodStr[] = "OpenPortWithSearch: ";    // Debug defn
    PortableSerial *comm = network->GetSerial();

    // Used to track if the comm port can be opened. Avoids repeated open
    // attempts on bad ports.
    bool commPortOpenable = false;

    // Get the desired serial port baud rate.
    char portName[32]; 
    int desiredBaud,trialBaud;
    PortableSerial::Parity desiredParity,trialParity;
    PortableSerial::StopBits desiredStopBits,trialStopBits;
    comm->GetConfig(portName,desiredBaud,desiredParity,desiredStopBits);

    // Attempt to open the serial port.
    trialBaud = desiredBaud;
    trialParity = desiredParity;
    trialStopBits = desiredStopBits;

    do {
        // If the port is open presently then connection to the 
        // controller occured, but not at the preferred baud. Use the 
        // established connection to change the controller to the desired 
        // settings and then change the host port to match.
        if (comm->IsOpen()) {
            trialBaud = desiredBaud;
            ifDbgCout << "Retrying connection on " << portName << " @ "
                      << trialBaud << " baud\n";
            ConstrainSerialSettings(trialBaud,trialParity,trialStopBits);
            comm->SetConfig(portName,desiredBaud,desiredParity,desiredStopBits);
            ChangeComm();
        } else {

            ConstrainSerialSettings(trialBaud,trialParity,trialStopBits);
            ifDbgCout << "Trying connection on " << portName << " @ "
                << trialBaud <<":8N" <<((trialStopBits == PortableSerial::OneStopBit) ? 1:2) << "\n";
            comm->SetConfig(portName,trialBaud,trialParity,trialStopBits);

            // Try to open the port.
            if (comm->Open()) {

                // Port opened successfully.
                commPortOpenable = true;

                // Test the connection.
                if (Sync() != PMD_ERR_OK) {

                    ifDbgCout << "Connection @" << trialBaud << " failed\n";

                    // step to next speed on failure.
                    trialBaud = NextBaud(trialBaud);
                    trialParity = desiredParity;
                    trialStopBits = desiredStopBits;
                    comm->Close();
                }
            } else {
                trialBaud = NextBaud(trialBaud);
            }
        }
    } while (trialBaud != desiredBaud && commPortOpenable);
}

//----------------------------------------------------------------------------
void PMDSerial::ChangeComm() {
//    char methodStr[] = "ChangeComm: ";

            
    // Get the current controller settings.
    tagPMDSerialBaudRate pmdBaud;
    tagPMDSerialParity   pmdParity;
    tagPMDSerialStopBits pmdStopBits;
    tagPMDSerialProtocol protocol;
    PMDuint8 multiDropID;
    primaryAxis->GetSerialPortMode(pmdBaud,pmdParity,pmdStopBits,
        protocol,multiDropID);
    
    // Get the desired serial port baud rate.
    char portName[32]; 
    int desiredBaud;
    PortableSerial::Parity parity;
    PortableSerial::StopBits stopBits;
    network->GetSerial()->GetConfig(portName,desiredBaud,parity,stopBits);

    // Change the controller's comm settings.
    pmdBaud = PMDSerial::HostToPMDBaud(desiredBaud);
    primaryAxis->SetSerialPortMode(pmdBaud,pmdParity,pmdStopBits,
        protocol,multiDropID);
    
    // Reopen the host interface with the new settings.
    network->GetSerial()->Open();
            
}

// ------------------------------------------------------------------------
PMDuint8 PMDSerial::BuildCommandBuffer( PMDuint8 *cmdBuf, 
                                    const PMDuint16* dataBuf, 
                                    PMDuint8 dataWordCount) {

    unsigned int c=0;
	unsigned int i;
	char checksum;
//    char methodStr[] = "BuildCommandBuffer: ";

	// Load address byte
	cmdBuf[c++] = (char)( (multiDropID) &0x1f);

    // Clear checksum byte (makes it easier to compute checksum later.
    cmdBuf[c++] = 0;

	// Add axis number and command code
	cmdBuf[c++] = (char)(dataBuf[0]>>8);
	cmdBuf[c++] = (char)(dataBuf[0]&0xff);

	// add data (handling byte swapping)
	for (i=1; i<dataWordCount; i++) {
		cmdBuf[c++] = (char)(dataBuf[i] >> 8);
		cmdBuf[c++] = (char)(dataBuf[i] & 0xFF);
	}

	// calculate checksum
	for (checksum=i=0; i<c; i++) checksum += cmdBuf[i];
	cmdBuf[1] = -checksum;
    return c;
}

// ------------------------------------------------------------------------
PMDuint16 PMDSerial::SendCommand(PMDuint8 xmtWordCount, PMDuint16* xmtData, 
                                 PMDuint8 rcvWordCount, PMDuint16* rcvData) {

	PMDuint8 xbuf[40],rbuf[40];
    PMDuint8 c;
    PMDuint16 status = PMD_ERR_OK;
    PortableSerial *comm = network->GetSerial();

    char methodStr[] = "SendCommand: ";

    c = BuildCommandBuffer(xbuf,xmtData,xmtWordCount);

    // Determine how many extra bytes are required for the controller response
    // based on the communication protocol
    int pBytes = (protocol == PMDSerialProtocolMultidropMagellan) ? 3:2;

    // Send the data buffer.
    unsigned int commandAttempts = 0;
    do {
        status = PMD_ERR_OK;

        // Display values being sent to controller if in debug mode
        ifDbg1 {
            dbgCout << '(' << PMDOpcodeText[xbuf[3]].text << ')'
                    << PMDOpcodeText[xbuf[3]].pad << ' ' << setfill('0');
            unsigned int bufNdx;
            for (bufNdx = c; bufNdx < 8; bufNdx++) cout << "   ";
            for (bufNdx = 0; bufNdx < c; bufNdx++) {
                cout << ' ' << setw(2) << hex << (unsigned int)(xbuf[bufNdx]) << dec;
            }
            cout <<" >" << int(c) << "X-R";
            cout << setfill(' ');
        }

        // Is the serial port configured to be multidrop with an address bit?
	    if (protocol == PMDSerialProtocolMultiDropUsingAddressBit) {

            // Serial port is configured to be multidrop with an address bit.
            // Fake the address bit using the parity bit
            if (!comm->FlushRecv()) {
                status = PMD_ERR_CommPortWrite;
            } else {
                char portName[32];
                int baud;
                PortableSerial::Parity parity;
                PortableSerial::StopBits stopBits;
                comm->GetConfig(portName,baud,parity,stopBits);
                comm->Open(portName, baud, PortableSerial::MarkParity, stopBits);
                if (!comm->Write(xbuf, 1)) {
                    status = PMD_ERR_CommPortWrite;
                } else {
                    comm->Open(portName, baud, PortableSerial::SpaceParity, stopBits);
		            if (!comm->WriteRead(xbuf+1, c-1,rbuf,2*rcvWordCount+pBytes)) 
                        status = PMD_ERR_CommPortRead;
                }
            }
        } else {

            // Serial port is not configured to be multidrop with an address bit.
            // Send the data normally.
            if (xbuf[3] == PMDOPReset) {
                if (!comm->Write(xbuf, c)) 
                    status = PMD_ERR_CommPortWrite;
            }
            else if (!comm->WriteRead(xbuf, c,rbuf,2*rcvWordCount+pBytes)) {
                // try synchronizing and reissuing command.
                status = PMD_ERR_CommPortRead;
            }
        }

        // Continue with comm if there were no errors.
        if (status == PMD_ERR_OK) {

            ifDbg1 {
                if (xbuf[3] == PMDOPReset)
                    cout << "0: (Reset cmd)";
                else {
                    cout << 2*rcvWordCount+pBytes << setfill('0') << "<";
                    for (unsigned char bufNdx = 0; bufNdx < 2*rcvWordCount+pBytes; bufNdx++)
                        cout << ' ' << setw(2) << hex << (int)(rbuf[bufNdx]) << dec;
                }
                cout << endl << setfill(' ');
            }

	        // verify the checksum
            char checksum = 0;
            int i;
	        for (i=0; i<(int)(2*rcvWordCount+pBytes); i++) checksum += rbuf[i];

            if (checksum != 0) {
                status = PMD_ERR_ChecksumError;
            } else {

	            // byte swap return data and stuff it into the array of 16-bit words
                c = pBytes; // Start grabbing bytes after the preamble
	            for( i=0; i<rcvWordCount; i++ ) {
		            rcvData[i]  = (PMDuint16)(rbuf[c++])<<8;
		            rcvData[i] |= (PMDuint16)(rbuf[c++]);
	            }

            }
        }

        // If in .2. mode, syncing can be accomplished by sending null bytes
        // until a response is received since timeouts are not used.
        // This method generally can't be used for multidrop because the 
        // address byte is non-zero (except controllers at address zero).
        if (status != PMD_ERR_OK) {
            if (protocol == PMDSerialProtocolPoint2Point) {
                ifDbg cout << " comm error '" << PMDUtils::GetErrorMessage(status) 
                     << "' on ctrlr " << (int)xbuf[0] << endl;
			    ReSync();
            } else { 
	      // PMDUtils::DoSleep(500); // Pause for 1/2 second before retrying.
	      PMDUtils::DoSleep(5); // Pause for 5ms before retrying.
	      //PFB Changed this on 11/19/2013 after conversation with
	      //Bryn.  Not tested, but fairly confident that 5ms is
	      //more than enough time to sleep and 500 ms was
	      //overkill.

            }
        }

        commandAttempts++;
    } while (status != PMD_ERR_OK && commandAttempts < commandAttemptLimit);

    // Check the returned status word for errors reported by the chipset.
    // Reflect that status in the method return status.
    // Status byte is in second response byte if in multi-drop on Magellan controller.
    PMDuint8 responseStatusByte = 
        (protocol == PMDSerialProtocolMultidropMagellan) ? rbuf[1]:rbuf[0];
    if (status == PMD_ERR_OK && responseStatusByte == 0) {
        return PMD_ERR_OK;
    } else if (status == PMD_ERR_OK) {
        status = responseStatusByte;
    }

    return status;

}

// ------------------------------------------------------------------------
void PMDSerial::Close() {

/*    if (isConnected) {

        // Reset the serial configuration to defaults before closing the handle.
        char portName[32];
        int baud;
        PortableSerial::Parity parity;
        PortableSerial::StopBits stopBits;
        comm->GetConfig(portName,baud,parity,stopBits);
        comm->Open(portName, baud, PortableSerial::MarkParity, stopBits);
	}
*/
    isConnected = false;

}

// ------------------------------------------------------------------------
// Syncronize with the PMD controller. If the data bus protocol is 
// point-to-point then synchronization is just a matter of sending out a
// series of zeroes until the attached controller responds (implemented
// through the ReSync method). If the data bus is not point-to-point, then
// synchronization can only be verified by issuing a complete command (in
// this case, a NOOP) to the controller and observing the response for 
// correctness.by 
PMDuint16 PMDSerial::Sync() {

    char methodStr[] = "Sync: ";    // Debug defn
    PMDuint16 status = PMD_ERR_OK;

    // If point-to-point protocol, send chars until something comes back.
    if (protocol == PMDSerialProtocolPoint2Point) {
        ifDbgCout << "Protocol is P2P, so ReSync before synching with NoOp\n";
        status = ReSync();
    }

    if (status == PMD_ERR_OK) {

        // Verify that we truely have sync.
        // Send a complete NoOp command and look for a valid response.
        PMDuint16 cmdBuf = PMDOPNoOperation; 
        ifDbgCout << "verify sync with NoOp\n";
        try {
            status = SendCommand(0,&cmdBuf,0,NULL);
        } catch (...) {
            ifDbgCout << "Caught exception\n";
            status = PMD_ERR_CommTimeoutError;
        }
        if (status == PMD_ERR_OK) {
            ifDbgCout << "sync verified\n";
        } else {
            ifDbgCout << "sync verification failed\n";
        }
    }

	return status;

}

// ------------------------------------------------------------------------
// Send out zeros (simulating a NoOp) 
// one at a time until the controller spits back response data. Once 
// a response is seen, purge any remaining data from the receive queue,
// at which point the interface is syncronized.
PMDuint16 PMDSerial::ReSync() {
    char methodStr[] = "ReSync: ";  // Debug defn
    PMDuint16 status = PMD_ERR_OK;
    PortableSerial *comm = network->GetSerial();

	// Flush the receive buffer
	comm->FlushRecv();

    // "Tickle" the serial interface until a response is received.
	const int maxSend = 4; // Should get some response after 4 chars if at right baud.
	int i;
    bool gotResponse = false;
    unsigned char ch = 0;

    ifDbgCout << "Pinging comm port                         ";
	for( i=1; i<=maxSend && status == PMD_ERR_OK && !gotResponse; i++ ) {

		// write a null character
        ifDbg cout << " 00";
        if (!comm->Write(&ch, 1)) {
            status = PMD_ERR_CommPortWrite;
        } else {

		// Attemp to read a response
	    gotResponse = comm->Read(&ch, 1);
        }

	}

	// React to results of tickling
    if (gotResponse) {
        unsigned char ch2;
        gotResponse = comm->Read(&ch2, 1);
        ifDbg
        {
            cout  << " >" << i-1 << setfill('0') <<"X-R2< " << setw(2)
                  << hex << (int)ch;
            if (gotResponse) cout << ' ' << setw(2) << (int)ch2;
            cout << dec << endl;
        }


    } else {
        ifDbg cout << " no response\n";
        status = PMD_ERR_CommTimeoutError;
    }

    //MUST HAVE this sleep to handle when the unit is in 9600 but the
    // machine is talking higher baud rate.
    PMDUtils::DoSleep(1);
    // flush any other data still in the read buffer.  
    comm->FlushRecv();

    return status;

}

// ------------------------------------------------------------------------
int PMDSerial::PMDToHostBaud(tagPMDSerialBaudRate pmdBaud,
                              tagPMDProductFamily family) {
    int baudLUT[] = {1200,2400,9600,19200,57600,115200,250000,416667};
    int baudLUTMagellan[] = {1200,2400,9600,19200,57600,115200,230400,460800};
    if (family == PMDFamilyMagellan)
        return baudLUTMagellan[pmdBaud];
    else
        return baudLUT[pmdBaud];
}

// ------------------------------------------------------------------------
PortableSerial::Parity 
PMDSerial::PMDToHostParity(tagPMDSerialParity pmdParity) {
    PortableSerial::Parity parityLUT[] = {
                            PortableSerial::NoParity,
                            PortableSerial::OddParity,
                            PortableSerial::EvenParity};
    return parityLUT[pmdParity];
}

// ------------------------------------------------------------------------
PortableSerial::StopBits 
PMDSerial::PMDToHostStopBits(tagPMDSerialStopBits pmdStopBits) {
    PortableSerial::StopBits stopBitsLUT[] = {PortableSerial::OneStopBit,
                                              PortableSerial::TwoStopBits};
    return stopBitsLUT[pmdStopBits];
}

// ------------------------------------------------------------------------
tagPMDSerialBaudRate PMDSerial::HostToPMDBaud(int hostBaud) {
    tagPMDSerialBaudRate retVal = PMDSerialBaud9600;
    switch (hostBaud) {
    case   1200: retVal = PMDSerialBaud1200; break;
    case   2400: retVal = PMDSerialBaud2400; break;
    case   9600: retVal = PMDSerialBaud9600; break;
    case  19200: retVal = PMDSerialBaud19200; break;
    case  57600: retVal = PMDSerialBaud57600; break;
    case 115200: retVal = PMDSerialBaud115200; break;
    case 230400: retVal = PMDSerialBaud230400; break;
    case 250000: retVal = PMDSerialBaud250000; break;
    case 416667: retVal = PMDSerialBaud416667; break;
    case 460800: retVal = PMDSerialBaud460800; break;
    default: break;
    }
    return retVal;
}
// ------------------------------------------------------------------------
tagPMDSerialParity 
PMDSerial::HostToPMDParity(PortableSerial::Parity hostParity) {
    tagPMDSerialParity parityLUT[] = {PMDSerialParityNone,
        PMDSerialParityOdd,PMDSerialParityEven,
        PMDSerialParityNone,PMDSerialParityNone}; // for mark&space parities
    return parityLUT[hostParity];

}

// ------------------------------------------------------------------------
tagPMDSerialStopBits 
PMDSerial::HostToPMDStopBits(PortableSerial::StopBits hostStopBits) {
    tagPMDSerialStopBits stopBitsLUT[] = {
        PMDSerial1StopBit,PMDSerial1StopBit,PMDSerial2StopBits};
    return stopBitsLUT[hostStopBits];

}

// ------------------------------------------------------------------------
bool PMDSerial::ConstrainSerialSettings(
        int &baud, PortableSerial::Parity &parity,
        PortableSerial::StopBits &stopBits) {

    // Constrain baud.
    bool constrained = ConstrainBaud(baud);

    // Constrain parity.
    // (No constraints on parity right now)

    // Constrain stop bits only if we can query the chip (i.e., comm to
    // chip is already open), the chip is not Magellan, and the baud setting
    // is 57600 or 115200.
    if (primaryAxis->GetController()->GetNetwork()->GetSerial()->IsOpen()) {
        PMDuint16 generation,motorType,numberAxes,special,custom,major,minor;
        primaryAxis->GetVersion(generation,motorType,numberAxes,special,
            custom,major,minor);
        if (generation != PMDFamilyMagellan &&
            (baud == 57600 || baud == 115200)) {
            constrained = (stopBits != PortableSerial::TwoStopBits);
            stopBits = PortableSerial::TwoStopBits;
        }
    }   // else stop bits must remain unconstrained because there isn't enough
        // information to make a decision.

    return constrained;
}

// ------------------------------------------------------------------------
bool PMDSerial::ConstrainSerialSettings(
        tagPMDSerialBaudRate &baud, tagPMDSerialParity &parity,
        tagPMDSerialStopBits &stopBits) {
    bool constrained = false;
    // Constrain stop bits only if we can query the chip (i.e., comm to
    // chip is already open), the chip is not Magellan, and the baud setting
    // is 57600 or 115200.
    if (primaryAxis->GetController()->GetNetwork()->GetSerial()->IsOpen()) {
        PMDuint16 generation,motorType,numberAxes,special,custom,major,minor;
        primaryAxis->GetVersion(generation,motorType,numberAxes,special,
            custom,major,minor);
        if (generation != PMDFamilyMagellan &&
            (baud == PMDSerialBaud57600 || baud == PMDSerialBaud115200)) {
            constrained = (stopBits != PMDSerial2StopBits);
            stopBits = PMDSerial2StopBits;
        }
    }

    return constrained;
}

// ------------------------------------------------------------------------
bool PMDSerial::ConstrainBaud(int &baud) {
    bool constrained;
    switch (baud) {
    case   1200:
    case   2400:
    case   9600:
    case  19200:
    case  57600:
    case 115200:
    case 230400:
    case 250000:
    case 416667:
    case 460800:
        constrained = false;
        break;
    default: 
        baud = PMDToHostBaud(defaultPMDBaud,prodFamily);
        constrained = true;
        break;
    }
    return constrained;
}

//----------------------------------------------------------------------------
bool PMDSerial::TokenToPMDProtocol(char *token, tagPMDSerialProtocol &protocol) {
    const char TokenPoint2Point[]           = "Point2Point";
    const char TokenMultidropMagellan[]     = "MultidropMagellan";
    const char TokenMultiDropAddressBit[]   = "MultiDropAddressBit";
    const char TokenMultiDropIdleLine[]     = "MultiDropIdleLine";

    bool tokenRecognized = true;
    if (!strcmp(token,TokenPoint2Point)) {
        protocol = PMDSerialProtocolPoint2Point;
    } else if (!strcmp(token,TokenMultidropMagellan)) {
        protocol = PMDSerialProtocolMultidropMagellan;
    } else if (!strcmp(token,TokenMultiDropAddressBit)) {
        protocol = PMDSerialProtocolMultiDropUsingAddressBit;
    } else if (!strcmp(token,TokenMultiDropIdleLine)) {
        protocol = PMDSerialProtocolMultiDropUsingIdleLineDetection;
    } else tokenRecognized = false;
    return tokenRecognized;

}

//----------------------------------------------------------------------------
void PMDSerial::ReadConfig(FILE *file, char *token) {

    // Define the tokens used here.
    const char TokenNetwork[] = "network";
    const char TokenProtocol[] = "protocol";

    bool tokenRecognized = true;
    unsigned int val;

    // Prime token stream.
    Parser::ReadToken(file,token);

    // Parse the stream.
    do {
        if (!strcmp(token,TokenNetwork)) {

            // Get the name of the associated network.
            Parser::ReadToken(file,token);

            // Get access to the named network and attach it to this controller.
            NetworkCollection *networks = PMDNetworks::Instance();
            network = (*networks)[token];
            if (network == NULL) tokenRecognized = false;

        } else if (!strcmp(token,TokenProtocol)) {
            Parser::ReadToken(file,token);
            tokenRecognized = TokenToPMDProtocol(token,protocol);
//            if (!strcmp(token,TokenPoint2Point)) {
//                protocol = PMDSerialProtocolPoint2Point;
//            } else if (!strcmp(token,TokenMultidropMagellan)) {
//                protocol = PMDSerialProtocolMultidropMagellan;
//            } else if (!strcmp(token,TokenMultiDropAddressBit)) {
//                protocol = PMDSerialProtocolMultiDropUsingAddressBit;
//            } else if (!strcmp(token,TokenMultiDropIdleLine)) {
//                protocol = PMDSerialProtocolMultiDropUsingIdleLineDetection;
//            } else tokenRecognized = false;
            if (tokenRecognized && protocol != PMDSerialProtocolPoint2Point) {
                Parser::ReadToken(file,token);
                if (sscanf(token,"%u",&val) == 1) {
                    multiDropID = val;
                } else tokenRecognized = false;
            }
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);
}
