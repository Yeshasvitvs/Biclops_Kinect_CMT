#if !defined PMDCONTROLLER_H
#define PMDCONTROLLER_H

#include <map>              // data structure used to hold collections
#include <string>           // key data type for collection
using namespace std;        // MSVC++6 needs this to compile

#include "PMDSerial.h"      // encapsulation of controller comm settings
#include "PMDAxisControl.h"
#include "PMDMemMap.h"

class PMDController {
public:

    char uniqueID[50];

    // Creates a textual descriptor for a controller by combining the network
    // name with the controller's multidrop ID.
    static void GenerateUniqueID(char* uniqueID, char* networkName, int controllerNumber);

    PMDController();
    PMDController(PMDIOTransport *ioTransport,char *uniqueID);
    PMDController(FILE *file, char *token); // Constructor using configuration file.

    PMDIOTransport *GetIOTransport() {return ioTransport;};
    void SetIOTransport(PMDIOTransport *ioTransport);

    PMDAxisControl* GetAxis(PMDAxis axis) { 
        return (axis < PMDMaxAxes) ? axes[axis] : NULL;
    }

    PMDAxisControl* GetAxis(char *axis);

    inline PMDAxisControl* GetPrimaryAxis() {return primaryAxis;}

    PMDNetwork* GetNetwork() {return ((PMDSerial *)ioTransport)->GetNetwork();}

    // Returns version number of controller as floating point value.
    float GetControllerVersion() {
        return majorSWVersion + (float)minorSWVersion/10;
    }

    void AddAxis(PMDAxisControl* axis);

    static const char *StartToken;
    static const char *EndToken;
    void ReadConfig(FILE *file, char *token);

    // Helper methods that allow the application to modify connectivity
    // without having to manipulate the transport mechanism directly.
    bool Connect();
    bool IsConnected();
    void Disconnect();
    bool ChangeComm(int baud);

    // Send command to put the control chip into a default state.
    bool ResetController();

    // Query the controller chip for its characteristics
    bool GetPMDVersion(PMDuint16 &productFamily, PMDuint16 &motorType, 
        PMDuint16 &axisCount, PMDuint16 &specialAttributes,
        PMDuint16 &customizationCode,PMDuint16 &majorSWVersion,
        PMDuint16 &minorSWVersion);
    tagPMDProductFamily GetPMDProductFamily();

    // Determine whether the controller has memory.
    bool HasMemory();

    // Configure the controller and all of the associated axes.
    bool ConfigureController();

    // Call this to configure each axis that is part of this controller.
    bool ConfigureAxes();

    // Set the controller's cycle time. The cycle time that may be assigned
    // is constrained by a lower limit based on the chipset type and the
    // number of active axes. If the cycle time requested is below the
    // minimum or assignment fails, a value of true will be returned.
    bool SetCycleTime(unsigned short cycleTime);
    unsigned short GetCycleTime() {
        return cyclePeriodInUSec;
    };

    inline int GetDebugLevel() { return debugLevel;};
    void SetDebugLevel(int level);

    // Digital outputs
    unsigned short GetDigitalOutput(unsigned char address);
    void SetDigitalOutput(unsigned char address, unsigned short data);

private:

    // Debug goodies
    int debugLevel;
    static const char*  classStr;

    // Controller axis info
    typedef map<string,PMDAxisControl*> AxisCollection;
    AxisCollection *axisCollection;
    PMDAxisControl* axes[PMDMaxAxes];
    PMDAxisControl* primaryAxis;

    // Parameters read from controller upon first contact.
    bool      gotVersionAlready;
    PMDuint16 productFamily;
    PMDuint16 motorType;
    PMDuint16 axisCount;
    PMDuint16 specialAttributes;
    PMDuint16 customizationCode;
    PMDuint16 majorSWVersion;
    PMDuint16 minorSWVersion;

    PMDuint16 cyclePeriodInUSec;

    // Used to determine if a controller has onboard memory.
    bool memoryExistenceHasBeenChecked;
    bool hasMemory;
    int memorySize;

    PMDIOTransport* ioTransport;

    // Record of current digital output values (since digital outputs can't be
    // directly read from the controller.
    unsigned short digitalOutput[256];

    void SetDefaults();
    void Initialize(PMDIOTransport *ioTransport);
};

#endif

