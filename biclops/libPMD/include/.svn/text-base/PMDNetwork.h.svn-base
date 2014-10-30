#if !defined PMDNETWORK_H
#define PMDNETWORK_H

#include <map>              // data structure used to hold collections
#include <string>           // key data type for collection
using namespace std;        // MSVC++6 needs this to compile

#include "PortableSerial.h"

// Empty class defs so this class can declare collections of them
class PMDController;
class Compass;

class PMDNetwork {
public:

    // Define start/stop tokens for configuration file parsing.
    static const char StartToken[];
    static const char StopToken[];
    static const char StartPMDNetToken[];
    static const char StopPMDNetToken[];

    char uniqueID[50];

    // Type definitions of collections contained in a network.
    typedef map<string,PMDController*> ControllerCollection;
    typedef map<string,Compass*> CompassCollection;

    PMDNetwork(const char *uniqueID, PortableSerial *comm);
    PMDNetwork(FILE *file, char *token); // Constructor using configuration file.
	virtual ~PMDNetwork();

    // Accessors
    PortableSerial *GetSerial() {return serial;};
    bool AddController(char *str, PMDController* controller);
    bool AddCompass(char *str, Compass* compass);
    bool OnlyOneDeviceIsOnNetwork();
    ControllerCollection *GetControllerCollection() {return controllers;};
    CompassCollection *GetCompassCollection() {return compasses;};

    int GetDebugLevel();
    void SetDebugLevel(int level);

    // Change all controllers on network to new configuration
    // Returns true on successful completion, false otherwise.
//    bool ChangeCommSettings(int baud);

private:

    // The implementation of this network
    PortableSerial *serial;

    // The collection of attached devices.
    ControllerCollection *controllers;
    CompassCollection *compasses;

//    int debugLevel;
    static const char*  classStr;

    void ReadConfig(FILE *file, char *token);
};

#endif
