#ifndef PortableSerial_h
#define	PortableSerial_h
//  PortableSerial.h -- A portable serial interface
//
//  TRACLabs -- A division of Metrica, Inc.
//

#ifdef WIN32
    #include <wtypes.h> // Needed for definition of HANDLE type
	#ifndef __MINGW32__
		#define OS_ENUM_TYPEDEF typedef
	#else
		#define OS_ENUM_TYPEDEF
	#endif
#else // Linux
    #include <sys/types.h>
    #include <sys/ioctl.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <signal.h>
    #include <termios.h>
    #include <cstdio>          // for FILE defn
	#define OS_ENUM_TYPEDEF
#endif

#include <pthread.h>        // for mutex protection of serial port

class PortableSerial {
public:

	OS_ENUM_TYPEDEF enum Parity {NoParity, EvenParity, OddParity,
                         MarkParity, SpaceParity};
	OS_ENUM_TYPEDEF enum StopBits {OneStopBit, OneAndAHalfStopBits, TwoStopBits};

    // Define start/stop tokens for configuration file parsing.
    static const char StartToken[];
    static const char StopToken[];

    // Default definitions
    static const char       defaultPortName[];
    static const int        defaultBaud;
    static const StopBits   defaultStopBits;
    static const Parity     defaultParity;

    PortableSerial();
    PortableSerial(FILE *file, char *token); // Constructor using configuration file.
    virtual ~PortableSerial();

    // Access to port parameters.
    void SetConfig(const char* portName, int baud, Parity parity, StopBits stopBits);
    void GetConfig(char* portName, int &baud, Parity &parity, StopBits &stopBits);

    // Open the port using the stored port parameters.
    bool Open();

    // Open the port using the provided port parameters.
    bool Open(const char* portName, int baud, Parity parity, StopBits stopBits);
    void Close();

    // True if the port is currently open.
    bool IsOpen() {return isOpen;};

    // Receives COUNT bytes from the serial port into the DATA buffer);
    bool Read(unsigned char* data, int count);

    // Sends COUNT bytes from the DATA buffer to the serial port.
    bool Write(const unsigned char* data, int count);

    // Atomic Write/Read operation as guaranteed by mutual exclusion.
    bool WriteRead( const unsigned char* wdata, int wcount,
                    unsigned char* rdata, int rcount);

    // support functions that may be called directly
    // once the port is initialized
    bool SetTimeout(int msec);
    bool FlushRecv();

    inline int GetDebugLevel() { return debugLevel;};
    void SetDebugLevel(int level) {debugLevel = level;};

private:

    static const char* classStr;
    bool            isOpen;
    char            portName[32];
	int             baud;
	Parity          parity;
	StopBits        stopBits;
    pthread_mutex_t serialMutex;

    void SetDefaults();

    // Loads serial configuration parameters from a file.
    void ReadConfig(FILE *file, char *token);

    int debugLevel;

#ifdef WIN32
	HANDLE portHandle;
    DWORD GetTime();  
#else //Linux
    int portHandle;
    sigset_t sigSet;
    struct termios termIOData;
    int timeout;
    int TimedRead(unsigned char* rdata, int rcount);
    int64_t GetTime();  
#endif

};

#endif
