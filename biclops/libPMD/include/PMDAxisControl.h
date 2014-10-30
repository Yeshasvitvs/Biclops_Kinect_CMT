#if !defined PMDAXISCONTROL_H
#define PMDAXISCONTROL_H

#include <ostream>

#include <cstdio>       // for FILE defn
using namespace std;    // for ofstream defn

class PMDController;

#include "PMD.h"
#include "PMDMemMap.h"

class PMDAxisControl : public PMD {
public:

    typedef struct {
		PMDuint16 kP;           // Proportional gain
		PMDuint16 kD;           // Derivative gain
		PMDuint16 kI;           // Integral gain
		PMDuint16 kVFF;         // Velocity feed-forward
		PMDuint16 kAFF;         // Acceleration feed-forward
        PMDuint16 kOut;         // Output scale factor (%)
		PMDuint32 intLim;       // integration limit count*cycles
		PMDuint16 motorLim;     // % of maximum
		PMDuint16 motorBias;    // Motor bias voltage(% of output)
		PMDuint32 errorLim;     // counts
    } ServoFilter;

    typedef struct {
        PMDint32    pos;        // Counts
        PMDint32    vel;        // Counts/second
        PMDuint32   acc;        // Counts/second/second
        PMDuint32   dec;        // Counts/second/second
        PMDuint32   jerk;       // Counts/second/second/second
    } CountsProfile;

    typedef struct {
        double pos;                 // Units
        double vel;                 // Units/second
        double acc;                 // Units/second/second
        double dec;                 // Units/second/second
        double jerk;                // Units/second/second/second
    } Profile;

    // Control chip cycle times
    static const double CyclePeriodStepSizeInUSec;
    static const int NormalControllerCycleMin;
    static const int ExtendedControllerCycleMin;

    char uniqueID[50];
    PMDuint8 ctrlID;

    PMDAxisControl(char *uniqueID, PMDController *controller,
                   PMDAxis axis = PMDAxis1);
//            PMDIOTransport* pmdIOTransport = NULL, PMDAxis axis = PMDAxis1);
//    PMDAxisControl(char *uniqueID, PMDController *controller,
//            PMDAxis axis = PMDAxis1);
    PMDAxisControl(FILE *file, char *token); // Constructor using configuration file.
    virtual ~PMDAxisControl() {};

    void SetDefaults();

    PMDController *GetController() {return controller;};

    // Configures the chip settings for the specific axis interface.
    // This method may be overridden from the defaults. 
    // See the implementation for default configuration details.
    virtual bool ConfigureHardware();

    // Load/store all axis parameters to/from a file.
    static const char *StartToken;
    static const char *EndToken;
    void GetAxisConfigBuffInfo(PMDuint16 &buffID, PMDuint32 &buffAddr);
    bool ReadConfig(FILE *file, char *token);
//    bool WriteConfig(FILE *file);
    bool CheckRead(int buff, int sum);
    bool ReadConfigDataFromMemory(int buff);

    // Does Write, Validate, and Writes Validation flag
    bool BurnAxisConfig();

    bool WriteConfigDataToMemory(int buff);
    bool ValidateConfigDataInMemory(int buff);
    void WriteConfigToFile(ostream& fp, char* indent);

    // Clear all flags that indicate memory has valid data.
    void InvalidateMemoryContents();

    // Access to the servo control parameters. With the homing flag asserted 
    // (true), these methods get/set the homing control parameters.
    void UpdateActualActiveFilter();
    void GetFilter(ServoFilter& sp, bool homing = false, bool currently_running = false);
    void SetFilter(const ServoFilter& sp, bool homing = false);
    void LoadRunFilter();

    // Access to profile generator parameters
    tagPMDProfileMode GetProfileMode();
    bool SetProfileMode(tagPMDProfileMode mode);
    void GetProfile(Profile& profile, bool homing = false);
    void GetRunProfile(Profile& profile){profile = runProfile;}
    void GetProfile(CountsProfile& profile, bool homing = false);
    void SetProfile(const Profile& profile, bool homing = false, 
                    bool doLoad = true);
    void SetProfile(const CountsProfile& profile, bool homing = false, 
                    bool doLoad = true);
    bool CropProfile(CountsProfile& profile);

    // Homing
    enum HomingMode {NoHoming, ReverseLimit, ForwardLimit, BothLimits, HomeAndIndex, BestWay};
    void SetHomedState(bool state);
    bool GetHomedState();
    bool FindHome(bool forceHoming = false, bool writeHomeFile=true);
    HomingMode GetHomingMode() {return homingMode;};
    void SetHomingMode(HomingMode newHomingMode) {homingMode = newHomingMode;};
    void SetHomingLimits(
		    const PMDlong32 fwdHardLimit, const PMDlong32 revHardLimit,
                    const PMDlong32 softLimitForward, const PMDlong32 softLimitReverse);
    void GetHomingLimits(
		    PMDlong32& fwdHardLimit, PMDlong32& revHardLimit,
                    PMDlong32& softLimitForward, PMDlong32& softLimitReverse);
    void CalibrateAnalogSensor();
    void FindMaxSpeed();

    // Axis motion control
    static const PMDuint16 MoveEventMask;
    static const PMDuint16 AutostopEventMask;
    void SetMoveWaitPeriod(u_int32_t waitPeriod) {this->waitPeriod = waitPeriod;};
    u_int32_t GetMoveWaitPeriod() { return waitPeriod;};
    void PreMove(); // Only call if NOT using Move()
    void PostMove();                        // Only call if NOT using Move()
    bool WaitForMotionToStop(bool includeCaptureEvent = false);
    bool Move(bool waitForMotionToStop = true, bool doUpdate = true);
    bool MoveWithCaptureInterrupt(bool waitForMotionToStop = true);
    bool PositionErrorOccured(bool forceGet = false);

    bool Halt(bool waitForMotionToStop=false);
    // Returns from move if capture event occurs
    bool Move(bool waitForMotionToStop, double pos, 
        double vel = 0.0, double acc = 0.0,
	      double dec = 0.0, double jerk = 0.0,
	      bool doUpdate = true);
    bool Move(bool waitForMotionToStop, PMDint32 pos, 
        PMDint32 vel = 0, PMDuint32 acc = 0,
	      PMDuint32 dec = 0, PMDuint32 jerk = 0,
	      bool doUpdate = true);
    void EnableLimitSensorProtection();
    void DisableLimitSensorProtection();


    // PseudoVel move commands implement velocity profile moves but adds 
    // motion limits to protect the axis from runaway conditions that can 
    // occur when communication is lost.
    bool MovePseudoVel(bool waitForMotionToStop,
		       PMDint32 vel = 0, PMDuint32 acc = 0,
		       PMDuint32 dec = 0,
		       bool doUpdate = true);
    bool MovePseudoVel(bool waitForMotionToStop, 
		       double vel = 0.0, double acc = 0.0,
		       double dec = 0.0,
		       bool doUpdate = true);
    
    bool IsMotionDone(bool includeCaptureEvent = false);

    // Amplifier control
    bool IsAmpEnabled() { return ampIsOn;};
    void EnableAmp();
    void DisableAmp();
    void SetMotorTypeInHardware(bool isBrushless);

    // Brake control
    bool HasBrake() { return hasBrake;};
    void EngageBrake();
    void DisengageBrake();
    bool IsBrakeEngaged();

    bool IsEStopped();

    // Motor type setting/checking
    PMDuint16 GetMotorType();
    void SetMotorType(PMDuint16 motorType);

    // Composite joint commands
    void Park();    // Lock brake, disable amp, and turn off closed loop control.
    void Unpark();  // Turn on closed loop control, enable amp, and unlock brake.
    bool ParkAndSave(); // Park axis and save joint state to file
    bool RecoverFromParkAndSave();  // Just what it says

    // Encoder/real world units conversion
    u_int32_t GetCountsPerEncoderCycle () {return countsPerEncoderCycle;};
    double CountsToUnits(PMDint32 val) {return ((double)val)/countsPerAxisCycle;};
    PMDint32 UnitsToCounts(double val) {
        return (PMDint32)(val*countsPerAxisCycle);};
    void CountsToUnits(Profile& profile, const CountsProfile& counts);
    void UnitsToCounts(const Profile& profile, CountsProfile& counts);

    // Conversions to/from native PMD units
    // These conversions are only used for sending values to a PMD controller
    // or getting values back from there. The PMD units are not used otherwise.
    PMDint32    CountsVelToPMD(PMDint32 countsVel);
    PMDuint32   CountsAccToPMD(PMDuint32 countsAcc);
    PMDuint32   CountsJerkToPMD(PMDuint32 countsJerk);
    PMDint32    PMDToCountsVel(PMDint32 pmdVel);
    PMDuint32   PMDToCountsAccToPMD(PMDuint32 pmdAcc);
    PMDuint32   PMDToCountsJerkToPMD(PMDuint32 pmdJerk);

    // Get the latest value of the event status register.
    PMDuint16 GetLastEventStatus();

    // Communication changes
    bool ChangeComm(int baud);

    // File parsing helper methods.
    bool ReadAngle(FILE *file, char *token, double &angle);
    bool ReadAxisStateFromFile();
    bool WriteAxisStateToFile();

private:
    
    enum {BrakeEngaged = 0, BrakeDisengaged = 1};
    static const PMDuint16 UserIOAddress;

    enum Polarity {ActiveHigh = 0, ActiveLow = 1};

    //Axis # within the controller is stored in PMDTransport via GetAxisNumber()

    bool configurationWasLoaded;    // True if config data was loaded from file

    // Actuator info
    bool            hasBrake;
    bool            brakeIsEngaged;
    tagPMDMotorOutputMode 
                    ampType;
    Polarity        ampPolarity;
    bool            isBrushless;
    unsigned short  poleCount;
    tagPMDPhaseInitializeMode
                    phaseInitMode;
    bool            invertHalls;
    tagPMDCommutationMode 
                    commutationMode;
    bool            invertMotorOut;

    // Encoder info
    bool            hasEncoderIndex;
    PMDuint32       countsPerEncoderCycle;
    double          encoderCyclesPerAxisCycle;
    double          motorCyclesPerEncoderCycle;
    double          countsPerAxisCycle;
    double          countsPerMotorCycle;
    PMDuint32       rangeOfMotion;

    // Physical parameters
    PMDlong32       fwdLimitCounts;     // + motion hard limit
    PMDlong32       revLimitCounts;     // - motion hard limit
    double          radiusMeters;       // axis radial dimension
    double          lengthMeters;       // axis overall length
    double          dhAlphaRadians;     // DH alpha
    double          proximalAMeters;    // perpendicular distance from 
                                        // proximalD to pitch rotation axis
    double          proximalDMeters;    // distance from proximal mating 
                                        // surface to pitch axis along 
                                        // proximal roll rotation axis
    double          distalAMeters;      // perpendicular distance from 
                                        // distalD to pitch rotation axis
    double          distalDMeters;      // distance from distal mating 
                                        // surface to pitch axis along 
                                        // distal roll rotation axis

    // Analog position sensor info
    bool            hasAnalogSensor;        // true if analog sensor exists
    PMDuint16       absPosAnalogChannel;    // analog input ID of sensor
//    PMDuint16       analogZero;             // analog value of zero postion
    double          aLUT[4];

    // Homing info
    HomingMode      homingMode;
    bool            isHomed;
    PMDint32        calibratedHomeOffset;
    bool            hasLimitSensors;
    bool            hasHardLimits;          // true if axis has hard limits
    bool            hasHomeSensor;          // true if a home sensor exists
//    u_int32_t       homeSensorWidth;
    PMDuint32       homeSensorWidth;
    Polarity        homeSignalPolarity;
    Polarity        limitSignalPolarity;
    bool            limitsMayBind;          // true if hard limits can bind
                                            // such as with a screw drive
    ServoFilter     homingFilter;
    Profile         homingProfile;
    CountsProfile   homingProfileCounts;
    CountsProfile   homeCfgCounts;   // Added to keep track of original homing Profile
    PMDlong32       softLimitPad;
    PMDlong32       fwdSoftLimitCounts;     // + motion soft limit
    PMDlong32       revSoftLimitCounts;     // - motion soft limit
    PMDlong32       homeDelta;
    u_int32_t       waitPeriod;
    bool mustEnableClosedLoopControl;
    tagPMDProfileMode profileMode;

    ServoFilter     runFilter;
    Profile         runProfile;

    // active control values
    bool            eventStatusLoadedSinceLastMove;
    PMDuint16       signalSense;
    PMDuint16       eventStatus;
    PMDuint16       activityStatus;
    PMDuint16       signalStatus;
    ServoFilter     activeFilter;
    Profile         cmdProfile;
    Profile         actProfile;
    CountsProfile   cmdProfileCounts;
    CountsProfile   loadedProfileCounts;
    CountsProfile   actProfileCounts;
    bool            ampIsOn;
    bool            brakeEngaged;
    bool            captureEventTriggered;

    int debugLevel;
    static const char*  classStr;

    void GoToLimit(CountsProfile &p);
    void WithdrawFromLimit(CountsProfile &p);
    bool FindHomeWithLimits();
    bool FindHomeWithHomeSensor();
    bool FindHomeWithHomeSensor2();
    bool FindHomeBestWay();
    bool FindIndex(int moveDir,PMDint32 &indexPos, 
                   PMDint32 motionLL, PMDint32 motionUL);
    bool CaptureEventTriggered();
    bool HomeSensorIsAsserted();
    bool FindHomeCenter(int &moveDir, PMDint32 &homeCenter,
                        PMDint32 motionLL, PMDint32 motionUL);
    bool FindLimit(PMDint32 relativeGoal, PMDint32 &foundLimitPos);
    PMDint32 EstimateAbsPosFromAnalogSensor(PMDuint16 analog);
    PMDuint16 SampleAnalog(PMDuint16 analogID, int sampleCount);

    // Transfers commanded values to controller
    void SendProfileToController(); // Sends already loaded values to controller
    void SendProfileToController(const CountsProfile& countsProfile);

    // Reference to parent controller for access to cycle time value.
    PMDController   *controller;
    double GetCycleTimeInSecs();

    // File parsing helper methods.
    bool ReadPolarity(FILE *file, char *token, Polarity &polarity);
    void ReadMotorParameters(FILE *file, char *token);
    void ReadEncoderParameters(FILE *file, char *token);
    void ReadAbsPosParameters(FILE *file, char *token);
    void ReadHomingParameters(FILE *file, char *token);
    void ReadProfile(FILE *file, char *token, Profile &profile);
    void ReadFilter(FILE *file, char *token, ServoFilter &filter);
    void ReadPhysical(FILE *file, char *token);

    typedef union {
        PMDint32 i;
        float    f;
    } PMDMemoryFloatOverlayType;
    typedef union {
        PMDint32 i;
        char str[4];
    } PMDMemoryCharsOverlayType;

    void ReadCountsProfileFromMemory(int buff, CountsProfile &prof);
    void ReadServoFilterFromMemory(int buff, ServoFilter &filter);
    void WriteCountsProfileToMemory(int buff, CountsProfile prof);
    void WriteServoFilterToMemory(int buff, ServoFilter filter);
    bool ValidateCountsProfileInMemory(int buff, CountsProfile prof);
    bool ValidateServoFilterInMemory(int buff, ServoFilter filter);
    void WriteProfileToFile(ostream& fp, char* indent,CountsProfile prof);
    void WriteServoFilterToFile(ostream& fp, char* indent, ServoFilter filter);

    void UpdateLimits();

};

#endif

