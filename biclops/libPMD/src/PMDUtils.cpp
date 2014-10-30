#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#else
#include <sys/time.h>
#endif

#include <cmath>
#include <cstdio>          // for file reading in GetToken()
#include <ctype.h>      // for isspace()
#include <iostream>     // for cout
using namespace std;
#include <string.h>  // for strcpy, strcmp
#include <stdlib.h>  // for abs
#include <pthread.h>

#include "Parser.h"
#include "PMDNetwork.h"
#include "PMDCollections.h"
#include "obstacle.h"

#include "PMDUtils.h"   // class being implemented here

const char PMDUtils::TokenDegrees[] = "degrees";
const char PMDUtils::TokenRadians[] = "radians";
const char PMDUtils::TokenRevolutions[] = "revolutions";
const char PMDUtils::TokenInches[] = "inches";
const char PMDUtils::TokenMeters[] = "meters";
const char PMDUtils::TokenUniqueID[] = "uniqueID";


pthread_mutex_t loadMutex=PTHREAD_MUTEX_INITIALIZER;

//----------------------------------------------------------------------------
void PMDUtils::DoSleep(clock_t wait) {
#ifdef WIN32
    Sleep(wait);
#else
    usleep(wait*1000); //usleep wants microseconds, not milliseconds.
#endif
}

//----------------------------------------------------------------------------

double PMDUtils::GetTime() {
#ifdef WIN32
  return clock();
#else
  struct timeval now;
  gettimeofday(&now,NULL);
  return now.tv_sec*1000.0+now.tv_usec/1000.0+0.5;
#endif
}


//----------------------------------------------------------------------------
int PMDUtils::Crop(int val, int minval, int maxval) {
    int cropval = val;
    if (val < minval) cropval = minval;
    if (val > maxval) cropval = maxval;
    return cropval;
}

//----------------------------------------------------------------------------
int PMDUtils::Rnd(double val) {
    return int(floor(val+0.5));
    //if (val > 0) return (int)(val + 0.5);
    //else return (int)(val - 0.5);
}

//-----------------------------------------------------------------------------
//static
bool PMDUtils::AlmostEqual(double v1, double v2, double nEpsilon)
{
	bool bRet = (((v2 - nEpsilon) < v1) && (v1 < (v2 + nEpsilon)));
	return bRet;
}

//----------------------------------------------------------------------------
bool PMDUtils::EstablishCommunication() {

    char methodStr[] = "PMDUtils::EstablishCommunication: ";  // Debug defn
    bool overallConnectionStatus = true;

    // Open the communication channels for all axes in the collection and
    // verify the connection to each unique controller.
    ControllerCollection::iterator iControl;
    ControllerCollection *controllers = PMDControllers::Instance();
    for(iControl=controllers->begin();iControl!=controllers->end();iControl++) {
        PMDController *controller = (*iControl).second;
        cout << methodStr << "Connecting to " << controller->uniqueID << endl;
        bool status = controller->Connect();
        overallConnectionStatus = overallConnectionStatus && status;
    }

    // Likewise for compasses, open comm channels if necessary and
    // verify the connection.
    CompassCollection::iterator iCompass;
    CompassCollection *compasses = Compasses::Instance();
    for(iCompass=compasses->begin();iCompass!=compasses->end();iCompass++) {
        Compass *compass = (*iCompass).second;
        bool status = compass->Connect();
        overallConnectionStatus = overallConnectionStatus && status;
    }

    cout<<"overallConnectionStatus"<<overallConnectionStatus<<endl;
    return overallConnectionStatus;
}

//----------------------------------------------------------------------------
 bool PMDUtils::ConfigureControllers() {
    ControllerCollection::iterator iter;
    ControllerCollection *controllers = PMDControllers::Instance();
    for(iter=controllers->begin();iter!=controllers->end();iter++) {
        PMDController *controller = iter->second;
        if (!controller->ConfigureController()) return false;

        controller->GetAxis(PMDAxis1)->UpdateActualActiveFilter();
    }

    return true;
}

//----------------------------------------------------------------------------
bool PMDUtils::Initialize(const char *fileName, bool connectToControllers, bool configureControllers) {
	if (PMDUtils::LoadConfiguration(fileName)) 
	{
		if (connectToControllers) 
		{
			if (PMDUtils::EstablishCommunication())  
			{
				if (configureControllers) 
				{
					return PMDUtils::ConfigureControllers();
				}
				return true;
			}
			return false;
		} 
		return true;
	}
	return false;
}

//----------------------------------------------------------------------------
PMDUtils::AxisVector *PMDUtils::CreateAxisLUT(vector<const char*> axisNames) {
    AxisVector *axisVector = new AxisVector();
    CreateAxisLUT(axisNames,*axisVector);
    return axisVector;
}

//----------------------------------------------------------------------------
void PMDUtils::CreateAxisLUT(vector<const char*> axisNames, AxisVector &axisVector) {
    unsigned int i;

    // Count the # of elements to be present in the vector and resize it.
    axisVector.clear();
    axisVector.resize(axisNames.size());

    // Now look up the axis handles by name and record them in the vector
    // indexed by axis number. Assumes the axisNames are in ascending order.
    AxisCollection *axisCollection = PMDAxes::Instance();
    for (i=0; i < axisNames.size(); i++) {
      axisVector[i] = (*axisCollection)[axisNames[i]];
    }
}


//----------------------------------------------------------------------------
void PMDUtils::GetAxisLUT(vector<const char*> &axisNames, AxisVector &axisVector) {
    AxisCollection *axisCollection = PMDAxes::Instance();

    axisNames.clear();
    axisNames.resize((*axisCollection).size());
    axisVector.clear();
    axisVector.resize(axisNames.size());

    AxisCollection::iterator iter = (*axisCollection).begin();

    int i;
    for (i=0; iter != (*axisCollection).end(); iter++, i++) {
      axisNames[i] = iter->first.c_str();
      axisVector[i] = iter->second;
    }
}


//----------------------------------------------------------------------------
bool PMDUtils::Coordinate(AxisList &axes) {

    // These calculations assume none of the axes are in motion.

    // Working parameters. "n" is the # of axes to be coordinated
    double length = 0; // Linear travel distance in n-space
    double speed = 0;  // Speed along n-space vector
    double acc = 0;    // Acceleration along n-space vector
    double dec = 0;    // Deceleration along n-space vector
    double jerk = 0;   // Jerk along n-space vector

    // Accumulate the values from each axis into the n-space vector
    PMDint32 actPos;
//    PMDAxisControl::Profile profile;
    PMDAxisControl::CountsProfile profile;
    AxisList::iterator itAL;
    for (itAL = axes.begin(); itAL != axes.end(); ++itAL) {
        (*itAL)->GetProfile(profile);
        (*itAL)->GetActualPosition(actPos);
        length += pow((double)(profile.pos-actPos),2);
        speed  += pow((double)(profile.vel),2);
        acc    += pow((double)(profile.acc),2);
        dec    += pow((double)(profile.dec),2);
        jerk   += pow((double)(profile.jerk),2);
    }

    length = sqrt(length);
    speed = sqrt(speed);
    acc = sqrt(acc);
    dec = sqrt(dec);
    jerk = sqrt(jerk);

    // Recompute values based on n-space vector components.
    for (itAL = axes.begin(); itAL != axes.end(); ++itAL) {
        (*itAL)->GetProfile(profile);
        (*itAL)->GetActualPosition(actPos);
#ifdef WIN32
        double dist = abs(profile.pos-actPos);
#else
        double dist = labs(profile.pos-actPos);
#endif
        double gain = dist/length;
        profile.vel = (PMDint32)(gain*speed);
        profile.acc = (PMDuint32)(gain*acc);
        profile.dec = (PMDuint32)(gain*dec);
        profile.jerk = (PMDuint32)(gain*jerk);
        (*itAL)->SetProfile(profile);
    }

    return true;

}

//----------------------------------------------------------------------------
bool PMDUtils::Coordinate(ProfileList &profiles) {

    // ASSUMPTIONS:
    // - None of the axes are in motion, i.e., vel(t=0) = 0
    // - acceleration/deceleration are constant, i.e., jerk = 0
    // - travel distance is stored in unccord.pos

    bool done;
    double maxAccelTime = 0.0;
    double maxDecelTime = 0.0;
    double maxSlewTime = 0.0;

    do {

        done = true;
        double maxMoveTime = 0;

        // First, verify that all accelerations/decelerations are nonzero
        ProfileList::iterator it;
        for (it = profiles.begin(); it != profiles.end(); ++it) {
            if (it->uncoord.acc == 0 || it->uncoord.dec == 0) 
                return false;
        }

        // Find the acceleration and slew times for each axis
        for (it = profiles.begin(); it != profiles.end(); ++it) {

            if (it->uncoord.pos != 0) { // skip zero-moves
                // a = v/t -> t = v/a (total time considers decel also)
                it->accelTime = it->uncoord.vel/it->uncoord.acc;
                it->decelTime = it->uncoord.vel/it->uncoord.dec;
    //            double a = it->accelTime ,d = it->decelTime ,
    //                   uv = it->uncoord.vel,ua = it->uncoord.acc,ud = it->uncoord.dec;

                // d = 1/2 *at^2 (same for decel)
                double accelDist = it->uncoord.acc*pow(it->accelTime,2)/2;
                double decelDist = it->uncoord.dec*pow(it->decelTime,2)/2;

                // Is the distance covered by acceleration and deceleration less
                // than the total travel distance?
                if (accelDist + decelDist < it->uncoord.pos) {

                    // Still have some distance that is covered at max velocity
                    it->slewTime = (it->uncoord.pos - accelDist - decelDist)/
                                  it->uncoord.vel;

                } else {

                    // Not enough time to get up to full velocity.
                    // Recompute accel & decel times and set slew time to zero
                    it->accelTime = sqrt(it->uncoord.pos / it->uncoord.acc);
                    it->decelTime = sqrt(it->uncoord.pos / it->uncoord.dec);
                    it->slewTime = 0;

                }
    //            double st = it->slewTime;

                // Keep track of the axis that has the longest move time
                double thisMoveTime = it->accelTime + it->decelTime + it->slewTime;
                if (thisMoveTime > maxMoveTime) {
                    maxMoveTime = thisMoveTime;
                    maxAccelTime = it->accelTime;
                    maxDecelTime = it->decelTime;
                    maxSlewTime = it->slewTime;
                }
            }
        }
        
        // Scale the motion profiles to match the duration of the longest move
        for (it = profiles.begin(); it != profiles.end(); ++it) {
            if (it->uncoord.pos != 0) {

                // Make all accelerations take the same amount of time
                it->coord.acc = it->uncoord.pos/
                             (pow(maxAccelTime,2) + maxAccelTime*maxSlewTime);

                // Make all decelerations take the same amount of time
                it->coord.dec = it->uncoord.pos/
                             (pow(maxDecelTime,2) + maxDecelTime*maxSlewTime);

                // The new slew rate is just whatever the new acceleration can
                // acheive during the initial acceleration time (v=at).
                it->coord.vel = it->coord.acc*maxAccelTime;
//                double ca = it->coord.acc,cv = it->coord.vel,up = it->uncoord.pos;
            }
        }

    } while (!done);

    return true;

}

//----------------------------------------------------------------------------
//void PMDUtils::UpdateAll(AxisList &axes) {
void PMDUtils::UpdateAll(AxisList axes) {
    AxisList::iterator axis;

    for (axis = axes.begin();axis != axes.end();axis++) (*axis)->Update();
    
}

//----------------------------------------------------------------------------
//void PMDUtils::UpdateAll(AxisVector &axes) {
void PMDUtils::UpdateAll(AxisVector axes) {
    AxisVector::iterator axis;

    for (axis = axes.begin();axis != axes.end();axis++) (*axis)->Update();

}
//----------------------------------------------------------------------------
bool PMDUtils::Move(AxisList axes, bool waitForMotionToStop) {

    AxisList::iterator axis;

    // Clear the event status if this multi-axis move
    // wants to wait for motion completion.
    if (waitForMotionToStop)
        for (axis = axes.begin();axis != axes.end();axis++)
            (*axis)->ResetEventStatus(~PMDAxisControl::MoveEventMask);

    // Start moving without waiting for motion to complete.
    for (axis = axes.begin();axis != axes.end();axis++)
        (*axis)->Move(false,false);
    UpdateAll(axes);

    // Wait for all motion to complete if desired.
    if (waitForMotionToStop) 
        for (axis = axes.begin();axis != axes.end();axis++)
            while (!((*axis)->IsMotionDone())) 
                PMDUtils::DoSleep(100);

    return true;
}

//----------------------------------------------------------------------------
bool PMDUtils::Move(AxisVector axes, bool waitForMotionToStop) {

    AxisVector::iterator axis;
	int loopCount = 0;
    // Clear the event status if this multi-axis move
    // wants to wait for motion completion.
    if (waitForMotionToStop)
        for (axis = (axes).begin();axis != axes.end();axis++)
            (*axis)->ResetEventStatus(PMDAxisControl::MoveEventMask);

    // Start moving without waiting for motion to complete.
    for (axis = axes.begin();axis != axes.end();axis++)
        (*axis)->Move(false,false);
    UpdateAll(axes);

    // Wait for all motion to complete if desired.
    if (waitForMotionToStop) 
        for (axis = axes.begin();axis != axes.end();axis++)
		{
			loopCount = 0;
            while (!((*axis)->IsMotionDone()) && (++loopCount < 150)) 
			
                PMDUtils::DoSleep(100);
		}
    return true;
}

//----------------------------------------------------------------------------
bool PMDUtils::Move(Kinematics::DoublesVector &cartesianCoordinate, 
                    AxisVector &axes,
                    Kinematics &kinematics, bool waitForMotionToStop) {

    // Allocate space for the results of inverse kinematics
    Kinematics::DoublesVector *desiredJointAngles = 
        kinematics.AllocateJointsVector();

    // Compute inverse
    if (kinematics.Inverse(cartesianCoordinate,*desiredJointAngles)) {

        // Inverse solution exists.

        // Check for collisions with self or other obstacles.
        Kinematics::CollisionType collisionType = 
            kinematics.DetectCollision(*desiredJointAngles);
        if (collisionType == Kinematics::NO_COLLISION) {

            // No collisions, so transfer new joint positions to axes.
            unsigned short jointCount = kinematics.GetDegreesOfFreedom();
            for (int i = 0; i < jointCount; i++) {
                PMDAxisControl::Profile profile;
                axes[i]->GetProfile(profile);
                profile.pos = RadsToRevs((*desiredJointAngles)[i]);
                axes[i]->SetProfile(profile);
            }

            // Execute the move.
            return Move(axes, waitForMotionToStop);
        }
        else return false;
    }
    else return false;
}

//----------------------------------------------------------------------------
bool PMDUtils::ReadAngle(FILE *file, char *token, double &angle) {
    bool tokenRecognized = true;

    // Get the numeric value and save it in a local buffer.
    Parser::ReadToken(file,token);
    char valToken[10];
    strcpy(valToken,token);

    // Get the units of measure for the angle value and convert it to revolutions.
    float fangle;
    Parser::ReadToken(file,token);
    if (strcmp(token,TokenDegrees) == 0) {
        sscanf(valToken,"%f",&fangle);
        angle = PMDUtils::DegsToRevs((double)fangle);
    } else if (strcmp(token,TokenRadians) == 0) {
        sscanf(valToken,"%f",&fangle);
        angle = PMDUtils::RadsToRevs((double)fangle);
    } else if (strcmp(token,TokenRevolutions) == 0) {
        sscanf(valToken,"%f",&fangle);
        angle = (double)fangle;
    } else
        tokenRecognized = false;
    return tokenRecognized;
}

//-----------------------------------------------------------------------------
bool PMDUtils::ReadLength(FILE *file, char *token, double &length) {
    bool tokenRecognized = true;

    // Get the numeric value and save it in a local buffer.
    Parser::ReadToken(file,token);
    char valToken[10];
    strcpy(valToken,token);

    // Get the units of measure and convert it to meters.
    float f;
    Parser::ReadToken(file,token);
    if (strcmp(token,TokenInches) == 0) {
        sscanf(valToken,"%f",&f);
        length = PMDUtils::InchesToMeters((double)f);
    } else if (strcmp(token,TokenMeters) == 0) {
        sscanf(valToken,"%f",&f);
        length = (double)f;
    } else
        tokenRecognized = false;
    return tokenRecognized;
}
/*
//-----------------------------------------------------------------------------
bool PMDUtils::FindToken(FILE *file, char *token, const char *find) {
    do {
        ReadToken(file,token);
    } while(strcmp(token,find) != 0 && !feof(file));
    return strcmp(token,find) == 0;
}
*/
//-----------------------------------------------------------------------------
bool PMDUtils::LoadConfiguration(const char *fileName, bool recursivelyCalled) {
    if (!recursivelyCalled) pthread_mutex_lock (&loadMutex);

    // Presently, this logic only supports references to objects that already
    // exist. For example, a network must exist before a controller on that
    // network can be defined. Future revisions may support an arbitrary
    // ordering. Although not enforced by this method's logic, there should be
    // at least one axis, and therefore one controller and one network.

    // Define the tokens used here.
    const char TokenFile[] = "file";
    const char TokenObstacle[] = "[obstacle]";

    bool tokenRecognized = true;
    char token[200];

    cout << "Opening file " << fileName << " is... ";
    FILE *file = fopen(fileName, "r");
    if (file == NULL) {
        cout << "unsuccessful. Abandoning configuration file load." << endl;
	    if (!recursivelyCalled) pthread_mutex_unlock (&loadMutex);
        return false;
    }
    cout << "successful." << endl;
    cout << "Parsing file " << fileName << ". " << endl;

    // Prime token stream.
    Parser::ReadToken(file,token);

    // Start parsing.
    // Valid entities are networks, controllers, and axes.
    do {

        // Check for network definition.
        if (!strcmp(token,PMDNetwork::StartToken)) {

            // Create an instance from the file contents
//            char uniqueID[50];
//            Parser::ReadToken(file,token);
//            if (!strcmp(token,TokenUniqueID)) {
//                Parser::ReadToken(file,token);
//                strcpy(uniqueID,token);
//            }

//            PortableSerial* serial = new PortableSerial(file,token);
            PMDNetwork* network = new PMDNetwork(file,token);

            // Verify that parsing went well.
            if (!strcmp(token,PMDNetwork::StopToken)) {

                // Got the expected end token, meaning we have a valid instance.
//                PMDNetwork* network = new PMDNetwork(uniqueID,serial);

                // Put this new instance into the collection
                NetworkCollection *networks = PMDNetworks::Instance();
                (*networks)[network->uniqueID] = network;

            } else tokenRecognized = false;

        // Check for controller definition.
        } else if (!strcmp(token,PMDController::StartToken)) {

            // Create an instance from the file contents
            PMDController *controller = new PMDController(file,token);

            // Verify that parsing went well.
            if (!strcmp(token,PMDController::EndToken)) {

                // Got the expected end token, meaning we have a valid instance.

                // Put this new instance into the controller collection
                ControllerCollection *controllers = PMDControllers::Instance();
                (*controllers)[controller->uniqueID] = controller;

            } else tokenRecognized = false;

        // Check for axis definition.
        } else if (!strcmp(token,PMDAxisControl::StartToken)) {

            // Create an instance from the file contents
            PMDAxisControl *axis = new PMDAxisControl(file,token);

            // Verify that parsing went well.
            if (!strcmp(token,PMDAxisControl::EndToken)) {

                // Got the expected end token, meaning we have a valid instance.

                // Put this new instance into the collection
                AxisCollection *axes = PMDAxes::Instance();
                (*axes)[axis->uniqueID] = axis;

            } else tokenRecognized = false;

        // Check for sequence definition.
        } else if (!strcmp(token,KinematicSequence::StartToken)) {

            // Create an instance from the file contents
            new KinematicSequence(file,token);

            // KinematicSequences store their own instance in an internal
            // collection, so no need to do anything with it here other than
            // verify that parsing went well.
            if (strcmp(token,KinematicSequence::EndToken)) tokenRecognized = false;

        // Check for sequence definition.
        } else if (!strcmp(token,Sequence::StartToken)) {

            // Create an instance from the file contents
            Sequence *sequence = new Sequence(file,token);

            // Verify that parsing went well.
            if (!strcmp(token,Sequence::EndToken)) {

                // Got the expected end token, meaning we have a valid instance.

                // Put this new instance into the collection
                SequenceCollection *sequences = Sequences::Instance();
                (*sequences)[sequence->uniqueID] = sequence;

            } else tokenRecognized = false;

        // Check for kinematics definition.
        } else if (!strcmp(token,Kinematics::StartToken)) {

            // A kinematic instance differs from the other creations here 
            // because the generic kinematics class has no usable content.
            // Consequently, some other entity must create a derived instance
            // of the base Kinematics class and load it into the kinematics
            // "collection". This logic loads the unique ID of the instance,
            // extracts it from the collection, then initializes that
            // instance with the remainder of the kinematics file definition.

            // Load instance ID
            Parser::ReadToken(file,token);
            if (!strcmp(token,TokenUniqueID)) {

                // Get uniqueID
                Parser::ReadToken(file,token);

                // Get and verify kinematics instance.
                KinematicsCollection *kCollection = KinematicSolutions::Instance();
                Kinematics *kinematics = (*kCollection)[token];
                if (kinematics != NULL) {

                    // Configure the instance
                    tokenRecognized = kinematics->Initialize(file,token);

                }
            }

            // Verify that parsing went well.
            tokenRecognized = (strcmp(token,Kinematics::EndToken) == 0);

        // Check for obstacle definition.
        } else if (!strcmp(token,TokenObstacle)) {

            // Reading obstacles belongs in the Obstacle class, but it
            // is simple so we do it here for now.
            // The format is obstacleName, x,y,z for the lower corner (most negative),
            // followed by the upper corner (most positive).
            Parser::ReadToken(file,token);
            char obstacleName[50];
            strcpy(obstacleName,token);
            Vec3_t vertecies[2];
            for (int j = 0; j<2;j++) for (int i=0;i<3;i++)
                ReadLength(file,token,vertecies[j].val[i]);
            CObstacle *obstacle = new CObstacle(&vertecies[0],&vertecies[1]);
            KinematicsCollection *kCollection = KinematicSolutions::Instance();
            Kinematics *kinematics = kCollection->begin()->second;  // assumes only one instance of kinematics
            kinematics->obstacles[obstacleName] = obstacle;

        // Check for compass definition.
        } else if (!strcmp(token,Compass::StartToken)) {

            // Create an instance from the file contents
            Compass *compass = new Compass(file,token);

            // Verify that parsing went well.
            if (!strcmp(token,Compass::EndToken)) {

                // Got the expected end token, meaning we have a valid instance.

                // Put this new instance into the collection
                CompassCollection *compasses = Compasses::Instance();
                (*compasses)[compass->uniqueID] = compass;

            } else tokenRecognized = false;

        // Check for additional configuration file definition.
        } else if (!strcmp(token,TokenFile)) {

            // Get the name of the file to read from.
            Parser::ReadToken(file,token);

            // Figure out the path to the file.
            size_t dirDelimiter = strcspn(fileName,"\\/");
            int pathEnd = dirDelimiter;
            for (int i = dirDelimiter+1; fileName[i] != 0; i++)
                if (fileName[i] == fileName[dirDelimiter]) 
                    pathEnd = i;
            pathEnd++;

            // Create the new file name
            char *newFileName = new char[pathEnd + strlen(token) + 1];
            strncpy(newFileName,fileName,pathEnd);
            newFileName[pathEnd] = 0;
            strcat(newFileName,token);

            tokenRecognized = LoadConfiguration(newFileName,true);

        // Current token does not match anything we're familiar with.
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);

    // Check for parse failure
    if (!feof(file))
        cout << "Parse error in configuration file at " << token << endl;
    fclose(file);
    if (!recursivelyCalled) pthread_mutex_unlock (&loadMutex);
    return tokenRecognized;

}

//-----------------------------------------------------------------------------
//static
const char* PMDUtils::GetErrorMessage(PMDuint16 errorCode) {
    switch (errorCode) {
    case PMD_ERR_OK : return "OK";
    case PMD_ERR_ChipsetReset : return "Chipset Reset" ;
    case PMD_ERR_InvalidInstruction : return "Invalid Instruction" ;
    case PMD_ERR_InvalidAxis : return "Invalid Axis" ;
    case PMD_ERR_InvalidParameter : return "Invalid Data Parameter" ;
    case PMD_ERR_TraceRunning : return "Trace Currently Running" ;
    case PMD_ERR_BadFlashUpdateFile : return "Bad Flash Update File" ;
    case PMD_ERR_BlockOutOfBounds : return "Block Out of Bounds" ;
    case PMD_ERR_TraceBufferZero : return "Trace Buffer Size Zero" ;
    case PMD_ERR_BadSerialChecksum : return "Invalid Checksum" ;
    case PMD_ERR_NotPrimaryPort : return "Invalid Instruction For Diagnostic Port" ;
    case PMD_ERR_InvalidNegativeValue : return "Invalid Negative Value for Profile Mode" ;
    case PMD_ERR_InvalidParameterChange : return "Invalid Parameter Change for Profile Mode" ;
    case PMD_ERR_LimitEventPending : return "Invalid Move with Limit Event Pending" ;
    case PMD_ERR_InvalidMoveIntoLimit : return "Invalid Move into Limit" ;

    case PMD_ERR_NotConnected : return "not connected" ;
    case PMD_ERR_ChipsetNotResponding : return "chipset not responding" ;
    case PMD_ERR_CommPortRead : return "comm port read" ;
    case PMD_ERR_CommPortWrite : return "comm port write" ;
    case PMD_ERR_InvalidCommPortAddress : return "invalid port address" ;
    case PMD_ERR_InvalidCommPortBaudRate : return "invalid port baud" ;
    case PMD_ERR_InvalidCommPortParity : return "invalid port parity" ;
    case PMD_ERR_InvalidCommPortStopBits : return "invalid port stop bits" ;
    case PMD_ERR_InvalidCommPortProtocol : return "invalid port protocol" ;
    case PMD_ERR_InvalidCommPortMDropID : return "invalid port multidrop ID" ;
    case PMD_ERR_InvalidSerialPort : return "Invalid Serial Port" ;
    case PMD_ERR_InvalidDK2000IOBase : return "Invalid DK2000 IO Base" ;
    case PMD_ERR_InvalidDK2000IRQ : return "Invalid DK2000 IRQ" ;
    case PMD_ERR_InvalidDK2000IOMode : return "Invalid DK2000 IO Mode" ;
    case PMD_ERR_DK2000NotInitialized : return "DK2000 Not Initialized" ;

    case PMD_ERR_CommunicationsError : return "Communication Error" ;
    case PMD_ERR_CommTimeoutError : return "Communication Timeout Error" ;
    case PMD_ERR_ChecksumError : return "Checksum Error" ;
    case PMD_ERR_CommandError : return "Command Error" ;
    default : return "Undefined error occurred";
    }
}


std::vector<long double> PMDUtils::polyfit(const std::vector<std::vector<double> >&  data_array, int order)
{
 // Polynomial Interpolation

 int data_pairs=data_array[0].size();

 std::vector<long double> B(order+1,0);
 std::vector<long double> S(order+1,0);
 std::vector<std::vector<long double> > a(order+1,B);
 
 long double A1, A2, Y1, m, S1, x1;

 std::vector<long double> polycoefs(order+1,0);

 int i, j, k, L, R;

 Y1 = 0;
 for( j = 0; j < data_pairs; j++ )
   {
     for( i = 1; i <= order; i++ )
   {
     B[i] = B[i] + data_array[1][j] * pow( data_array[0][j], i );
     for( k = 1; k <= order; k++ )
       {
         a[i][k] = a[i][k] + pow( data_array[0][j], (i + k) );
       }
     S[i] = S[i] + pow( data_array[0][j], i );
   }
     Y1 = Y1 + data_array[1][j];
   }

 for( i = 1; i<= order; i++ )
   {
     for( j = 1; j <= order; j++ )
   {
     a[i][j] = a[i][j] - S[i] * S[j] / (long double) data_pairs;
   }
     B[i] = B[i] - Y1 * S[i] / (long double) data_pairs;
   }

 for( k = 1; k <= order; k++ )
   {
     R = k; A1 = 0;
     for( L = k; L <= order; L++ )
   {
     A2 = fabs( a[L][k] );
     if( A2 > A1 ){ A1 = A2; R = L;}
   }
     if( A1 == 0 ) return polycoefs;
     if( R != k )  {
   for( j = k; j <= order; j++ )
     {
       x1 = a[R][j]; a[R][j] = a[k][j]; a[k][j] = x1;
     }
   x1 = B[R]; B[R] = B[k]; B[k] = x1;
     }
     for( i = k; i <= order; i++ )
   {
     m = a[i][k];
     for( j = k; j <= order; j++ )
       {
         if( i == k )
       a[i][j] = a[i][j] / m;
         else
       a[i][j] = a[i][j] - m * a[k][j];
       }
     if( i == k )
       B[i] = B[i] / m;
     else
       B[i] = B[i] - m * B[k];
   }
   }

 polycoefs[order] = B[order];
 for( k = 1; k <= order - 1; k++ )
   {
     i = order - k; S1 = 0;
     for( j = 1; j <= order; j++ )
   {
     S1 = S1 + a[i][j] * polycoefs[j];
   }
     polycoefs[i] = B[i] - S1;
   }

 S1 = 0;
 for( i = 1; i <= order; i++ )
   {
     S1 = S1 + polycoefs[i] * S[i] / (long double)data_pairs;
   }
 polycoefs[0] = (Y1 / (long double)data_pairs - S1);
 for( i = 0; i <= order; i++ )
   if (polycoefs[i]==-0)
     polycoefs[i]=0;

 std::cout.precision(10);
 std::cout << "Coeffs: \n";
 for(i = 0; i <= order; i++ )
   std::cout << (float)polycoefs[i] <<"\t#"<<i<<"th order term\n";

 return polycoefs;
}

