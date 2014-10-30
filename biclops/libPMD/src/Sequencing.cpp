#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppress warnings about long identifiers
#endif

#include <set>      // needed to support coordinate validation structure
using std::set;     // (see SequenceAction constructor)
#include <string.h>  // for strcmp,strcpy

#include "Sequencing.h"
#include "PMDCollections.h"
#include "PMDUtils.h"
#include "Parser.h"

const char SequenceAction::TokenMoveStart[] = "[move]";
const char SequenceAction::TokenMoveEnd[] = "[/move]";
const char SequenceAction::TokenPause[] = "pause";
const char SequenceAction::TokenHome[] = "home";
const char SequenceAction::TokenRepeat[] = "repeat";
const char SequenceAction::TokenPark[] = "park";

//----------------------------------------------------------------------------
SequenceAction::
SequenceAction(unsigned int pauseDuration) {
    this->actionType = Pause;
    this->pauseDuration = pauseDuration;
}

//----------------------------------------------------------------------------
SequenceAction::
SequenceAction(PMDAxisControl *axis) {
    this->actionType = Home;
    this->axis = axis;
}

//----------------------------------------------------------------------------
SequenceAction::
SequenceAction(Sequence *repeatSequence, 
             unsigned int repeatCount) {
    this->actionType = Repeat;
    this->repeatSequence = repeatSequence;
    this->repeatCount = repeatCount;
}

//----------------------------------------------------------------------------
SequenceAction::
SequenceAction(FILE *file, char *token) {

    // Instruction type is Move.
    actionType = Move;

    // Load default values for case where values don't occur during parse.
    pauseDuration = 0;
    coordinated = false;
    blockUntilDone = false;

    const char TokenBlockUntilDone[] = "blockUntilDone";
    const char TokenCoordinated[] = "coordinated";
    const char TokenKinematics[] = "kinematics";

    kinematics = NULL;

    // Define stucture for validating world-space coordinate values.
    // (Populated only when a valid kinematics set is identified from the file)
    typedef set<string> StringSet;
    StringSet validCoords;

    // Get direct access to the needed collections
    AxisCollection *axes = PMDAxes::Instance();

    // Prime token stream.
    bool tokenRecognized = true;
    Parser::ReadToken(file,token);

    // Parse the stream.
    do {
        if (!strcmp(token,TokenBlockUntilDone)) {
            blockUntilDone = true;
        } else if (!strcmp(token,TokenCoordinated)) {
            tokenRecognized = 
                Parser::ReadBool(file,token,coordinated);
        } else if (!strcmp(token,TokenKinematics)) {
            Parser::ReadToken(file,token);
            KinematicSequenceCollection* kinematicSequences = 
                KinematicSequences::Instance();
            KinematicSequenceCollection::iterator kit = kinematicSequences->find(token);
            if (kit != kinematicSequences->end()) {
                kinematics = kit->second;
                //Populate the world-coordinate validation structure
                const char *coordList[] = {"X", "Y", "Z", "yaw", "pitch", "roll", "orbit", NULL};
                for (int i=0;coordList[i] != NULL;i++)
                    validCoords.insert(coordList[i]);
            } else
                tokenRecognized = false;

        } else if (kinematics != NULL) {
            // Must be reading world-space values.

            // Get dimension of world-space coordinate and verify it's authenticity
            StringSet::iterator it = validCoords.find(token);
            if (it != validCoords.end()) {
                double dval;
                if (it == validCoords.find("X") || 
                    it == validCoords.find("Y") || 
                    it == validCoords.find("Z"))
                    tokenRecognized = PMDUtils::ReadLength(file,token,dval);
                else
                    tokenRecognized = PMDUtils::ReadAngle(file,token,dval);
                if (tokenRecognized) coordinates[*it] = dval;
            } else
                tokenRecognized = false;

        } else {
            // Must be reading joint-space commands
            // Verify the authenticity of the axis by checking to see if it exists.
            AxisCollection::iterator axIt = axes->find(token);
            if (axIt != axes->end()) {

                // Store the goal position for this axis.
                double angle;
                char axisToken[40];
                strcpy(axisToken,token);
                tokenRecognized = axIt->second->ReadAngle(file,token,angle);
                if (tokenRecognized) coordinates[axisToken] = angle;

            } else
                tokenRecognized = false;
        }
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);
}

//----------------------------------------------------------------------------
SequenceAction::SequenceAction(PMDAxisControl *axis, bool parkIt) {
    actionType = Park;
    this->axis = axis;
    this->parkIt = parkIt;
}

//----------------------------------------------------------------------------
bool SequenceAction::
Execute() {
    bool status = true;

    // This has to be here instead of within the case statement because the 
    // compiler can't handle declaration of a variable within it.
    Coordinates::iterator coordIt;

    switch (actionType) {
    case Pause:
        PMDUtils::DoSleep(pauseDuration);
        return true;
        break;
    case Move:
        

        // Is this move in joint space or world space?
        if (kinematics == NULL) {
            // Move is in joint space.
            // Build axis list from coordinates and do move.
            AxisCollection *axisCollection = PMDAxes::Instance();
            PMDUtils::AxisList axisList;
            for (coordIt = coordinates.begin();coordIt != coordinates.end(); coordIt++) {
                // Assume axis in coordinates collection is valid.
                PMDAxisControl *axis = (*axisCollection)[coordIt->first];
                PMDAxisControl::Profile profile;
                axis->GetProfile(profile);
                profile.pos = coordIt->second;
                axis->SetProfile(profile);
                axisList.push_back(axis);
            }

            // Coordinate the axes if desired
            if (coordinated) PMDUtils::Coordinate(axisList);

            // Execute the moves.
            return PMDUtils::Move(axisList,blockUntilDone);

        } else {
            // Move is in world space.
            // Get world-space coordinates into a form consumable by the 
            // kinematics code
            Kinematics::DoublesVector worldVector(7,0.0);
            Kinematics::DoublesVector jointVector(7,0.0);
            for (coordIt = coordinates.begin();coordIt != coordinates.end(); coordIt++) {
                if (!strcmp(coordIt->first.c_str(),"X")) worldVector[0] = coordIt->second;
                else if (!strcmp(coordIt->first.c_str(),"Y"))     worldVector[1] = coordIt->second;
                else if (!strcmp(coordIt->first.c_str(),"Z"))     worldVector[2] = coordIt->second;
                else if (!strcmp(coordIt->first.c_str(),"yaw"))   worldVector[3] = coordIt->second;
                else if (!strcmp(coordIt->first.c_str(),"pitch")) worldVector[4] = coordIt->second;
                else if (!strcmp(coordIt->first.c_str(),"roll"))  worldVector[5] = coordIt->second;
                else if (!strcmp(coordIt->first.c_str(),"orbit")) worldVector[6] = coordIt->second;
            }

            // Perform the inverse kinematics
            if (kinematics->kinematics->Inverse(worldVector,jointVector)) {
                // Move the results to the axis
                int i = 0;
                PMDUtils::AxisList::iterator axisIt;
                for (axisIt = kinematics->axes.begin();axisIt != kinematics->axes.end(); axisIt++) {
                    PMDAxisControl *axis = *axisIt;
                    PMDAxisControl::Profile profile;
                    axis->GetProfile(profile);
                    profile.pos = jointVector[i];
                    axis->SetProfile(profile);
                    i++;
                }

                // Coordinate the axes if desired
                if (coordinated) PMDUtils::Coordinate(kinematics->axes);

                // Execute the moves.
                return PMDUtils::Move(kinematics->axes,blockUntilDone);
            }

        }

        break;
    case Repeat:
        if (repeatCount > 0) {
            for (unsigned int i = 0; i<repeatCount;i++) 
                status = status && repeatSequence->Execute();
            return status;
        } else {
            // Repeat forever or until an error occurs.
            while (status) {
                status = status && repeatSequence->Execute();
            }
            return status;
        }
        break;
    case Home:
        return axis->FindHome();
        break;
    case Park:
        if (parkIt) axis->Park();
        else axis->Unpark();
        return true;
        break;

    }
    return false;
}

const char Sequence::StartToken[] = "[sequence]";
const char Sequence::EndToken[] = "[/sequence]";

//----------------------------------------------------------------------------
Sequence::
Sequence(FILE *file, char *token) {

    bool tokenRecognized = true;
    unsigned int val;
    SequenceAction *action;

    // Get direct access to the needed collections
    AxisCollection *axes = PMDAxes::Instance();
    SequenceCollection *sequences = Sequences::Instance();

    // Prime token stream.
    Parser::ReadToken(file,token);

    // Parse the stream.
    do {
        if (!strcmp(token,PMDUtils::TokenUniqueID)) {
            Parser::ReadToken(file,token);
            strcpy(uniqueID,token);
        } else if (!strcmp(token,SequenceAction::TokenPause)) {
            // Get the pause duration and create the action.
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            action = new SequenceAction(val);
            sequence.push_back(action);
        } else if (!strcmp(token,SequenceAction::TokenHome)) {
            // Get the axis identifier, find it's 
            // reference and create the action.
            Parser::ReadToken(file,token);
            AxisCollection::iterator axIt = axes->find(token);
            if (axIt != axes->end()) {
                action = new SequenceAction(axIt->second);
                sequence.push_back(action);
            }
        } else if (!strcmp(token,SequenceAction::TokenRepeat)) {
            // Get the sequence identifier, find it's 
            // reference and create the action.
            Parser::ReadToken(file,token);
            Sequence *repeatSequence = (*sequences)[token];
            Parser::ReadToken(file,token);
            tokenRecognized = sscanf(token,"%u",&val) == 1;
            if (repeatSequence != NULL) {
                action = new SequenceAction(repeatSequence,val);
                sequence.push_back(action);
            }
        } else if (!strcmp(token,SequenceAction::TokenMoveStart)) {
            action = new SequenceAction(file,token);
            if (!strcmp(token,SequenceAction::TokenMoveEnd))
                sequence.push_back(action);
            else tokenRecognized = false;
        } else if (!strcmp(token,SequenceAction::TokenPark)) {
            // Get the axis identifier, find it's 
            // reference and create the action.
            Parser::ReadToken(file,token);
            PMDAxisControl *axis = (*axes)[token];
            if (axis != NULL) {
                bool parkIt;
                if (Parser::ReadBool(file,token,parkIt)) {
                    action = new SequenceAction(axis,parkIt);
                    sequence.push_back(action);
                }
            }
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);
}

//----------------------------------------------------------------------------
bool Sequence::
Execute() {
    bool lastActionExecutedSuccessfully = true;

    // Execute each action in order.
    SequenceList::iterator action;
    for (action = sequence.begin(); 
         action != sequence.end() && lastActionExecutedSuccessfully; 
         action++)
        lastActionExecutedSuccessfully = (*action)->Execute();
    
    return lastActionExecutedSuccessfully;
}

const char KinematicSequence::StartToken[] = "[kinematicSequence]";
const char KinematicSequence::EndToken[] = "[/kinematicSequence]";
const char KinematicSequence::TokenAxis[] = "axis";

//----------------------------------------------------------------------------
KinematicSequence::
KinematicSequence(FILE *file, char *token) {
    bool tokenRecognized = true;
    char uniqueID[30];

    // Get direct access to the needed collections
    AxisCollection *axisCollection = PMDAxes::Instance();

    // Prime token stream.
    Parser::ReadToken(file,token);

    // Parse the stream.
    do {
        if (!strcmp(token,PMDUtils::TokenUniqueID)) {
            Parser::ReadToken(file,token);
            strcpy(uniqueID,token);
            kinematics = (*KinematicSolutions::Instance())[token];
        } else if (!strcmp(token,TokenAxis)) {
            Parser::ReadToken(file,token);
            if (axisCollection->find(token) != axisCollection->end()) {
                axes.push_back((*axisCollection)[token]);
            }
        } else tokenRecognized = false;
        if (tokenRecognized) Parser::ReadToken(file,token);
    } while (!feof(file) && tokenRecognized);

    if (!strcmp(token,EndToken)){
        KinematicSequenceCollection *kinematicsCollection = 
            KinematicSequences::Instance();
        (*kinematicsCollection)[uniqueID] = this;
    }

}

