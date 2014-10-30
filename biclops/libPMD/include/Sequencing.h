#if !defined Sequencing_h
#define Sequencing_h

#include "PMDAxisControl.h"
#include "PMDUtils.h"           // for AxisList type
#include "singleton.h"      // to enforce single instance of each collection

#include <list>
using std::list;
#include <map>
using std::map;
#include <string>           // key data type for collection
using std::string;

class Sequence;
class KinematicSequence;

class SequenceAction {
public:

    // Define start/stop tokens for configuration file parsing.
    static const char TokenMoveStart[];
    static const char TokenMoveEnd[];
    static const char TokenPause[];
    static const char TokenHome[];
    static const char TokenRepeat[];
    static const char TokenPark[];

    // Possible instructions
    enum ActionType {Pause,Move,Home,Repeat,Park};

    // Pause action constructor.
    SequenceAction(unsigned int pauseDuration = 0);
    // Home action constructor.
    SequenceAction(PMDAxisControl *axis);
    // Repeat action constructor.
    SequenceAction(Sequence *repeatSequence, 
                 unsigned int repeatCount = 1);
    // Move action constructor (it's too complicated to do with parameters).
    SequenceAction(FILE *file, char *token);
//    SequenceAction(MoveSet *axes, 
//                 unsigned int period = 0, bool coordinated = false);
    // Park action constructor.
    SequenceAction(PMDAxisControl *axis, bool parkIt);

    // Execute the action represented herein.
    bool Execute();

private:

    // Data type associated with the Move instruction that represents the 
    // collection of axes to move and their desired movement.

    // Instruction type to be executed for this sequence step.
    ActionType actionType;

    // Indicates duration of pause instruction in milliseconds
    unsigned int pauseDuration;

    // True when an instruction (e.g., a move) is to finish before continuing.
    bool blockUntilDone;

    // When action Park, tells whether joint is to be parked (true) or unparked.
    bool parkIt;

    PMDAxisControl *axis;

    // Attributes specific to a move instruction.
    bool coordinated;       // Indicates if the axes are to be coordinated.
    KinematicSequence *kinematics; // Points to kinematic sol'n when move goal is in
                            // not in joint space.
    typedef map<string,double> Coordinates;
    Coordinates coordinates;// Move destinations stored as key-value pairs
//    typedef list<PMDint32> MoveGoals;
//    MoveGoals moveGoals;// Positions to which each axis is directed.
//    PMDUtils::AxisList moveAxes;
                        // Collection of axes to be moved or homed 
                        // (for homing, there should only be one axis).

    // Parameters specific to the repeat instruction.
    Sequence *repeatSequence;    // Sequence to be repeated
    unsigned int repeatCount;       // # of times to repeat. A value of zero
                                    // indicates "repeat indefinitely."

};

class Sequence {
public:

    // Define start/stop tokens for configuration file parsing.
    static const char StartToken[];
    static const char EndToken[];

    char uniqueID[50];

    Sequence(FILE *file, char *token); // Constructor using configuration file.

    bool Execute();

private:

    // A collection of ordered steps in a sequence.
    typedef list<SequenceAction*> SequenceList;
    SequenceList sequence;

};

// Structure to associate a kinematics instance with the corresponding axes.
class KinematicSequence {
public:
    static const char StartToken[];
    static const char EndToken[];
    KinematicSequence(FILE *file, char *token);
    Kinematics *kinematics;
//    vector<PMDAxisControl*> axes;
    PMDUtils::AxisList axes;
private:
    static const char TokenAxis[];
};

// A collection of the defined kinematic models available for sequencing.
typedef map<string,KinematicSequence*> KinematicSequenceCollection;
class KinematicSequences: public CSingleton<KinematicSequenceCollection> {
    friend class CSingleton<KinematicSequenceCollection>;
 
private:
    KinematicSequences();
    ~KinematicSequences();
};

#endif

