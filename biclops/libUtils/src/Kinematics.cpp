#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#endif

#include "Kinematics.h"

const char *Kinematics::StartToken = "[kinematics]";
const char *Kinematics::EndToken = "[/kinematics]";
