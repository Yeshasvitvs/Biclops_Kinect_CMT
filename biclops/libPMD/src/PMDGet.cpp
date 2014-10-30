#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#endif

#include "PMDGet.h"

AxisCollection* PMDGet::GetAxes() {
  return PMDAxes::Instance();
}
