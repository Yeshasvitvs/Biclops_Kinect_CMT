#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>

#define SEGMENTS 9
#define SLICES   1

#define RAD_TO_DEG      57.29577951308232
#define DEG_TO_RAD       0.01745329251994

#define SEED 17

#define GRAVPOS 9.801
#define GRAVPOS2 4.9005

#if defined _MSC_VER
#define M_PI (3.1415926535897932384626433832795)
#endif
#define M_2PI (2.0*M_PI)
#define M_PI2 (0.5*M_PI)


#define RANDUNITFLOAT (((rand()%20000) - 10000)*0.0001)

#ifndef fori
#define fori(x) for(i=0;i<(x);i++)
#define forj(x) for(j=0;j<(x);j++)
#define forii(x) for(int i=0;i<(x);i++)
#define forij(x) for(int j=0;j<(x);j++)
#define forik(x) for(int k=0;k<(x);k++)
#define fori1(x) for(i=1;i<(x);i++)
#define forj1(x) for(j=1;j<(x);j++)
#define fork0(x) for(k=0;k<(x);k++)
#define fork1(x) for(k=1;k<(x);k++)
#endif

#endif






