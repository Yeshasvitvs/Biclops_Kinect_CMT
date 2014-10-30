#define dbgCout cerr << classStr << methodStr 
#define ifDbg if (GetDebugLevel() > 0)
#define ifDbg1 if (GetDebugLevel() > 1)
#define ifDbgCout ifDbg dbgCout
#define ifDbgCout1 ifDbg1 dbgCout
