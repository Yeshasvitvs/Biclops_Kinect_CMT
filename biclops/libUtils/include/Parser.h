//////////////////////////////////////////////////////////////////////////
// Copyright © 2011 by Bryn Wolfe and TRACLabs Inc.                     //
// All rights reserved. No part of this software or source code may be  //
// used or reproduced in any form or by any means, or stored in a       //
// database or retrieval system, without prior written aprroval from    //
// TRACLabs. This notice must not be removed or modified under penalty  //
// of United States copyright law.                                      //
//////////////////////////////////////////////////////////////////////////
#if !defined Parser_h
#define Parser_h

#include <stdio.h>          // Defines FILE type

class Parser
{
public:
    // File parsing utilities
    // A '#' character starts an end-of-line comment.
    // Returns true if a valid token is returned, false otherwise.
    static bool ReadToken (FILE *file, char *token, bool returnComments = false);
    static bool ReadBool  (FILE *file, char *token, bool &value);
    static bool FindToken (FILE *file, char *token, const char *find);

};

#endif