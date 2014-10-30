//////////////////////////////////////////////////////////////////////////
// Copyright © 2011 by Bryn Wolfe and TRACLabs Inc.                     //
// All rights reserved. No part of this software or source code may be  //
// used or reproduced in any form or by any means, or stored in a       //
// database or retrieval system, without prior written aprroval from    //
// TRACLabs. This notice must not be removed or modified under penalty  //
// of United States copyright law.                                      //
//////////////////////////////////////////////////////////////////////////

#include "Parser.h"     // This class
#include "ctype.h"      // for isspace, strcpy, and strcmp
#include "string.h"     // strcpy, and strcmp

//----------------------------------------------------------------------------
bool Parser::ReadToken(FILE *file, char *token, bool returnComments)
{
    bool gotToken = false;
    const int BUFSIZE = 200;
    char buf[BUFSIZE];
    int ch;
    if (file != NULL) {
        while (gotToken == false && !feof(file)) {
            ch = fgetc(file);
            if (ch == '#') {
                // Consume to the end of the line
                if (returnComments) {
                    if(fgets(token,BUFSIZE-1,file) == NULL) {
                        // TODO: some error processing here
                    }
                } else {
                    if(fgets(buf,BUFSIZE-1,file) == NULL) {
                        // TODO: some error processing here
                    }
                }
                // This should make sure we actually hit the end of line,
                // but I don't have time to figure it out right now (BTW).
            } else if (isspace(ch)) {
                // Skip white space
                if(fscanf(file," ") == EOF) {
                    // TODO: some error processing here
                }
            } else if (ch == '[') {
                // Consume bracketed parameter
                char* bptr = buf;
                do {
                    *bptr++ = (char)ch;
                    ch = fgetc(file);
                } while (ch != ']' && !feof(file));
                *bptr++ = ']';
                *bptr = '\0';
                    gotToken = true;
            } else {
                // Have a real token to get
                char* bptr = buf;
                do {
                    *bptr++ = (char)ch;
                    ch = fgetc(file);
                } while (!isspace(ch) && !feof(file));
                *bptr = '\0';
                gotToken = true;
            }
        }
    }

    if (gotToken) strcpy(token,buf);
    return gotToken;
}

//----------------------------------------------------------------------------
bool Parser::ReadBool(FILE *file, char *token, bool &value) {
    const char TokenTrue[] = "true";
    const char TokenFalse[] = "false";
    bool tokenRecognized = true;
    ReadToken(file,token);
    if (strcmp(token,TokenTrue) == 0) 
        value = true;
    else if (strcmp(token,TokenFalse) == 0) 
        value = false;
    else tokenRecognized = false;
    return tokenRecognized;
}

//-----------------------------------------------------------------------------
bool Parser::FindToken(FILE *file, char *token, const char *find) {
    do {
        ReadToken(file,token);
    } while(strcmp(token,find) != 0 && !feof(file));
    return strcmp(token,find) == 0;
}
