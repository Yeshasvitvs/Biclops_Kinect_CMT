//////////////////////////////////////////////////////////////////////////
// Copyright © 2011 by Bryn Wolfe and TRACLabs Inc.                     //
// All rights reserved. No part of this software or source code may be  //
// used or reproduced in any form or by any means, or stored in a       //
// database or retrieval system, without prior written aprroval from    //
// TRACLabs. This notice must not be removed or modified under penalty  //
// of United States copyright law.                                      //
//////////////////////////////////////////////////////////////////////////

#if (defined WIN32 && !defined __MINGW32__)
#pragma warning(disable: 4786) // suppresses warning about long identifiers
#endif

#include <stdlib.h> // for atoi on Linux
#include <string.h>  // for strcpy, strcmp
#include "PMDNetwork.h"     // This class
#include "PMDUtils.h"       // For TokenUniqueID def
#include "PMDController.h"
#include "Parser.h"         // for parsing config files
#include "PMDCollections.h" // for collections of controllers and axes
#include "DebugMacros.h"

// Populate constants defined by this class
const char* PMDNetwork::classStr = "PMDNetwork";
const char PMDNetwork::StartToken[] = "[network]";
const char PMDNetwork::StopToken[] = "[/network]";
//const char PMDNetwork::StartPMDNetToken[] = "[PMDNetwork]";
//const char PMDNetwork::StopPMDNetToken[] = "[/PMDNetwork]";

//#define dbgCout cout << classStr << methodStr 

//----------------------------------------------------------------------------
PMDNetwork::PMDNetwork(const char *uniqueID, PortableSerial *serial) {
    strcpy(this->uniqueID,uniqueID);
    this->serial = serial;
    controllers = new ControllerCollection;
    compasses = new CompassCollection;
}

// ------------------------------------------------------------------------
PMDNetwork::PMDNetwork(FILE *file, char *token) {

    controllers = new ControllerCollection;
    compasses = new CompassCollection;

    // Load configuration from the file.
    ReadConfig(file,token);

}

//----------------------------------------------------------------------------
PMDNetwork::~PMDNetwork() {

}

//----------------------------------------------------------------------------
bool PMDNetwork::AddController(char* str, PMDController* controller) {
    (*controllers)[str] = controller;
    return ((*controllers)[str] != NULL);
}

//----------------------------------------------------------------------------
bool PMDNetwork::AddCompass(char *str, Compass* compass) {
    (*compasses)[str] = compass;
    return ((*compasses)[str] != NULL);
}

//----------------------------------------------------------------------------
bool PMDNetwork::OnlyOneDeviceIsOnNetwork() {
    return ((controllers->size() + compasses->size()) == 1);
}

// ------------------------------------------------------------------------
void PMDNetwork::ReadConfig(FILE *file, char *token) {

    // Valid token definitions
    const char TokenProtocol[] = "pmdProtocol";
    const char TokenAxis[] = "axis";

    // Local declarations
    bool validToken = true;
    tagPMDSerialProtocol pmdProtocol = PMDSerialProtocolPoint2Point;

    // Prime the token stream.
    Parser::ReadToken(file,token);

    // Parse the stream.
    do {
        if (!strcmp(token,PMDUtils::TokenUniqueID)) {
            Parser::ReadToken(file,token);
            strcpy(uniqueID,token);
        } else if (!strcmp(token,PortableSerial::StartToken)) {
            serial = new PortableSerial(file,token);
            validToken = !strcmp(token,PortableSerial::StopToken);
        } else if (!strcmp(token,TokenProtocol)) {
            Parser::ReadToken(file,token);
            validToken = PMDSerial::TokenToPMDProtocol(token, pmdProtocol);
        } else if (!strcmp(token,TokenAxis)) {
            int axisNum = 0;
            int contNum = 0;
            char appName[50], axisName[50], contName[50];
            Parser::ReadToken(file,token); axisNum = atoi(token);
            Parser::ReadToken(file,token); contNum = atoi(token);
            Parser::ReadToken(file,token); strcpy(appName,token);
            Parser::ReadToken(file,token); strcpy(axisName,token);
            // Could read debugLevel for an axis, where a -1 means "use network debug level"

            // If the controller doesn't exist, create it.
            PMDController::GenerateUniqueID(contName,uniqueID,contNum);
            ControllerCollection *globalCtlrs = PMDControllers::Instance();
            PMDController *controller = (*globalCtlrs)[contName];
            if (controller == NULL)
            {   PMDSerial *pmdSerial = new PMDSerial(this,pmdProtocol,contNum);
                controller = new PMDController(pmdSerial,contName);
                controller->SetDebugLevel(GetDebugLevel());
                (*globalCtlrs)[contName] = controller;  // add to global collection
                (*controllers)[contName] = controller;  // add to local collection
            }

            // If the axis doesn't exist, create it.
            AxisCollection *axes = PMDAxes::Instance();
	    
	    /* USED TO USE axisName, but now use appName for collections */
            PMDAxisControl *axis = (*axes)[appName];
            if (axis == NULL)
            {   
     	        axis = new PMDAxisControl(axisName,controller,axisNum);
                axis->SetDebugLevel(GetDebugLevel());
                controller->AddAxis(axis);  // Controller axis list entry

		/* USED TO USE BOTH axisName and appName, but now use
		   only appName for collections */
		//(*axes)[axisName] = axis;   // Global axis list entry under UID name
                (*axes)[appName] = axis;   // Global axis list entry under app name
            }
        } else validToken = false;
        if (validToken) Parser::ReadToken(file,token);
    } while (!feof(file) && validToken);

}

// ------------------------------------------------------------------------
int PMDNetwork::GetDebugLevel()
{    if (serial != NULL) return serial->GetDebugLevel();
    else return 0;
}

// ------------------------------------------------------------------------
void PMDNetwork::SetDebugLevel(int level) 
{   if (serial != NULL) serial->SetDebugLevel(level);
}
