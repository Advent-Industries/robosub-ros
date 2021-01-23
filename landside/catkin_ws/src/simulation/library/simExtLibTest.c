#include "simExtLibTest.h"

//#include "stackArray.h"
//#include "stackMap.h"
//#include "simLib.h"

//extern "C" {
    #include "stack/stackArray.h"
    #include "stack/stackMap.h"
    #include "simLib.h"
//}

#include <iostream>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif

#if defined(__linux) || defined(__APPLE__)
    #include <unistd.h>
    #include <string.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define RHO 1000            // density of water
#define WATER_HEIGHT 0      // the z height of the water
#define QUADRATIC_DRAG 1    // true is drag should be quadratic, rather than linear
#define PI 3.14159

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_VERSION 5 // 2 since version 3.2.1, 3 since V3.3.1, 4 since V3.4.0, 5 since V3.4.1

static LIBRARY simLib; // the CoppelisSim library that we will dynamically load and bind

int calcBuoyancy(int handle, float* buoy) {
    // buoyancy = rho * V * g, where V is volume of object underwater
    // calculates V assuming the object fills the entire space of its bounding box
    float minsize[3];
    float maxsize[3];
    float objsize[3];

    for (int i = 0; i < 2; i++) {
        if (simGetObjectFloatParameter(handle, 15+i, minsize+i) != 0);// return -1;
        if (simGetObjectFloatParameter(handle, 18+i, maxsize+i) != 0);// return -1;
        objsize[i] = maxsize[i] - minsize[i];
    }

    float vol = objsize[0] * objsize[1];
    float zdepth; // the amount that the robot is below the water

    // this whole block can probably be condensed into a one liner
    // but I'm leaving it as is for now because I want to logic to be explicit, in the likely event
    // that we have to change it in the near future.
    if (minsize[2] > WATER_HEIGHT) {
        zdepth = 0;
    } else if (maxsize[2] < WATER_HEIGHT) {
        zdepth = objsize[2];
    } else {
        zdepth = WATER_HEIGHT - minsize[2];
    }

    vol *= zdepth;

    float grav_vector[3];
    simGetArrayParameter(sim_arrayparam_gravity, grav_vector);
    float grav = grav_vector[2];

    printf("rho %d, vol %f, grav %f\n", RHO, vol, grav);

    float buoyForce = RHO * vol * grav;

    buoy[0] = 0;
    buoy[1] = 0;
    buoy[2] = buoyForce;
}


// --------------------------------------------------------------------------------------
// simExtSkeleton_getData: an example of custom Lua command
// --------------------------------------------------------------------------------------
#define LUA_SIMPLEBUOY_COMMAND "simLibraryTest.simpleBuoy" // the name of the new Lua command

void LUA_SIMPLEBUOY_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;

    CStackArray inArguments;
    inArguments.buildFromStack(stack);
    simAddLog("Libtest", sim_verbosity_msgs, "in thing addlog");

    if ( (inArguments.getSize()>=2)&&inArguments.isString(0)&&inArguments.isNumber(1) )
    { // we expect at least 2 arguments: a string and a map

        std::string tmp("we received a string (");
        tmp+=inArguments.getString(0).c_str();
        tmp+=") and also the handle (";
        tmp+=inArguments.getInt(1);
        tmp+=")";
        printf("args: %s, %d\n", inArguments.getString(0).c_str(), inArguments.getInt(1));

        int handle = inArguments.getInt(1);

        simAddLog("Library TEst",sim_verbosity_msgs,tmp.c_str());
        float center[3] = {0,0,.5};
        float force[3] = {0,0,.5};
        calcBuoyancy(handle, force);
        //force[2] = 12;
        printf("after calc buoy\n");
        printf("buoy force: %f, %f, %f\n", force[0], force[1], force[2]);
        simAddForce(handle, center, force);
        printf("after add force\n");
    }
    else
        simSetLastError(LUA_SIMPLEBUOY_COMMAND,"Not enough arguments or wrong arguments.");

    // Now return a string and a map:
    CStackArray outArguments;
    outArguments.pushString("Hello World");
    CStackMap* map=new CStackMap();
    map->setBool("operational",true);
    CStackArray* pos=new CStackArray();
    double _pos[3]={0.0,1.0,2.0};
    pos->setDoubleArray(_pos,3);
    map->setArray("position",pos);
    outArguments.pushMap(map);
    outArguments.buildOntoStack(stack);
    printf("end\n");
}
// --------------------------------------------------------------------------------------


// This is the plugin start routine (called just once, just after the plugin was loaded):
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
    // Dynamically load and bind CoppelisSim functions:
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#else
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the CoppelisSim library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\coppeliaSim.dll";
#elif defined (__linux)
    temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
    temp+="/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the CoppelisSim library:
    simLib=loadSimLibrary(temp.c_str());
    if (simLib==NULL)
    {
        printf("simExtPluginSkeleton: error: could not find or correctly load the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
        return(0); // Means error, CoppelisSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        printf("simExtPluginSkeleton: error: could not find all required functions in the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppelisSim will unload this plugin
    }

    // Check the version of CoppelisSim:
    int simVer,simRev;
    simGetIntegerParameter(sim_intparam_program_version,&simVer);
    simGetIntegerParameter(sim_intparam_program_revision,&simRev);
    if( (simVer<40000) || ((simVer==40000)&&(simRev<1)) )
    {
        simAddLog("PluginSkeleton",sim_verbosity_errors,"sorry, your CoppelisSim copy is somewhat old, CoppelisSim 4.0.0 rev1 or higher is required. Cannot start the plugin.");
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppelisSim will unload this plugin
    }

    // Implicitely include the script lua/simExtPluginSkeleton.lua:
    simRegisterScriptVariable("simSkeleton","require('simExtLibTest')",0);

    // Register the new function:
    simRegisterScriptCallbackFunction(strConCat(LUA_SIMPLEBUOY_COMMAND,"@","PluginSkeleton"),strConCat("...=",LUA_SIMPLEBUOY_COMMAND,"(string data1, number data2)"),LUA_SIMPLEBUOY_CALLBACK);

    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when CoppelisSim is ending, i.e. releasing this plugin):
SIM_DLLEXPORT void simEnd()
{
    // Here you could handle various clean-up tasks

    unloadSimLibrary(simLib); // release the library
}

// This is the plugin messaging routine (i.e. CoppelisSim calls this function very often, with various messages):
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=NULL;

    // Here we can intercept many messages from CoppelisSim (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the CoppelisSim user manual.

    if (message==sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag=true; // CoppelisSim dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message==sim_message_eventcallback_menuitemselected)
    { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message==sim_message_eventcallback_instancepass)
    {   // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in CoppelisSim. This message is the most convenient way to do so:

        int flags=auxiliaryData[0];
        bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
        bool instanceSwitched=((flags&64)!=0);

        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }

        if (sceneContentChanged)
        { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message==sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))

    }

    if (message==sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start

    }

    if (message==sim_message_eventcallback_simulationended)
    { // Simulation just ended

    }

    if (message==sim_message_eventcallback_moduleopen)
    { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message==sim_message_eventcallback_modulehandle)
    { // A script called simHandleModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message==sim_message_eventcallback_moduleclose)
    { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message==sim_message_eventcallback_instanceswitch)
    { // We switched to a different scene. Such a switch can only happen while simulation is not running

    }

    if (message==sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message==sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag=false;
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}