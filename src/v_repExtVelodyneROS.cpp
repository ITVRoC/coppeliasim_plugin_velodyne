#include "../include/coppeliasim_plugin_velodyne/v_repExtVelodyneROS.h"
#include "../include/scriptFunctionData.h"
#include "../include/v_repLib.h"
#include <iostream>
#include "../include/coppeliasim_plugin_velodyne/velodyneROSModelCont.h"

#include "../include/coppeliasim_plugin_velodyne/ros_server_velodyne.h"
#include <iostream>

#ifdef _WIN32
#ifdef QT_COMPIL
#include <direct.h>
#else
#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)

#include <unistd.h>

#endif /* __linux || __APPLE__ */

#define CONCAT(x, y, z) x y z
#define strConCat(x, y, z)    CONCAT(x,y,z)

LIBRARY vrepLib;

CVelodyneROSModelCont *velodyneROSModelContainer;


// --------------------------------------------------------------------------------------
// simExtVelodyneROS_createVelodyneROSModel
// --------------------------------------------------------------------------------------
#define LUA_CREATEVELODYNEROSMODEL_COMMAND_PLUGIN "simExtVelodyneROS_createVelodyneROSModel@VelodyneROS"
#define LUA_CREATEVELODYNEROSMODEL_COMMAND "simExtVelodyneROS_createVelodyneROSModel"

const int inArgs_CREATEVELODYNEROSMODEL[] = {
        9,
        sim_script_arg_int32 | sim_script_arg_table, 4,
        sim_script_arg_float, 0,
        sim_script_arg_int32, 0,
        sim_script_arg_float, 0,
        sim_script_arg_float | sim_script_arg_table, 2,
        sim_script_arg_float, 0,
        sim_script_arg_int32, 0,
        sim_script_arg_string, 0,
        sim_script_arg_string, 0,
};

void LUA_CREATEVELODYNEROSMODEL_CALLBACK(SScriptCallBack *p)
{
    CScriptFunctionData D;
    int velodyneHandle = -1;
    if (D.readDataFromStack(p->stackID, inArgs_CREATEVELODYNEROSMODEL, inArgs_CREATEVELODYNEROSMODEL[0] - 5,
                            LUA_CREATEVELODYNEROSMODEL_COMMAND)) // -5 because the last 5 arguments are optional
    {
        std::vector<CScriptFunctionDataItem> *inData = D.getInDataPtr();
        int options = 0;
        float pointSize = 2.0f;
        float scalingFactor = 1.0f;
        float coloringDistances[2] = {1, 5};
        int visionSensorHandes[4];
        std::string topic = "/velodyne";
        std::string frameId = "lidar";
        for (int i = 0; i < 4; i++)
            visionSensorHandes[i] = inData->at(0).int32Data[i];
        float frequency = inData->at(1).floatData[0];
        int LocalFrameHandle;
        if (inData->size() > 2)
        { // we have the optional 'options' argument:
            options = inData->at(2).int32Data[0];
        }
        if (inData->size() > 3)
        { // we have the optional 'pointSize' argument:
            pointSize = inData->at(3).floatData[0];
        }
        if (inData->size() > 4)
        { // we have the optional 'coloringDistance' argument:
            coloringDistances[0] = inData->at(4).floatData[0];
            coloringDistances[1] = inData->at(4).floatData[1];
        }
        if (inData->size() > 5)
        { // we have the optional 'displayScalingFactor' argument:
            scalingFactor = inData->at(5).floatData[0];
        }
        if (inData->size() > 6)
        {
            LocalFrameHandle = inData->at(6).int32Data[0];
        }
        if (inData->size() > 7)
        {
            frameId = inData->at(7).stringData[0];
        }
        if (inData->size() > 8)
        {
            topic = inData->at(8).stringData[0];
        }
        CVelodyneROSModel *obj = new CVelodyneROSModel(visionSensorHandes, frequency, options, pointSize,
                                                       coloringDistances, scalingFactor, LocalFrameHandle, frameId,
                                                       topic);
        velodyneROSModelContainer->addObject(obj);
        if (obj->doAllObjectsExistAndAreVisionSensors())
        {
            if (obj->areVisionSensorsExplicitelyHandled())
                velodyneHandle = obj->getVelodyneHandle(); // success
            else
                simSetLastError(LUA_CREATEVELODYNEROSMODEL_COMMAND,
                                "Vision sensors should be explicitely handled."); // output an error
        } else
            simSetLastError(LUA_CREATEVELODYNEROSMODEL_COMMAND,
                            "Invalid handles, or handles are not vision sensor handles."); // output an error

        if (velodyneHandle == -1)
            velodyneROSModelContainer->removeObject(obj->getVelodyneHandle());
    }
    D.pushOutData(CScriptFunctionDataItem(velodyneHandle));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVelodyneROS_destroyVelodyneROSModel
// --------------------------------------------------------------------------------------
#define LUA_DESTROYVELODYNEROSMODEL_COMMAND_PLUGIN "simExtVelodyneROS_destroyVelodyneROSModel@VelodyneROS"
#define LUA_DESTROYVELODYNEROSMODEL_COMMAND "simExtVelodyneROS_destroyVelodyneROSModel"

const int inArgs_DESTROYVELODYNEROSMODEL[] = {
        1,
        sim_script_arg_int32, 0,
};

void LUA_DESTROYVELODYNEROSMODEL_CALLBACK(SScriptCallBack *p)
{
    CScriptFunctionData D;
    int result = -1;
    if (D.readDataFromStack(p->stackID, inArgs_DESTROYVELODYNEROSMODEL, inArgs_DESTROYVELODYNEROSMODEL[0],
                            LUA_DESTROYVELODYNEROSMODEL_COMMAND))
    {
        std::vector<CScriptFunctionDataItem> *inData = D.getInDataPtr();
        int handle = inData->at(0).int32Data[0];
        CVelodyneROSModel *obj = velodyneROSModelContainer->getObject(handle);
        if (obj != NULL)
        {
            velodyneROSModelContainer->removeObject(obj->getVelodyneHandle());
            result = 1;
        } else
            simSetLastError(LUA_DESTROYVELODYNEROSMODEL_COMMAND, "Invalid handle."); // output an error
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVelodyne_handleVelodyneROSModel
// --------------------------------------------------------------------------------------
#define LUA_HANDLEVELODYNEROSMODEL_COMMAND_PLUGIN "simExtVelodyneROS_handleVelodyneROSModel@VelodyneROS"
#define LUA_HANDLEVELODYNEROSMODEL_COMMAND "simExtVelodyneROS_handleVelodyneROSModel"

const int inArgs_HANDLEVELODYNEROSMODEL[] = {
        2,
        sim_script_arg_int32, 0, //Velodyne Handle
        sim_script_arg_float, 0, //dt
};

void LUA_HANDLEVELODYNEROSMODEL_CALLBACK(SScriptCallBack *p)
{
    CScriptFunctionData D;

    bool result = false;
    if (D.readDataFromStack(p->stackID, inArgs_HANDLEVELODYNEROSMODEL, inArgs_HANDLEVELODYNEROSMODEL[0],
                            LUA_HANDLEVELODYNEROSMODEL_COMMAND))
    {
        std::vector<CScriptFunctionDataItem> *inData = D.getInDataPtr();
        int handle = inData->at(0).int32Data[0];
        float dt = inData->at(1).floatData[0];
        CVelodyneROSModel *obj = velodyneROSModelContainer->getObject(handle);
        if (obj != NULL)
        {
            result = obj->handle(dt);
        } else
            simSetLastError(LUA_HANDLEVELODYNEROSMODEL_COMMAND, "Invalid handle."); // output an error
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// This is the plugin start routine:
VREP_DLLEXPORT unsigned char v_repStart(void *reservedPointer, int reservedInt)
{ // This is called just once, at the start of V-REP
    // Dynamically load and bind V-REP functions:
    char curDirAndFile[1024];
#ifdef _WIN32
#ifdef QT_COMPIL
    _getcwd(curDirAndFile, sizeof(curDirAndFile));
#else
    GetModuleFileName(NULL,curDirAndFile,1023);
    PathRemoveFileSpec(curDirAndFile);
#endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

#ifdef _WIN32
    temp+="\\v_rep.dll";
#elif defined (__linux)
    temp += "/libv_rep.so";
#elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */

    vrepLib = loadVrepLibrary(temp.c_str());
    if (vrepLib == NULL)
    {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'VelodyneROS' plugin.\n";
        return (0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib) == 0)
    {
        std::cout
                << "Error, could not find all required functions in the V-REP library. Cannot start 'VelodyneROS' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }

    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if (vrepVer < 30200) // if V-REP version is smaller than 3.02.00
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'VelodyneROS' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }

    // Initialize the ROS part:
    if (!ROS_server::initialize())
    {
        std::cout << "ROS master is not running. Cannot start 'rosVelodyne' plugin.\n";
        return (0); //If the master is not running then the plugin is not loaded.
    }

    // Register the new Lua commands:

    // modified for locally publishing pointcloud
    simRegisterScriptCallbackFunction(LUA_CREATEVELODYNEROSMODEL_COMMAND_PLUGIN,
                                      strConCat("number velodyneHandle=", LUA_CREATEVELODYNEROSMODEL_COMMAND,
                                                "(table_4 visionSensorHandles,number frequency,number options=0,number pointSize=2,table_2 coloring_closeFarDist={1,5},number displayScalingFactor=1, number LocalFrameHandle)"),
                                      LUA_CREATEVELODYNEROSMODEL_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_DESTROYVELODYNEROSMODEL_COMMAND_PLUGIN,
                                      strConCat("number result=", LUA_DESTROYVELODYNEROSMODEL_COMMAND,
                                                "(number velodyneHandle)"), LUA_DESTROYVELODYNEROSMODEL_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_HANDLEVELODYNEROSMODEL_COMMAND_PLUGIN,
                                      strConCat("table points=", LUA_HANDLEVELODYNEROSMODEL_COMMAND,
                                                "(number velodyneHandle,number dt)"),
                                      LUA_HANDLEVELODYNEROSMODEL_CALLBACK);

    velodyneROSModelContainer = new CVelodyneROSModelCont();

    return (3);    // initialization went fine, we return the version number of this extension module (can be queried with simGetModuleName)
    // Version 2 since 3.2.1
    // Version 3 since 3.3.1
}

// This is the plugin end routine:
VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP

    delete velodyneROSModelContainer;

    ROS_server::shutDown();    // shutdown the ROS_server

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void *v_repMessage(int message, int *auxiliaryData, void *customData, int *replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle

    // This function should not generate any error messages:
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);

    void *retVal = NULL;

    if (message == sim_message_eventcallback_simulationended)
    { // Simulation just ended
        velodyneROSModelContainer->removeAll();
    }

    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    return (retVal);
}

