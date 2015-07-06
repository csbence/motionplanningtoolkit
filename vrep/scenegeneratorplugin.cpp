#include <iostream>
#include <unistd.h>
#include "v_repLib.h"
#include "luaFunctionData.h"

#include "vrepinterface.hpp"
#include "plannerfunctions.hpp"
#include "../utilities/instancefilemap.hpp"
#include "../utilities/datafile.hpp"

#include "positionsampler.hpp"

#include <boost/thread/thread.hpp>

#define VREP_DLLEXPORT extern "C"

bool pluginActive = true;

LIBRARY vrepLib;
VREPInterface *interface;
InstanceFileMap *args;
simFloat start = -1, dt = 1;

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt) {
	char curDirAndFile[1024];
	getcwd(curDirAndFile, sizeof(curDirAndFile));
	std::string currentDirAndPath(curDirAndFile);
	std::string temp(currentDirAndPath);

#if defined (__linux)
	temp+="/libv_rep.so";
#elif defined (__APPLE__)
	temp+="/libv_rep.dylib";
#endif

	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL) {
		fprintf(stderr, "Error, could not find or correctly load v_rep.dll. Cannot start 'BubbleRob' plugin.\n");
		return 0;
	}
	if (getVrepProcAddresses(vrepLib)==0) {
		fprintf(stderr, "Error, could not find all required functions in v_rep.dll. Cannot start 'BubbleRob' plugin.\n");
		unloadVrepLibrary(vrepLib);
		return 0;
	}

	// Check the V-REP version:
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<30200) {
		fprintf(stderr, "Sorry, your V-REP copy is somewhat old, V-REP 3.2.0 or higher is required. Cannot start 'BubbleRob' plugin.\n");
		unloadVrepLibrary(vrepLib);
		return 0;
	}

	simChar* instanceFile = simGetStringParameter(sim_stringparam_app_arg1);

	if(instanceFile == NULL || strlen(instanceFile) == 0) {
		fprintf(stderr, "...libv_repExtskiesel.dylib not running\n");
		pluginActive = false;
		return 1;
	}

	args = new InstanceFileMap(instanceFile);

	interface = new VREPInterface(*args);

	simStartSimulation();

	boost::thread workerThread([&](){
		dfheader(stdout);

		VREPInterface::State start;
		interface->makeStartState(start);

		simInt goalHandle = simGetObjectHandle("Goal");
		VREPInterface::State goal;

		simFloat vals[3];
		simGetObjectPosition(goalHandle, -1, vals);
		for(unsigned int i = 0; i < 3; ++i) {
			goal.rootPosition.push_back(vals[i]);
			goal.goalPositionVars.push_back(vals[i]);
		}

		simGetObjectOrientation(goalHandle, -1, vals);
		for(unsigned int i = 0; i < 3; ++i) {
			goal.rootOrientation.push_back(vals[i]);
			goal.goalOrientationVars.push_back(vals[i]);
		}

		PositionSampler<VREPInterface, VREPInterface> positionSampler(interface, interface);

		auto safeState = positionSampler.generateSafeState(start);

//		if(args->value("Planner").compare("RRT") == 0) {
//			solveWithRRT(interface, args, start, goal);
//		} else if(args->value("Planner").compare("RRT Connect") == 0) {
//			solveWithRRTConnect(interface, args, start, goal);
//		} else if(args->value("Planner").compare("Plaku IROS 2014") == 0) {
//			solveWithPlaku(interface, args, start, goal);
//		} else {
//			fprintf(stderr, "unrecognized planner!");
//			exit(1);
//		}
//
		dffooter(stdout);
	});

	return 1;
}

VREP_DLLEXPORT void v_repEnd() {
	// This is called just once, at the end of V-REP
	unloadVrepLibrary(vrepLib);
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData) {
	// This is called quite often. Just watch out for messages/events you want to handle
	// This function should not generate any error messages:
	void* retVal=NULL;

	if(pluginActive) {
		if(start < 0) {
			start = simGetSimulationTime();
			dt = interface->simulatorReady();
		}

		simFloat curDT = simGetSimulationTime() - start;
		bool collision = interface->collision();
		if(collision || dt <= curDT) {
			interface->simulatorDone(curDT, collision);
			start = -1;
		}
	}

	return retVal;
}