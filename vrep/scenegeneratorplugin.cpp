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

namespace {
	const char *const pluginName = "SceneGenerator";
	PositionSampler<VREPInterface, VREPInterface> *positionSampler;

	void generateState() {
		VREPInterface::State start;
		interface->makeStartState(start);

		fprintf(stderr, "DEBUG:: Create sampler instance... \n");

		fprintf(stderr, "DEBUG:: Generate safe state... \n");
		auto safeState = positionSampler->generateSafeState(*interface, start);
		if (safeState) {
				fprintf(stderr, "DEBUG:: Safe state generation was successful \n");
				auto state = safeState.get();

				state.print();

			} else {
				fprintf(stderr, "DEBUG:: Safe state generation is failed \n");

			}
	}
}


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

	fprintf(stderr, "DEBUG:: Star simulation... \n");

	simStartSimulation();

	fprintf(stderr, "DEBUG:: Simulation started... \n");

	boost::thread workerThread([&](){
		dfheader(stdout);

//		generateState();

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

//	if(pluginActive) {
//		if(start < 0) {
//			start = simGetSimulationTime();
//			fprintf(stderr, "DEBUG:: Before:: Simulation ready \n");
//			dt = interface->simulatorReady();
//			fprintf(stderr, "DEBUG:: After:: Simulation ready \n");
//		}
//
//		simFloat curDT = simGetSimulationTime() - start;
//		bool collision = interface->collision();
//		if(collision || dt <= curDT) {
//			interface->simulatorDone(curDT, collision);
//			start = -1;
//		}
//	}

	// A script called simOpenModule (by default the main script). Is only called during simulation.
	if (message == sim_message_eventcallback_moduleopen) {
		// is the command also meant for this plugin?
		if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
			// we arrive here only at the beginning of a simulation
			std::cerr << "Plugin::SceneGenerator:: Module open" << std::endl;

			// TODO
			positionSampler = new PositionSampler<VREPInterface, VREPInterface>(interface);

		}
	}

	// A script called simHandleModule (by default the main script). Is only called during simulation.
	if (message == sim_message_eventcallback_modulehandle) {
		// is the command also meant for this plugin?
		if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
			// we arrive here only while a simulation is running
			std::cerr << "Plugin::SceneGenerator:: Module handle" << std::endl;
//			auto goal = startState;
			generateState();

//			goal.stateVars = getConfiguration("Goal").getStateVars();
//			startState.print();
//			goal.print();
//			interface->loadState(goal);
		}
	}

	// A script called simCloseModule (by default the main script). Is only called during simulation.
	if (message == sim_message_eventcallback_moduleclose) {
		// is the command also meant for this plugin?
		if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
			// we arrive here only at the end of a simulation
			std::cerr << "Plugin::SceneGenerator:: Module close"  << std::endl;
		}
	}

	// Simulation ended:
	if (message==sim_message_eventcallback_simulationended)	{
	}

	return retVal;
}