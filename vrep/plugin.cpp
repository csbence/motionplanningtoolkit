#include <iostream>
#include <unistd.h>
#include "v_repLib.h"
#include "luaFunctionData.h"

#include "../utilities/instancefilemap.hpp"
#include "vrepinterface.hpp"
#include "../planners/rrt.hpp"
#include "../planners/kpiece.hpp"
#include "../samplers/uniformsampler.hpp"
#include "../utilities/flannkdtreewrapper.hpp"
#include "../planners/AnytimeHybridSearch.hpp"

#include <boost/thread/thread.hpp>

#define VREP_DLLEXPORT extern "C"

LIBRARY vrepLib;
VREPInterface *interface;
InstanceFileMap *args;
simFloat start = -1, dt = 1;
VREPInterface::State startState;

namespace {
	const char *const pluginName = "skiesel";

	VREPInterface::State getConfiguration(const char *objectName) {
		VREPInterface::State goal;
		const simInt goalHandle = simGetObjectHandle(objectName);
		simFloat vals[3];
		simGetObjectPosition(goalHandle, -1, vals);
		for (unsigned int i = 0; i < 3; ++i) {
			goal.goalPositionVars.push_back(vals[i]);
			goal.stateVars.push_back(vals[i]);
		}

		simGetObjectOrientation(goalHandle, -1, vals);
		for (unsigned int i = 0; i < 3; ++i) {
			goal.goalOrientationVars.push_back(vals[i]);
			goal.stateVars.push_back(vals[i]);
		}

		// TODO Remove! This is for testing
		for (unsigned int i = 0; i < 6; ++i) {
			goal.stateVars.push_back(0);
		}

		return goal;
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

	args = new InstanceFileMap(instanceFile);

	interface = new VREPInterface(*args);

	simStartSimulation();

//	boost::thread workerThread([&](){
		VREPInterface::State start;
		interface->makeStartState(start);

		VREPInterface::State goal = getConfiguration("Goal");

		typedef UniformSampler<VREPInterface, VREPInterface> Sampler;
		Sampler sampler(*interface, *interface);

		flann::KDTreeSingleIndexParams kdtreeType;

		FLANN_KDTreeWrapper<flann::KDTreeSingleIndexParams,
							flann::L2<double>,
							VREPInterface::Edge> kdtree(kdtreeType, interface->getTreeStateSize());

		typedef AnytimeHybridSearch<VREPInterface, VREPInterface, Sampler> hybridPlanner;

//		hybridPlanner planner(*interface, *interface, sampler, *args);

		RRT<VREPInterface,
			VREPInterface,
			Sampler,
			FLANN_KDTreeWrapper<flann::KDTreeSingleIndexParams, flann::L2<double>, VREPInterface::Edge> > planner(*interface, *interface, sampler, kdtree, *args);

//		KPIECE<VREPInterface, VREPInterface> planner(*interface, *interface, *args);

//		std::cerr << "Start planner\n";

		std::cerr << "Start state: \n";
		start.print();
		start.printGoalPostition();
		start.printGoalOrientation();

		std::cerr << "Goal state: \n";
		goal.print();
		goal.printGoalPostition();
		goal.printGoalOrientation();

//		planner.query(start, goal);
//		std::cerr << "Planner stopped\n";
//	});

	interface->makeStartState(startState);


	std::cout << "Plugin started";

	return 1;
}

VREP_DLLEXPORT void v_repEnd() {
	// This is called just once, at the end of V-REP
	unloadVrepLibrary(vrepLib);
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData) {
	// This is called quite often. Just watch out for messages/events you want to handle
	// This function should not generate any error messages:

	// Keep following 5 lines at the beginning and unchanged:
	static bool refreshDlgFlag=true;
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal=NULL;

//	std::cout << "Plugin::Skiesel:: Start: " << start;

//	if(start < 0) {
//		start = simGetSimulationTime();
//		std::cerr << "Plugin::Skiesel:: Simulation time: " << start << std::endl;
//		std::cerr << "Plugin::Skiesel:: Singal ready" << std::endl;
//		dt = interface->simulatorReady();
//		std::cerr << "Plugin::Skiesel:: Interface ready" << std::endl;
//	} else {
//
//	}
//
//	simFloat curDT = simGetSimulationTime() - start;
//	bool collision = interface->collision();
//	if(collision || dt <= curDT) {
//		const std::string cause = collision ? "Collision detected. " : "Time is up. ";
//		std::cerr << "Plugin::Skiesel:: Simulation is over. " << cause << std::endl;
//
//		interface->simulatorDone(curDT, collision);
//		start = -1;
//	}

	typedef UniformSampler<VREPInterface, VREPInterface> Sampler;
	Sampler sampler(*interface, *interface);

	auto state = sampler.sampleConfiguration();
//	VREPInterface::State start;
//	interface->makeStartState(start);


    // A script called simOpenModule (by default the main script). Is only called during simulation.
    if (message == sim_message_eventcallback_moduleopen) {
        // is the command also meant for this plugin?
        if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
            // we arrive here only at the beginning of a simulation
			std::cerr << "Plugin::Skiesel:: Module open" << std::endl;
		}
    }

    // A script called simHandleModule (by default the main script). Is only called during simulation.
    if (message == sim_message_eventcallback_modulehandle) {
        // is the command also meant for this plugin?
        if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
            // we arrive here only while a simulation is running
			std::cerr << "Plugin::Skiesel:: Module handle" << std::endl;
			auto goal = startState;

			goal.stateVars = getConfiguration("Goal").getStateVars();
			startState.print();
			goal.print();
			interface->loadState(goal);
		}
    }

    // A script called simCloseModule (by default the main script). Is only called during simulation.
    if (message == sim_message_eventcallback_moduleclose) {
        // is the command also meant for this plugin?
        if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
            // we arrive here only at the end of a simulation
			std::cerr << "Plugin::Skiesel:: Module close"  << std::endl;
		}
    }

	// Simulation ended:
	if (message==sim_message_eventcallback_simulationended)	{
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return retVal;
}