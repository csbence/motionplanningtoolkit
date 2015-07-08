#include <iostream>
#include <sstream>
#include <fstream>

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
	const float goalRegionSize[3] = {0.5, 0.5, 0.5};
	const int numberOfInstances = 10;

	PositionSampler<VREPInterface, VREPInterface> *positionSampler;
	VREPInterface::State startState;
	simInt goalRegionHandle;

	simInt createCube() {
		simFloat mass = 0;
		return simCreatePureShape(0, 2 + 16, goalRegionSize, mass, NULL);
	}

	simInt setObjectCollidable(simInt objectHandle) {
		const simInt objectProperties = simGetObjectSpecialProperty(objectHandle);
		simSetObjectSpecialProperty(objectHandle, objectProperties | sim_objectspecialproperty_collidable);
	}

	simInt createGoalRegion() {
		auto goalRegionHandle = createCube();
		setObjectCollidable(goalRegionHandle);
		return goalRegionHandle;
	}

	void setup() {
		positionSampler = new PositionSampler<VREPInterface, VREPInterface>(interface, goalRegionSize);
		VREPInterface::State state;
		interface->makeStartState(state);
		startState = state;

		goalRegionHandle = createGoalRegion();
	}

	void freeMemory() {
		if (positionSampler != NULL) {
			delete positionSampler;
			positionSampler = NULL;
		}
	}

	void writeToFile(const int instanceNumber, const fcl::Transform3f startTransform, const fcl::Transform3f goalTransform) {
		std::fstream file;
		std::stringstream fileName;

		fileName <<
		"/Users/bencecserna/Documents/Development/projects/motionplanningtoolkit/motionplanningtoolkit/vrep/description_files/instances/blimp/";
		fileName << instanceNumber << ".inst";

		file.open(fileName.str(), std::fstream::out);

		if (!file.is_open()) {
			fprintf(stderr, "can't open instance file: %s\n", fileName.str().c_str());
			exit(1);
		}

		const auto startTranslation = startTransform.getTranslation();
		const auto startQuaternion = startTransform.getQuatRotation();

		const auto goalTranslation = goalTransform.getTranslation();
		const auto goalQuaternion = goalTransform.getQuatRotation();

		file << "Agent Start Location ? " << startTranslation[0] << " " << startTranslation[1] << " " <<
		startTranslation[2] << std::endl;
		file << "Agent Start Orientation ? " << startQuaternion[0] << " " << startQuaternion[1] << " " <<
		startQuaternion[2] << " " << startQuaternion[3] << std::endl;

		file << "Agent Goal Location ? " << goalTranslation[0] << " " << goalTranslation[1] << " " <<
		goalTranslation[2] << std::endl;
		file << "Agent Goal Orientation ? " << goalQuaternion[0] << " " << goalQuaternion[1] << " " <<
		goalQuaternion[2] << " " << goalQuaternion[3] << std::endl;

		file.close();
	}

	void generateInstanceFiles() {

		int i = 1;
		while (i <= 10) {
			const auto goalTransform = positionSampler->generateSafeGoalRegion(goalRegionHandle);
			const auto startTransform = positionSampler->generateSafeStartState();

			if (startTransform && goalTransform) {
				writeToFile(i++, goalTransform.get(), startTransform.get());
			}
		}

	}
}

VREP_DLLEXPORT unsigned char v_repStart(void *reservedPointer, int reservedInt) {
	char curDirAndFile[1024];
	getcwd(curDirAndFile, sizeof(curDirAndFile));
	std::string currentDirAndPath(curDirAndFile);
	std::string temp(currentDirAndPath);

#if defined (__linux)
	temp+="/libv_rep.so";
#elif defined (__APPLE__)
	temp += "/libv_rep.dylib";
#endif

	vrepLib = loadVrepLibrary(temp.c_str());
	if (vrepLib == NULL) {
		fprintf(stderr, "Error, could not find or correctly load v_rep.dll. Cannot start 'BubbleRob' plugin.\n");
		return 0;
	}
	if (getVrepProcAddresses(vrepLib) == 0) {
		fprintf(stderr,
				"Error, could not find all required functions in v_rep.dll. Cannot start 'BubbleRob' plugin.\n");
		unloadVrepLibrary(vrepLib);
		return 0;
	}

	// Check the V-REP version:
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
	if (vrepVer < 30200) {
		fprintf(stderr,
				"Sorry, your V-REP copy is somewhat old, V-REP 3.2.0 or higher is required. Cannot start 'BubbleRob' plugin.\n");
		unloadVrepLibrary(vrepLib);
		return 0;
	}

	simChar *instanceFile = simGetStringParameter(sim_stringparam_app_arg1);

	if (instanceFile == NULL || strlen(instanceFile) == 0) {
		fprintf(stderr, "...libv_repExtskiesel.dylib not running\n");
		pluginActive = false;
		return 1;
	}

	args = new InstanceFileMap(instanceFile);

	interface = new VREPInterface(*args);

	simStartSimulation();

	return 1;
}

VREP_DLLEXPORT void v_repEnd() {
	// This is called just once, at the end of V-REP
	unloadVrepLibrary(vrepLib);
}

VREP_DLLEXPORT void *v_repMessage(int message, int *auxiliaryData, void *customData, int *replyData) {
	// This is called quite often. Just watch out for messages/events you want to handle
	// This function should not generate any error messages:
	void *retVal = NULL;

	// A script called simOpenModule (by default the main script). Is only called during simulation.
	if (message == sim_message_eventcallback_moduleopen) {
		// is the command also meant for this plugin?
		if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
			// we arrive here only at the beginning of a simulation
			std::cerr << "Plugin::SceneGenerator:: Module open" << std::endl;

			setup();
			generateInstanceFiles();
		}
	}

	// A script called simHandleModule (by default the main script). Is only called during simulation.
	if (message == sim_message_eventcallback_modulehandle) {
		// is the command also meant for this plugin?
		if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
		}
	}

	// A script called simCloseModule (by default the main script). Is only called during simulation.
	if (message == sim_message_eventcallback_moduleclose) {
		// is the command also meant for this plugin?
		if ((customData == NULL) || (strcmp(pluginName, (char *) customData) == 0)) {
			// we arrive here only at the end of a simulation
			std::cerr << "Plugin::SceneGenerator:: Module close" << std::endl;

			freeMemory();
		}
	}

	// Simulation ended:
	if (message == sim_message_eventcallback_simulationended) {
	}

	return retVal;
}