#pragma once

#include <cmath>
#include <stdlib.h>

#include "../utilities/fcl_helpers.hpp"
#include "../utilities/openglwrapper.hpp"
#include "../utilities/instancefilemap.hpp"

class Omnidirectional {
public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;

	class State {
	public:
		State() : stateVars(3) {
			stateVars[0] = 0;
			stateVars[1] = 0;
			stateVars[2] = 0;
		}
		State(double x, double y, double z = 0) : stateVars(3) {
			stateVars[0] = x;
			stateVars[1] = y;
			stateVars[2] = z;
		}
		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.begin()+3) {}

		double x() const { return stateVars[0]; }
		double y() const { return stateVars[1]; }
		double z() const { return stateVars[2]; }
		const StateVars& getStateVars() const { return stateVars; }

		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

#ifdef WITHGRAPHICS
		void draw() const {
			std::vector<double> pt(stateVars.begin(), stateVars.end());
			pt.push_back(1);
			OpenGLWrapper::getOpenGLWrapper().drawPoints(pt);
		}
#endif

	private:
		StateVars stateVars;
		int treeIndex;
	};

	class Edge {
	public:
		Edge(const State &start, const State &end, double cost) : start(start), end(end), cost(cost), treeIndex(0) {}
		Edge(const Edge& e) : start(e.start), end(e.end), cost(e.cost), treeIndex(e.treeIndex) {}

		/* needed for being inserted into NN datastructure */
		const StateVars& getStateVars() const { return end.getStateVars(); }
		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

#ifdef WITHGRAPHICS
		void draw() const {
			std::vector<double> line(start.getStateVars().begin(), start.getStateVars().end());
			line.push_back(1);
			line.insert(line.end(), end.getStateVars().begin(), end.getStateVars().end());
			line.push_back(1);
			OpenGLWrapper::getOpenGLWrapper().drawLines(line);
		}
#endif

		const State start, end;
		double cost;
		int treeIndex;
	};

	Omnidirectional(const InstanceFileMap &args) :
		mesh(args.value("Agent Mesh")) {

		}

	StateVarRanges getStateVarRanges(const WorkspaceBounds& bounds) const {
		return bounds;
	}

	State buildState(const StateVars& stateVars) const {
		return State(stateVars);
	}

	bool isGoal(const State &state, const State &goal) const {
		return fabs(state.x() - goal.x()) < 0.01 &&
		fabs(state.y() - goal.y()) < 0.01 &&
		fabs(state.z() - goal.z()) < 0.01;
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		double startX = start.x();
		double startY = start.y();
		double startZ = start.z();

		double dx = goal.x() - startX;
		double dy = goal.y() - startY;
		double dz = goal.z() - startZ;

		double dist = sqrt(dx*dx + dy*dy + dz*dz);
		double fraction = dt / dist;
		if(fraction > 1) fraction = 1;


		State state(startX + dx * fraction,
					startY + dy * fraction,
					startZ + dz * fraction);

		return Edge(start, state, dt);
	}

	Edge randomSteer(const State &start, double dt) const {
		double startX = start.x();
		double startY = start.y();
		double startZ = start.z();

		double randX = (double)rand() / (double)RAND_MAX;
		double randY = (double)rand() / (double)RAND_MAX;
		double randZ = (double)rand() / (double)RAND_MAX;

		double dist = sqrt(randX*randX + randY*randY + randZ*randZ);

		State state(startX + randX / dist,
					startY + randY / dist,
					startZ + randZ / dist);

		return Edge(start, state, dt);
	}

	const SimpleAgentMeshHandler& getMesh() const {
		return mesh;
	}

	std::vector<fcl::Transform3f> getPoses(const Edge &edge, double dt) const {
		std::vector<fcl::Transform3f> poses;
		
		double startX = edge.start.x();
		double startY = edge.start.y();
		double startZ = edge.start.z();

		double endX = edge.end.x();
		double endY = edge.end.y();
		double endZ = edge.end.z();

		double dx = endX - startX;
		double dy = endY - startY;
		double dz = endZ - startZ;

		double dist = sqrt(dx*dx + dy*dy + dz*dz);
		unsigned int iterations = dist / dt;

		if(iterations < 1) {
			fcl::Vec3f start(startX, startY, startZ);
			fcl::Vec3f end(endX, endY, endZ);
			poses.push_back(fcl::Transform3f(start));
			poses.push_back(fcl::Transform3f(end));
		} else {
			double step = dt / dist;

			for(unsigned int i = 0; i < iterations; ++i) {
				double stepSize = step * (double)i;

				fcl::Vec3f translation(startX + stepSize * dx, 
										startY + stepSize * dy,
										startZ + stepSize * dz);
				poses.push_back(fcl::Transform3f(translation));
			}
			if((double)iterations * dt < dist) {
				fcl::Vec3f end(endX, endY, endZ);
				poses.push_back(fcl::Transform3f(end));
			}
		}

		return poses;
	}

private:
	SimpleAgentMeshHandler mesh;
};