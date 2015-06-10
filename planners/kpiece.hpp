#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

template<class Workspace, class Agent>
class KPIECE {
public:

	class WrappedState : public ompl::base::RealVectorStateSpace::StateType {
	public:
		const typename Agent::Edge *agentEdge;
		bool valid;
	};

	KPIECE(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), foundGoal(false) {
			collisionCheckDT = stod(args.value("Collision Check Delta t"));

			const typename Workspace::WorkspaceBounds &workspaceBounds = workspace.getBounds();
			stateSpaceDim = workspaceBounds.size();

			ompl::base::RealVectorStateSpace *baseSpace = new ompl::base::RealVectorStateSpace(stateSpaceDim);
			ompl::base::RealVectorBounds bounds(stateSpaceDim);
			for(unsigned int i = 0; i < stateSpaceDim; ++i) {
				bounds.setLow(i, workspaceBounds[i].first);
				bounds.setHigh(i, workspaceBounds[i].second);
			}
			baseSpace->setBounds(bounds);
			ompl::base::StateSpacePtr space = ompl::base::StateSpacePtr(baseSpace);

			const std::vector< std::pair<double, double> > &controlBounds = workspace.getControlBounds();
			controlSpaceDim = controlBounds.size();

			ompl::control::RealVectorControlSpace *baseCSpace = new ompl::control::RealVectorControlSpace(space, controlSpaceDim);
			ompl::base::RealVectorBounds cbounds(controlSpaceDim);
			for(unsigned int i = 0; i < controlSpaceDim; ++i) {
				cbounds.setLow(i, controlBounds[i].first);
				cbounds.setHigh(i, controlBounds[i].second);
			}

			baseCSpace->setBounds(cbounds);
			ompl::control::ControlSpacePtr cspace(baseCSpace);

			// construct an instance of  space information from this control space
			spaceInfoPtr = ompl::control::SpaceInformationPtr(new ompl::control::SpaceInformation(space, cspace));

			// set state validity checking for this space
			spaceInfoPtr->setStateValidityChecker(boost::bind(&KPIECE::isStateValid, this, spaceInfoPtr.get(), _1));

			// set the state propagation routine
			spaceInfoPtr->setStatePropagator(boost::bind(&KPIECE::propagate, this, _1, _2, _3, _4));

			pdef = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(spaceInfoPtr));

			kpiece = new ompl::control::KPIECE1(spaceInfoPtr);
		}

	~KPIECE() {
		delete kpiece;
	}

	bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) const {
		return state->as<WrappedState>()->valid;
	}

	void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result) {
		const WrappedState *wrappedState = start->as<WrappedState>();
		const ompl::control::RealVectorControlSpace::ControlType *realVectorControl = control->as<ompl::control::RealVectorControlSpace::ControlType>();

		typename Agent::Control agentControl(controlSpaceDim);
		typename Agent::Edge edge = workspace.steerWithControl(wrappedState->agentEdge->end, agentControl, duration);

		const typename Agent::StateVars &endStateVars = edge.getTreeStateVars();

		WrappedState* resultWrappedState = result->as<WrappedState>();

		resultWrappedState->agentEdge = new typename Agent::Edge(edge);

		for(unsigned int i = 0; i < endStateVars.size(); ++i) {
			resultWrappedState->values[i] = endStateVars[i];
		}

		resultWrappedState->valid = workspace.safeEdge(agent, edge, collisionCheckDT);

		foundGoal |= workspace.isGoal(edge.end, *agentGoal);
	}

	bool didFindGoal() const {
		return foundGoal;
	}

	void query(const typename Agent::State &start, const typename Agent::State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {
		WrappedState omplStart;
		omplStart.agentEdge = new typename Agent::Edge(start);
		omplStart.valid = true;

		const typename Agent::StateVars &startStateVars = omplStart.agentEdge->getTreeStateVars();
		for(unsigned int i = 0; i < startStateVars.size(); ++i) {
			omplStart.values[i] = startStateVars[i];
		}

		agentGoal = new typename Agent::State(goal);

		WrappedState omplGoal;
		omplGoal.agentEdge = new typename Agent::Edge(start);
		omplGoal.valid = true;

		const typename Agent::StateVars &goalStateVars = omplGoal.agentEdge->getTreeStateVars();;
		for(unsigned int i = 0; i < goalStateVars.size(); ++i) {
			omplGoal.values[i] = goalStateVars[i];
		}

		pdef->setStartAndGoalStates(&omplStart, &omplGoal);

		kpiece->setProblemDefinition(pdef);

		kpiece->setup();

		ompl::base::PlannerTerminationCondition tc =  ompl::base::plannerOrTerminationCondition(
														ompl::base::timedPlannerTerminationCondition(300),
														ompl::base::PlannerTerminationCondition(boost::bind(&KPIECE::didFindGoal, this)));

		ompl::base::PlannerStatus solved = kpiece->solve(tc);

		if (solved) {
			fprintf(stderr, "found goal\n");
			ompl::base::PathPtr path = pdef->getSolutionPath();
			path->print(std::cout);
		} else {
			fprintf(stderr, "did not find goal\n");
		}
	}

private:
	const Workspace &workspace;
	const Agent &agent;
	typename Agent::State *agentGoal;
	ompl::control::KPIECE1 *kpiece;
	ompl::control::SpaceInformationPtr spaceInfoPtr;
	ompl::base::ProblemDefinitionPtr pdef;
	unsigned int stateSpaceDim, controlSpaceDim;
	double collisionCheckDT;
	bool foundGoal;
};