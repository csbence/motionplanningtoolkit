#pragma once

#include <random>
#include <algorithm>
#include <fcl/math/transform.h>
#include <boost/optional.hpp>

template<typename Workspace, typename Agent>
class PositionSampler {
public:
	typedef typename Agent::State State;

	PositionSampler(Workspace *workspace, const float offset[]) : workspace(workspace) {
		environmentCollisionHandle = workspace->getEnvironmentCollisionHandle();

		const auto workspaceBounds = workspace->getBounds();
		const auto startBounds = workspace->getStartBounds();
		const auto goalBounds = workspace->getGoalBounds();

		double workspaceLowerBound, workspaceUpperBound;
		double startLowerBound, startUpperBound;
		double goalLowerBound, goalUpperBound;

		for (int i = 0; i < 3; ++i) {

			// Workspace
			workspaceLowerBound = workspaceBounds[i].first + offset[i] / 2;
			workspaceUpperBound = workspaceBounds[i].second - offset[i] / 2;

			workspaceDistributions.emplace_back(workspaceLowerBound, workspaceUpperBound);
			std::cerr << "Workspace Bounds (no offset)::  " << i << " :: " << workspaceBounds[i].first << " - " << workspaceBounds[i].second <<
			std::endl;
			std::cerr << "Workspace Bounds::  " << i << " :: " << workspaceLowerBound << " - " << workspaceUpperBound <<
			std::endl;

			// Start
			startLowerBound = startBounds[i].first;
			startUpperBound = startBounds[i].second;
			startLowerBound = std::max(startLowerBound, workspaceLowerBound);
			startUpperBound = std::min(startUpperBound, workspaceUpperBound);

			startDistributions.emplace_back(startLowerBound, startUpperBound);

			std::cerr << "Start Bounds::      " << i << " :: " << startLowerBound << " - " << startUpperBound <<
			std::endl;

			// Goal
			goalLowerBound = goalBounds[i].first;
			goalUpperBound = goalBounds[i].second;
			goalLowerBound = std::max(goalLowerBound, workspaceLowerBound);
			goalUpperBound = std::min(goalUpperBound, workspaceUpperBound);

			goalDistributions.emplace_back(goalLowerBound, goalUpperBound);

			std::cerr << "Goal Bounds::       " << i << " :: " << goalLowerBound << " - " << goalUpperBound <<
			std::endl;
		}
	}

	boost::optional<fcl::Transform3f> generateSafeGoalRegion(simInt goalRegionHandle) const {
		boost::optional<fcl::Transform3f> validTransform;
		for (int i = 0; i < 1000; i++) {
			simFloat vals[4];
			const fcl::Vec3f &position = getRandomVector(goalDistributions, generator);
			const fcl::Quaternion3f quaternion;

			for (unsigned int i = 0; i < 3; ++i)
				vals[i] = position[i];

			simSetObjectPosition(goalRegionHandle, -1, vals);

			if (simCheckCollision(goalRegionHandle, environmentCollisionHandle) == 0) {
				validTransform = fcl::Transform3f(quaternion, position);
				break;
			}
		}

		return validTransform;
	}

	boost::optional<fcl::Transform3f> generateSafeStartState() const {
		const simInt agentHandle = workspace->getAgentHandle();

		boost::optional<fcl::Transform3f> validTransform;
		for (int i = 0; i < 1000; i++) {
			simFloat vals[4];

			const fcl::Vec3f &position = getRandomVector(startDistributions, generator);

			for(unsigned int i = 0; i < 3; ++i)
				vals[i] = position[i];

			simSetObjectPosition(agentHandle, -1, vals);

			// Generate random Euler rotation
			simFloat rad = zeroToOne(generator) * 2 * M_PI;
			simFloat rotation[3] = { -M_PI / 2, rad, -M_PI / 2};

			simSetObjectOrientation(agentHandle, -1, rotation);

			if (simCheckCollision(agentHandle, environmentCollisionHandle) == 0) {

				// Extract quaternion
				simFloat quaternion[4];
				simGetObjectQuaternion(agentHandle, -1, quaternion);

				fcl::Quaternion3f quaternion3f(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

				validTransform = fcl::Transform3f(quaternion3f, position);
				break;
			}
		}

		return validTransform;
	}

	boost::optional<State> generateSafeState(Agent &agent, State &canonicalState) const {
		boost::optional<State> state;
		auto transform = generateRandomValidTransform(agent, canonicalState);

		if (transform) {
			state = workspace->translationToState(canonicalState, transform.get().getTranslation());
		}

		return state;
	}

	boost::optional<fcl::Transform3f> generateRandomValidTransform(Agent &agent, State &canonicalState) const {
		boost::optional<fcl::Transform3f> validTransform;

		for (int i = 0; i < 1000; i++) {
			auto transform = generateRandomTransform();

			if (workspace->safePose(agent, transform, canonicalState)) {
				validTransform.emplace(transform);
				break;
			}
		}

		return validTransform;
	}

	fcl::Transform3f generateRandomTransform(std::vector<std::uniform_real_distribution<double> > &dist) const {
		const fcl::Vec3f translation = getRandomVector(dist, generator);
		const fcl::Quaternion3f rotation = getRandomZOnlyQuaternion();
		const fcl::Transform3f transform(rotation, translation);

		return transform;
	}

	fcl::Vec3f getRandomVector(std::vector<
			std::uniform_real_distribution<double> > &distributions, std::default_random_engine &generator) const {
		fcl::Vec3f vector;
		for (unsigned int i = 0; i < distributions.size(); ++i) {
			vector[i] = distributions[i](generator);
		}
		return vector;
	}

	fcl::Quaternion3f getRandomZOnlyQuaternion() const {
		double rad = zeroToOne(generator) * 2 * M_PI;
		fcl::Quaternion3f quaternion;
		fcl::Vec3f axis(1, 0, 0);
		quaternion.fromAxisAngle(axis, rad);
		return quaternion;
	}

private:
	simInt environmentCollisionHandle;
	const Workspace *workspace;
	mutable std::vector<
			std::uniform_real_distribution<double> > workspaceDistributions, startDistributions, goalDistributions;
	mutable std::default_random_engine generator;
	mutable std::uniform_real_distribution<double> zeroToOne;
};
