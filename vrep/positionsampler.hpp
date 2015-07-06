#pragma once

#include <random>
#include <fcl/math/transform.h>
#include <boost/optional.hpp>

template<typename Workspace, typename Agent>
class PositionSampler {
public:

	typedef typename Agent::State State;

	PositionSampler(Workspace *workspace) : workspace(workspace) {
		auto bounds = workspace->getBounds();

		for (auto range : bounds) {
			linearDistributions.emplace_back(range.first, range.second);
		}
	}

	boost::optional<State> generateSafeState(State &canonicalState) const {
		boost::optional<State> state;
		auto transform = generateRandomValidTransform(canonicalState);

		if (transform) {
			state = Agent::transformToState(canonicalState, transform.get(), 0);
		}

		return state;
	}

	boost::optional<fcl::Transform3f> generateRandomValidTransform(State &canonicalState) const {
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

	fcl::Transform3f generateRandomTransform() const {
		const fcl::Vec3f translation = getRandomVector(linearDistributions, generator);
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
		fcl::Vec3f axis(0, 0, 1);
		quaternion.fromAxisAngle(axis, rad);
		return quaternion;
	}

private:
	const Workspace *workspace;
	mutable std::vector<std::uniform_real_distribution<double> > linearDistributions;
	mutable std::default_random_engine generator;
};
