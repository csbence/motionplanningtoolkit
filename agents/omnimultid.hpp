#pragma once

class OmniMultiD {
public:
	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	typedef std::vector <std::pair<double, double>> WorkspaceBounds;
	typedef std::vector <std::pair<double, double>> StateVarRanges;

	class State;

	class Edge;

	typedef State AbstractState;
	typedef std::vector <AbstractState> AbstractEdge;

	class State {
	public:
		State(const int dimensions) : treeStateVars(dimensions) {
		}

		State(const State &) = default;

		State(State &&) = default;

		State &operator=(const State &) = default;

		State &operator=(State &&) = default;

		explicit State(StateVars vars) : treeStateVars(std::move(vars)) {
		}

		const StateVars &getTreeStateVars() const {
			return treeStateVars;
		}

		const StateVars &getStateVars() const {
			return treeStateVars;
		}

		bool equals(const State &s) const {
			for (unsigned int i = 0; i < getStateVars().size(); ++i) {
				if (fabs(getStateVars()[i] - s.getStateVars()[i]) > 0.000001) {
					return false;
				}
			}
			return true;
		}

		void print() const {
			const auto &stateVars = getStateVars();
			for (auto v : stateVars) {
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

		void move(Control control) {
			const int dimensions = treeStateVars.size();
			assert(control.size() == dimensions);

			for (int i = 0; i < dimensions; ++i) {
				treeStateVars[i] = treeStateVars[i] + control[i];
			}
		}

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			if (treeStateVars.size() > 2) {
				const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

				std::vector<double> pt(treeStateVars.begin(), treeStateVars.begin() + 3);
				pt.push_back(1);
				pt.push_back(0);
				pt.push_back(0);
				pt.push_back(1);
				pt.push_back(1);
				pt.insert(pt.end(), color.getColor().begin(), color.getColor().end());
				pt.insert(pt.end(), identity.begin(), identity.end());

				OpenGLWrapper::getOpenGLWrapper().drawPoints(std::move(pt));
			} else {
				const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

				std::vector<double> pt(treeStateVars.begin(), treeStateVars.begin() + 2);
				pt.push_back(0);
				pt.push_back(1);
				pt.push_back(0);
				pt.push_back(0);
				pt.push_back(1);
				pt.push_back(1);
				pt.insert(pt.end(), color.getColor().begin(), color.getColor().end());
				pt.insert(pt.end(), identity.begin(), identity.end());

				OpenGLWrapper::getOpenGLWrapper().drawPoints(std::move(pt));
			}
		}

#endif

		std::vector <State> getTransforms() const {
			return std::vector < State > {*this};
		}

		static std::vector <State> interpolate(const State &a, const State &b, const double dt) {
			const int dimensions = a.getStateVars().size();
			std::vector <State> intermediateStates;

			BOOST_ASSERT(dimensions == b.getStateVars().size());
			BOOST_ASSERT(dimensions > 0);

			// Find largest difference
			double maxDifference = 0;
//			int maxDifferenceIndex = 0;
			for (int i = 0; i < dimensions; ++i) {
				double difference = std::abs(a.getStateVars()[i] - b.getStateVars()[i]);
				if (difference > maxDifference) {
					maxDifference = difference;
//					maxDifferenceIndex = i;
				}
			}

			const int numberOfSteps = maxDifference / dt;

			// If there are no intermediate states
			if (numberOfSteps == 0) {
				return intermediateStates;
			}

			// Calculate step sizes
			std::vector<double> stepSizes(dimensions);
			for (int i = 0; i < dimensions; ++i) {
				double difference = b.getStateVars()[i] - a.getStateVars()[i];
				stepSizes[i] = difference / numberOfSteps;
			}

			// Generate intermediate states;
			for (int i = 0; i < numberOfSteps; ++i) {

				StateVars intermediateStateVars(dimensions);

				for (int j = 0; j < dimensions; ++j) {
					intermediateStateVars[j] = a.getStateVars()[j] + i * stepSizes[j];
				}

				intermediateStates.emplace_back(std::move(intermediateStateVars));
			}

			return intermediateStates;
		}

		static State getRandomAbstractState(const std::vector <std::pair<double, double>> &bounds) {
			std::vector<double> stateVars;
			for (std::pair<double, double> lowerUpper : bounds) {
				std::uniform_real_distribution<double> distribution(lowerUpper.first, lowerUpper.second);
				stateVars.push_back(distribution(GlobalRandomGenerator));
			}

			return State(stateVars);
		}

		static double evaluateDistance(const State &rhs, const State &lhs) {
			const auto &rhsStateVars = rhs.getStateVars();
			const auto &lhsStateVars = lhs.getStateVars();

			BOOST_ASSERT_MSG(rhsStateVars.size() == lhsStateVars.size(),
							 "Cannot evaluate the distance of two states with different dimensionality.");

			const int dimensions = rhsStateVars.size();

			double linearDistance = 0;
			double euclideanSum = 0;

			for (int i = 0; i < dimensions; ++i) {
				double diff = std::abs(rhsStateVars[i] - lhsStateVars[i]);
				linearDistance += diff;
				euclideanSum += diff * diff;
			}

			return std::sqrt(euclideanSum);
		}

	private:
		StateVars treeStateVars;
	};

	class Edge {
	public:
		Edge(const State &s) : start(s),
							   end(s),
							   duration(0),
							   g(0),
							   parent(NULL) {
		}

		Edge(const State start, const State end, double cost, const std::vector<double> control, double duration)
				: start(std::move(start)),
				  end(std::move(end)),
				  cost(cost),
				  g(0),
				  treeIndex(0),
				  duration(duration),
				  control(std::move(control)),
				  parent(NULL) {
		}

		Edge(const Edge &) = default;

		Edge(Edge &&) = default;

		Edge &operator=(const Edge &) = default;

		Edge &operator=(Edge &&) = default;

		double gCost() const {
			return g;
		}

		void updateParent(Edge *p) {
			parent = p;
			g = p->gCost() + cost;
		}

		void print() {
			start.print();
			end.print();
		}

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &startVars = start.getStateVars();
			const auto &endVars = end.getStateVars();

			if (startVars.size() > 2) {
				OpenGLWrapper::getOpenGLWrapper()
						.drawLine(startVars[0], startVars[1], startVars[2], endVars[0], endVars[1], endVars[2], color);
			} else {
				OpenGLWrapper::getOpenGLWrapper()
						.drawLine(startVars[0], startVars[1], 0, endVars[0], endVars[1], 0, color);
			}

		}

#endif

		/* needed for being inserted into NN datastructure */
		const StateVars &getTreeStateVars() const {
			return end.getStateVars();
		}

		int getPointIndex() const {
			return treeIndex;
		}

		void setPointIndex(int ptInd) {
			treeIndex = ptInd;
		}

		Control getControl() const {
			return control;
		}

		State start, end;
		double cost, duration, g;
		int treeIndex;
		Edge *parent;
		Control control;
	};

	OmniMultiD(const InstanceFileMap &args) : dimensions(args.integerVal("Dimensions")),
											  workspaceBounds(dimensions) {

		for (int i = 0; i < dimensions; ++i) {
			workspaceBounds[i].first = 0;
			workspaceBounds[i].second = 1;
		}

		const auto &stateVarDomains = getStateVarRanges(workspaceBounds);
		
		std::uniform_real_distribution<double> distribution(-1, 1);
		distributions.resize(stateVarDomains.size(), distribution);

		boost::char_separator<char> sep(" ");
		boost::tokenizer <boost::char_separator<char>> tokens(args.value("Goal Thresholds"), sep);
		for (const auto &token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}
	}

	unsigned int getTreeStateSize() const {
		return dimensions;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds &bounds) const {
		return bounds;
	}

	bool isGoal(const State &state, const State &goal) const {
		const auto &targetStateVars = state.getStateVars();
		const auto &goalStateVars = goal.getStateVars();
		const int numberOfLinks = targetStateVars.size();

		BOOST_ASSERT_MSG(numberOfLinks == goalStateVars.size(),
						 "The goal state and the candidate state must have the same dimensionality");

		BOOST_ASSERT_MSG(numberOfLinks == dimensions,
						 "The goal state and the goal state thresholds must have the same dimensionality");

		double difference = 0;
		for (int i = 0; i < numberOfLinks; ++i) {
			difference = targetStateVars[i] - goalStateVars[i];
			if (std::abs(difference) > goalThresholds[i]) {
				return false;
			}
		}

		return true;
	}

	Edge randomSteer(const State &start, double dt) const {
		const int dimensions = start.getStateVars().size();
		Control controls(dimensions);
		StateVars stateVars(dimensions);

		double max = 0;

		for (int i = 0; i < dimensions; ++i) {
<<<<<<< HEAD
<<<<<<< HEAD
			auto &distribution = distributions[i];
			double v = distribution(GlobalRandomGenerator) * dt;
=======
			double v = distributions[i](GlobalRandomGenerator) * dt;
>>>>>>> skiesel/master
=======
			double v = distributions[i](GlobalRandomGenerator) * dt;
>>>>>>> skiesel/master

			max = std::max(max, v);
			controls[i] = v;
			stateVars[i] = start.getStateVars()[i] + v;
		}

		State newState = buildState(stateVars);
		return Edge(start, newState, State::evaluateDistance(start, newState), controls, max);
	}

	Edge steerWithControl(const State &start, const Edge &getControlFromThisEdge, double dt) const {
		return steerWithControl(start, getControlFromThisEdge.getControl(), dt);
	}

	Edge steerWithControl(const State &start, const Control controls, double dt) const {
		const int numberOfLinks = start.getStateVars().size();
		StateVars stateVars(numberOfLinks);

		BOOST_ASSERT(numberOfLinks == controls.size());

		double max = 0;

		for (int i = 0; i < numberOfLinks; ++i) {
			double v = controls[i];
			max = std::max(max, v);
			stateVars[i] = start.getStateVars()[i] + v;
		}

		State newState = buildState(stateVars);
		return Edge(start, newState, State::evaluateDistance(start, newState), controls, max);
	}

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> skiesel/master
	Edge steer(const State &start, const State &goal, double dt) const {
		double dist = State::evaluateDistance(start, goal);
		double scale = dt / dist;
		if(scale > 1) scale = 1;

		const auto &startVars = start.getStateVars();
		const auto &goalVars = goal.getStateVars();
		
		StateVars stateVars(startVars.size());
		Control controls(startVars.size());

		for (int i = 0; i < startVars.size(); ++i) {
			double diff = goalVars[i] - startVars[i];
			controls[i] = diff * scale;
			stateVars[i] = startVars[i] + controls[i];
		}

		State newState = buildState(stateVars);
		return Edge(start, newState, State::evaluateDistance(start, newState), controls, scale * dt);
	}

<<<<<<< HEAD
>>>>>>> skiesel/master
=======
>>>>>>> skiesel/master
	Edge constructEdge(const State &start, const State &end) const {
		fprintf(stderr, "OmniMultiD::constructEdge not implemented\n");
		exit(1);
	}

	State buildState(const StateVars &stateVars) const {
		return State(stateVars);
	}

#ifdef WITHGRAPHICS

	void drawMesh() {
	}

	void drawMesh(const State &s) const {
	}

	void drawMesh(const fcl::Transform3f &transform, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
	}

	void drawSolution(const fcl::Transform3f &transform, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
	}

	void drawSolution(const std::vector<const Edge *> &solution, double dt = std::numeric_limits<
			double>::infinity()) const {
	}

#endif

	WorkspaceBounds getControlBounds() const {
		return workspaceBounds; // TODO define control bounds
	}

	Control controlFromVector(const Control &controls) const {
		return controls;
	}

	State getRandomStateNearAbstractState(const AbstractState &state, double radius) const {
		const auto &sourceStateVars = state.getStateVars();
		const int dimensions = sourceStateVars.size();
		Control controls(dimensions);
		StateVars stateVars(dimensions);

		for (int i = 0; i < dimensions; ++i) {
			auto &distribution = distributions[i];
			double v = distribution(GlobalRandomGenerator);
			controls[i] = v;
		}

		State tempState(controls);
		const double distance = State::evaluateDistance(state, tempState);
		const double ratio = distance < radius ? 1 : radius / distance; // Not uniform!

		for (int i = 0; i < dimensions; ++i) {
			stateVars[i] = sourceStateVars[i] + controls[i] * ratio;
		}

		return State(stateVars);
	}

	AbstractState toAbstractState(const State &s) const {
		return s;
	}

	inline bool areAbstractEdgesSymmetric() const {
		return true;
	}

	AbstractEdge generateAbstractEdge(const AbstractState &s1, const AbstractState &s2) const {
		std::vector <AbstractState> edge = {s1, s2};
		return edge;
	}

#ifdef WITHGRAPHICS

	void draw() const {
	}

#endif

private:
	const int dimensions;
	std::vector<double> goalThresholds;
	WorkspaceBounds workspaceBounds; // TODO remove

	mutable std::vector <std::uniform_real_distribution<double>> distributions;
};
