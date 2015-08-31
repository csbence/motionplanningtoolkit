#pragma once

#include <vector>
#include <math.h>
#include <random>

namespace Planar {
<<<<<<< HEAD
	class PlanarVector {
	public:
		PlanarVector(double f = 0.0f) : x(f),
										y(f) {
		}

		PlanarVector(double x, double y) : x(x),
										   y(y) {
		}

		PlanarVector(const PlanarVector &) = default;

		PlanarVector(PlanarVector &&) = default;

		PlanarVector &operator=(const PlanarVector &) = default;

		PlanarVector &operator=(PlanarVector &&) = default;

		bool operator==(PlanarVector &rhs) {
			return x == rhs.x && y == rhs.y;
		}

		double x, y;
	};

	/**
	 * From: http://paulbourke.net/geometry/pointlineplane/
	 * Based on the implementation of Damian Coventry
	 */
	class LineSegment {
	public:
		PlanarVector begin;
		PlanarVector end;

		LineSegment() {
		}

		LineSegment(const PlanarVector &begin, const PlanarVector &end) : begin(begin),
																		  end(end) {
		}

		enum class IntersectResult { PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING, OVERLAPPING };

		IntersectResult intersect(const LineSegment &segment, PlanarVector &intersection) {
			double denom = ((segment.end.y - segment.begin.y) * (end.x - begin.x)) -
						   ((segment.end.x - segment.begin.x) * (end.y - begin.y));

			double numerA = ((segment.end.x - segment.begin.x) * (begin.y - segment.begin.y)) -
							((segment.end.y - segment.begin.y) * (begin.x - segment.begin.x));

			double numberB = ((end.x - begin.x) * (begin.y - segment.begin.y)) -
							 ((end.y - begin.y) * (begin.x - segment.begin.x));

			if (denom == 0.0f) {
				if (numerA == 0.0f && numberB == 0.0f) {
					// The lines are coincident. Check if the segments are overlapping or not.
					// It is sufficient to check only one dimension, since the points are on the same line.

					double localMin = std::min(begin.x, end.x);
					double localMax = std::max(begin.x, end.x);

					double segmentMin = std::min(segment.begin.x, segment.end.x);
					double segmentMax = std::max(segment.begin.x, segment.end.x);

					return (localMax <= segmentMin || segmentMax <= localMin) ? IntersectResult::PARALLEL
																			  : IntersectResult::OVERLAPPING;
//					return IntersectResult::COINCIDENT;
				}

				return IntersectResult::PARALLEL;
			}

			double ua = numerA / denom;
			double ub = numberB / denom;

			if (ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f) {
				// Get the intersection point.
				intersection.x = begin.x + ua * (end.x - begin.x);
				intersection.y = begin.y + ua * (end.y - begin.y);

				return IntersectResult::INTERESECTING;
			}

			return IntersectResult::NOT_INTERESECTING;
		}

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			OpenGLWrapper::getOpenGLWrapper()
					.drawLine(begin.x, begin.y, 0, end.x, end.y, 0, OpenGLWrapper::Color::Blue());
		}

#endif
	};

	/**
	 * @return True if the segments are coincident or intersecting.
	 */
	bool checkCollision(LineSegment segment1, LineSegment segment2) {
		PlanarVector intersection;

		LineSegment::IntersectResult intersectResult = segment1.intersect(segment2, intersection);

		// Filter out joint collision for neighbor segments.
		if (segment1.end == segment2.begin || segment2.end == segment1.begin) {
			return intersectResult == LineSegment::IntersectResult::COINCIDENT;
		}

		return intersectResult == LineSegment::IntersectResult::COINCIDENT ||
			   intersectResult == LineSegment::IntersectResult::INTERESECTING;
	}
=======
class PlanarVector {
public:
	PlanarVector(double f = 0.0f) : x(f),
		y(f) {
	}

	PlanarVector(double x, double y) : x(x),
		y(y) {
	}

	PlanarVector(const PlanarVector &) = default;

	PlanarVector(PlanarVector &&) = default;

	PlanarVector &operator=(const PlanarVector &) = default;

	PlanarVector &operator=(PlanarVector &&) = default;

	bool operator==(PlanarVector &rhs) {
		return x == rhs.x && y == rhs.y;
	}

	double x, y;
};

/**
 * From: http://paulbourke.net/geometry/pointlineplane/
 * Based on the implementation of Damian Coventry
 */
class LineSegment {
public:
	PlanarVector begin;
	PlanarVector end;

	LineSegment() {
	}

	LineSegment(const PlanarVector &begin, const PlanarVector &end) : begin(begin),
		end(end) {
	}

	enum class IntersectResult {
	    PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING, OVERLAPPING
	};

	IntersectResult intersect(const LineSegment &segment, PlanarVector &intersection) {
		double denom = ((segment.end.y - segment.begin.y) * (end.x - begin.x)) -
		               ((segment.end.x - segment.begin.x) * (end.y - begin.y));

		double numerA = ((segment.end.x - segment.begin.x) * (begin.y - segment.begin.y)) -
		                ((segment.end.y - segment.begin.y) * (begin.x - segment.begin.x));

		double numberB = ((end.x - begin.x) * (begin.y - segment.begin.y)) -
		                 ((end.y - begin.y) * (begin.x - segment.begin.x));

		if(denom == 0.0f) {
			if(numerA == 0.0f && numberB == 0.0f) {
				// The lines are coincident. Check if the segments are overlapping or not.
				// It is sufficient to check only one dimension, since the points are on the same line.

				double localMin = std::min(begin.x, end.x);
				double localMax = std::max(begin.x, end.x);

				double segmentMin = std::min(segment.begin.x, segment.end.x);
				double segmentMax = std::max(segment.begin.x, segment.end.x);

				return (localMax <= segmentMin || segmentMax <= localMin) ? IntersectResult::PARALLEL
				       : IntersectResult::OVERLAPPING;
//					return IntersectResult::COINCIDENT;
			}

			return IntersectResult::PARALLEL;
		}

		double ua = numerA / denom;
		double ub = numberB / denom;

		if(ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f) {
			// Get the intersection point.
			intersection.x = begin.x + ua * (end.x - begin.x);
			intersection.y = begin.y + ua * (end.y - begin.y);

			return IntersectResult::INTERESECTING;
		}

		return IntersectResult::NOT_INTERESECTING;
	}

<<<<<<< HEAD
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			OpenGLWrapper::getOpenGLWrapper()
					.drawLine(begin.x, begin.y, 0, end.x, end.y, 0, color);
		}
=======
#ifdef WITHGRAPHICS

	void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
		OpenGLWrapper::getOpenGLWrapper()
		.drawLine(begin.x, begin.y, 0, end.x, end.y, 0, OpenGLWrapper::Color::Blue());
	}
>>>>>>> skiesel/master

#endif
};

/**
 * @return True if the segments are coincident or intersecting.
 */
bool checkCollision(LineSegment segment1, LineSegment segment2) {
	PlanarVector intersection;

	LineSegment::IntersectResult intersectResult = segment1.intersect(segment2, intersection);

	// Filter out joint collision for neighbor segments.
	if(segment1.end == segment2.begin || segment2.end == segment1.begin) {
		return intersectResult == LineSegment::IntersectResult::COINCIDENT;
	}

	return intersectResult == LineSegment::IntersectResult::COINCIDENT ||
	       intersectResult == LineSegment::IntersectResult::INTERESECTING;
}
>>>>>>> linkage
}

class Link {
public:
	Link()
<<<<<<< HEAD
			: segment(),
			  angle(0) {
=======
		: segment(),
		  angle(0) {
>>>>>>> linkage
	}

	Planar::PlanarVector updateLineSegment(const Planar::PlanarVector startPoint, const double absAngle) {

		Planar::PlanarVector endPoint;

		endPoint.x = startPoint.x - std::cos(absAngle);
		endPoint.y = startPoint.y + std::sin(absAngle);

		segment.begin = startPoint;
		segment.end = endPoint;

		return endPoint;
	}

	double getAngle() const {
		return angle;
	}

	void setAngle(const double angle) {
		this->angle = angle;
	}

	void addAngle(const double angle) {
		this->angle += angle;
	}

	Planar::LineSegment getSegment() {
		return segment;
	}

#ifdef WITHGRAPHICS

	void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
		segment.draw(color);
	}

#endif

private:
	double angle;
	Planar::LineSegment segment;
};

class PlanarLinkage {
public:
	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	typedef std::vector<std::pair<double, double> > WorkspaceBounds;
	typedef std::vector<std::pair<double, double> > StateVarRanges;

	class State;
<<<<<<< HEAD

	typedef State AbstractState;
=======
	class Edge;

	typedef State AbstractState;
	typedef std::vector<AbstractState> AbstractEdge;
>>>>>>> linkage

	class State {
	public:
		State() : treeStateVars(3) {
			setAngles(treeStateVars);
		}

		State(const State &) = default;

		State(State &&) = default;

		State &operator=(const State &) = default;

		State &operator=(State &&) = default;

		explicit State(StateVars vars) : treeStateVars(std::move(vars)) {
			setAngles(treeStateVars);
		}

<<<<<<< HEAD
		const StateVars &getStateVars() const { return treeStateVars; }

		bool equals(const State &s) const {
			for (unsigned int i = 0; i < getStateVars().size(); ++i) {
				if (fabs(getStateVars()[i] - s.getStateVars()[i]) > 0.000001) {
=======
		const StateVars &getStateVars() const {
			return treeStateVars;
		}

		bool equals(const State &s) const {
			for(unsigned int i = 0; i < getStateVars().size(); ++i) {
				if(fabs(getStateVars()[i] - s.getStateVars()[i]) > 0.000001) {
>>>>>>> linkage
					return false;
				}
			}
			return true;
		}

		void print() const {
			auto stateVars = getStateVars();
<<<<<<< HEAD
			for (auto v : stateVars) {
=======
			for(auto v : stateVars) {
>>>>>>> linkage
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

		void move(Control control) {
			const int numberOfLinks = links.size();
			assert(control.size() == numberOfLinks);

			Planar::PlanarVector currentEnd;
			double absAngle = 0;
<<<<<<< HEAD
			for (int i = 0; i < numberOfLinks; ++i) {
=======
			for(int i = 0; i < numberOfLinks; ++i) {
>>>>>>> linkage
				double angle = control[i];
				links[i].addAngle(angle);

				absAngle += links[i].getAngle();
				currentEnd = links[i].updateLineSegment(currentEnd, absAngle);
			}
		}

		void setAngles(std::vector<double> angles) {
			const int numberOfLinks = links.size();
			assert(angles.size() == numberOfLinks);

			Planar::PlanarVector currentEnd;
			double absAngle = 0;
<<<<<<< HEAD
			for (int i = 0; i < numberOfLinks; ++i) {
=======
			for(int i = 0; i < numberOfLinks; ++i) {
>>>>>>> linkage
				double angle = angles[i];
				links[i].setAngle(angle);

				absAngle += links[i].getAngle();
				currentEnd = links[i].updateLineSegment(currentEnd, absAngle);
			}
		}

		bool checkCollision(Link link1, Link link2) const {
			return Planar::checkCollision(link1.getSegment(), link2.getSegment());
		}

<<<<<<< HEAD
		bool hasCollision() {
			const int size = links.size();

			for (int i = 0; i < size; ++i) {
				for (int j = i + 1; j < size; ++j) {
					if (checkCollision(links[i], links[j])) {
=======
		bool hasCollision() const {
			const int size = links.size();

			for(int i = 0; i < size; ++i) {
				for(int j = i + 1; j < size; ++j) {
					if(checkCollision(links[i], links[j])) {
>>>>>>> linkage
						return true;
					}
				}
			}

			return false;
		}

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
<<<<<<< HEAD
			for (Link link : links) {
				link.draw(color);
			}
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();
=======
<<<<<<< HEAD

			OpenGLWrapper::Color c = OpenGLWrapper::Color(0, 1, 0, 0.01);
			for (Link link : links) {
				link.draw(c);
=======
			for(Link link : links) {
				link.draw(color);
>>>>>>> skiesel/master
			}
>>>>>>> linkage
		}

#endif

<<<<<<< HEAD
		PlanarLinkage::State toAbstractState() const {
			return *this;
		}

		std::vector<State> getTransforms() const {
			return std::vector<State>{*this};
=======
		std::vector<State> getTransforms() const {
			return std::vector<State> {*this};
		}

		static AbstractEdge generateAbstractEdge(const AbstractState &a, const AbstractState &b, const double dt) {
			return interpolate(a,b,dt);
>>>>>>> linkage
		}

		static std::vector<State> interpolate(const State &a, const State &b, const double dt) {
			const int dim = a.getStateVars().size();
			std::vector<State> intermediateStates;

			BOOST_ASSERT(dim == b.getStateVars().size());
			BOOST_ASSERT(dim > 0);

			// Find largest difference
			double maxDifference = 0;
			int maxDifferenceIndex = 0;
<<<<<<< HEAD
			for (int i = 0; i < dim; ++i) {
				double difference = std::abs(a.getStateVars()[i] - b.getStateVars()[i]);
				if (difference > maxDifference) {
=======
			for(int i = 0; i < dim; ++i) {
				double difference = std::abs(a.getStateVars()[i] - b.getStateVars()[i]);
				if(difference > maxDifference) {
>>>>>>> linkage
					maxDifference = difference;
					maxDifferenceIndex = i;
				}
			}

			const int numberOfSteps = maxDifference / dt;

			// If there are no intermediate states
<<<<<<< HEAD
			if (numberOfSteps == 0) {
=======
			if(numberOfSteps == 0) {
>>>>>>> linkage
				return intermediateStates;
			}

			// Calculate step sizes
			std::vector<double> stepSizes(dim);
<<<<<<< HEAD
			for (int i = 0; i < dim; ++i) {
=======
			for(int i = 0; i < dim; ++i) {
>>>>>>> linkage
				double difference = b.getStateVars()[i] - a.getStateVars()[i];
				stepSizes[i] = difference / numberOfSteps;
			}

			// Generate intermediate states;
<<<<<<< HEAD
			for (int i = 0; i < numberOfSteps; ++i) {

				StateVars intermediateStateVars(dim);

				for (int j = 0; j < dim; ++j) {
=======
			for(int i = 0; i < numberOfSteps; ++i) {

				StateVars intermediateStateVars(dim);

				for(int j = 0; j < dim; ++j) {
>>>>>>> linkage
					intermediateStateVars[j] = a.getStateVars()[j] + i * stepSizes[j];
				}

				intermediateStates.emplace_back(std::move(intermediateStateVars));
			}

			return intermediateStates;
		}

		static State getRandomAbstractState(const std::vector<std::pair<double, double> > &bounds) {
			return State();
		}

		static double evaluateDistance(const State &a, const State &b) {
<<<<<<< HEAD
			return 0;
=======
			return 1;
>>>>>>> linkage
		}

		StateVars treeStateVars;
	private:
		std::vector<Link> links = std::vector<Link>(3);
	};

	class Edge {
	public:
		Edge(const State &s) : start(s),
<<<<<<< HEAD
							   end(s),
							   duration(0) {
		}

		Edge(const State start, const State end, double cost, const std::vector<double> control, double duration)
				: start(std::move(start)),
				  end(std::move(end)),
				  cost(cost),
				  treeIndex(0),
				  duration(duration),
				  control(std::move(control)) {
=======
			end(s),
			duration(0) {
		}

		Edge(const State start, const State end, double cost, const std::vector<double> control, double duration)
			: start(std::move(start)),
			  end(std::move(end)),
			  cost(cost),
			  treeIndex(0),
			  duration(duration),
			  control(std::move(control)) {
>>>>>>> linkage
		}

		Edge(const Edge &) = default;

		Edge(Edge &&) = default;

		Edge &operator=(const Edge &) = default;

		Edge &operator=(Edge &&) = default;

<<<<<<< HEAD
=======
		double gCost() const {
			return 0;
		}

>>>>>>> linkage
		void print() {
			start.print();
			end.print();
		}

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
		}

		/* needed for being inserted into NN datastructure */
<<<<<<< HEAD
		const StateVars &getTreeStateVars() const { return end.getStateVars(); }

		int getPointIndex() const { return treeIndex; }

		void setPointIndex(int ptInd) { treeIndex = ptInd; }

		Control getControl() const { return control; }
=======
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
>>>>>>> linkage

		State start, end;
		double cost, duration;
		int treeIndex;
		Edge *parent;
		Control control;
	};

	PlanarLinkage(const InstanceFileMap &args) : workspaceBounds(3) {

<<<<<<< HEAD
		for (int i = 0; i < workspaceBounds.size(); ++i) {
=======
		for(int i = 0; i < workspaceBounds.size(); ++i) {
>>>>>>> linkage
			workspaceBounds[i].first = -M_PI;
			workspaceBounds[i].second = M_PI;
		}

		auto stateVarDomains = getStateVarRanges(workspaceBounds);
<<<<<<< HEAD
		for (auto range : stateVarDomains) {
=======
		for(auto range : stateVarDomains) {
>>>>>>> linkage
			distributions.emplace_back(range.first, range.second);
		}

		boost::char_separator<char> sep(" ");
		boost::tokenizer<boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
<<<<<<< HEAD
		for (auto token : tokens) {
=======
		for(auto token : tokens) {
>>>>>>> linkage
			goalThresholds.push_back(std::stod(token));
		}
	}

	unsigned int getTreeStateSize() const {
		return 3;
	}


	StateVarRanges getStateVarRanges(const WorkspaceBounds &bounds) const {
		return bounds;
	}

	bool isGoal(const State &state, const State &goal) const {
		auto targetStateVars = state.getStateVars();
		auto goalStateVars = goal.getStateVars();
		const int numberOfLinks = targetStateVars.size();

		assert(numberOfLinks == goalStateVars.size());

		double difference = 0;

<<<<<<< HEAD
		for (int i = 0; i < numberOfLinks; ++i) {
			difference = targetStateVars[i] - goalStateVars[i];
			if (std::abs(difference) > goalThresholds[i]) {
=======
		for(int i = 0; i < numberOfLinks; ++i) {
			difference = targetStateVars[i] - goalStateVars[i];
			if(std::abs(difference) > goalThresholds[i]) {
>>>>>>> linkage
				return false;
			}
		}

		return true;
	}

	Edge randomSteer(const State &start, double dt) const {
		const int numberOfLinks = start.getStateVars().size();
		Control controls(numberOfLinks);
		StateVars stateVars(numberOfLinks);

		double sum = 0;
		double max = 0;

<<<<<<< HEAD
		for (int i = 0; i < numberOfLinks; ++i) {
=======
		for(int i = 0; i < numberOfLinks; ++i) {
>>>>>>> linkage
			auto distribution = distributions[i];
			double v = distribution(generator) / 20;
			std::cerr << "  " << start.getStateVars()[i];
			std::cerr << "-> " << v;

<<<<<<< HEAD

=======
>>>>>>> linkage
			sum += v;
			max = std::max(max, v);
			controls[i] = v;
			stateVars[i] = start.getStateVars()[i] + v;
		}

//		stateVars[1] = 0;
//		stateVars[2] = 0;

		std::cerr << " \nSteer ";


		return Edge(start, buildState(stateVars), sum, controls, max);
	}

	Edge steerWithControl(const State &start, const Edge &getControlFromThisEdge, double dt) const {
		return steerWithControl(start, getControlFromThisEdge.getControl(), dt);
	}

	Edge steerWithControl(const State &start, const Control controls, double dt) const {
		const int numberOfLinks = start.getStateVars().size();
		StateVars stateVars(numberOfLinks);

		BOOST_ASSERT(numberOfLinks == controls.size());

		double sum = 0;
		double max = 0;

<<<<<<< HEAD
		for (int i = 0; i < numberOfLinks; ++i) {
=======
		for(int i = 0; i < numberOfLinks; ++i) {
>>>>>>> linkage
			double v = controls[i];
			sum += v;
			max = std::max(max, v);
			stateVars[i] = start.getStateVars()[i] + v;
		}

		return Edge(start, buildState(stateVars), sum, controls, max);
	}

	State buildState(const StateVars &stateVars) const {
		return State(stateVars);
	}

	void drawMesh() {
	}

	void drawMesh(const State &s) const {
	}

	void drawMesh(const fcl::Transform3f &transform, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
	}

	void drawSolution(const fcl::Transform3f &transform, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
	}

	void drawSolution(const std::vector<const Edge *> &solution, double dt = std::numeric_limits<
<<<<<<< HEAD
			double>::infinity()) const {
=======
	                  double>::infinity()) const {
>>>>>>> linkage
	}

	WorkspaceBounds getControlBounds() const {
		return workspaceBounds;
	}

	Control controlFromVector(const Control &controls) const {
		return controls;
	}

<<<<<<< HEAD
=======
	State getRandomStateNear(const State &state, const State &canonicalState, double radius) const {
		return state; //TODO
	}

>>>>>>> linkage
	/*** Workspace interface ***/
	WorkspaceBounds getBounds() const {
		return workspaceBounds;
	}

<<<<<<< HEAD
=======
	State getRandomStateNearAbstractState(const AbstractState &state, double radius) const {

		fprintf(stderr, "PlanarLinkage::getRandomStateNearAbstractState not implemented\n");
		exit(1);

		return State();
	}

	AbstractState toAbstractState(const State &s) const {
			return s;
		}


>>>>>>> linkage
	bool safeEdge(const PlanarLinkage &agent, const Edge &edge, double dt, bool checkSelfCollision = false) const {
		auto intermediateStates = PlanarLinkage::State::interpolate(edge.start, edge.end, dt);
		intermediateStates.push_back(edge.end);
		intermediateStates.push_back(edge.start);

		return safeStates(intermediateStates);
	}

<<<<<<< HEAD
	bool safePoses(const PlanarLinkage &agent, const std::vector<State> &states, const State &state = State()) const {
=======
	inline bool areAbstractEdgesSymmetric() const {
		return true;
	}

	AbstractEdge generateAbstractEdge(const AbstractState &s1, const AbstractState &s2) const {
		AbstractEdge edge;

		fprintf(stderr, "PlanarLinkage::generateAbstractEdge not implemented\n");
		exit(1);

		return edge;
	}

	bool safeAbstractEdge(const PlanarLinkage &agent, const AbstractEdge &edge, double dt) const {
		return safeStates(edge);
	}

	bool safeStates(const PlanarLinkage &agent, const std::vector<State> &states) const {
>>>>>>> linkage
		return safeStates(states);
	}

	bool safeStates(const std::vector<State> &states) const {
<<<<<<< HEAD
		for (auto state : states) {
			if (state.hasCollision()) return false;
=======
		for(auto state : states) {
			if(state.hasCollision()) return false;
>>>>>>> linkage
		}
		return true;
	}

<<<<<<< HEAD
=======
	bool safeState(const PlanarLinkage &pl, const State &state) const {
		return !state.hasCollision();
	}

	bool safeAbstractState(const PlanarLinkage &pl, const AbstractState &state) const {
		return !state.hasCollision();
	}

	bool safeState(const State &state) const {
		return !state.hasCollision();
	}

>>>>>>> linkage
	void draw() const {
	}

private:
	std::vector<double> goalThresholds;
	WorkspaceBounds workspaceBounds;

	std::vector<std::uniform_real_distribution<double> > distributions;
	mutable std::default_random_engine generator;
};



