#pragma once

#include <vector>
#include <math.h>
#include <random>

namespace Planar {
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
}

class Link {
public:
	Link()
			: segment(),
			  angle(0) {
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

	class State {
	public:
		State() : stateVars(3),
				  links(3) {
		}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()),
								links(3) {
			setAngles(stateVars);
		}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()),
									   links(3) {
			setAngles(stateVars);
		}

		const StateVars &getStateVars() const { return stateVars; }

		bool equals(const State &s) const {
			for (unsigned int i = 0; i < stateVars.size(); ++i) {
				if (fabs(stateVars[i] - s.stateVars[i]) > 0.000001) {
					return false;
				}
			}
			return true;
		}

		void print() const {
			for (auto v : stateVars) {
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

		bool move(Control control) {
			const int numberOfLinks = links.size();
			assert(control.size() == numberOfLinks);

			Planar::PlanarVector currentEnd;
			double absAngle = 0;
			for (int i = 0; i < numberOfLinks; ++i) {
				double angle = control[i];
				links[i].addAngle(angle);

				absAngle += links[i].getAngle();
				currentEnd = links[i].updateLineSegment(currentEnd, absAngle);
			}

			return true;
			return hasCollision();
		}

		bool setAngles(std::vector<double> angles) {
			const int numberOfLinks = links.size();
			assert(angles.size() == numberOfLinks);

			Planar::PlanarVector currentEnd;
			double absAngle = 0;
			for (int i = 0; i < numberOfLinks; ++i) {
				double angle = angles[i];
				links[i].setAngle(angle);

				absAngle += links[i].getAngle();
				currentEnd = links[i].updateLineSegment(currentEnd, absAngle);
			}

			bool collision = hasCollision(); //
			fprintf(stderr, "%s\n", collision ? "collision" : "safe"); //

			return collision;
		}

		bool checkCollision(Link link1, Link link2) const {
			return Planar::checkCollision(link1.getSegment(), link2.getSegment());
		}

		bool hasCollision() {
			const int size = links.size();

			for (int i = 0; i < size; ++i) {
				for (int j = i + 1; j < size; ++j) {
					if (checkCollision(links[i], links[j])) {
						return true;
					}
				}
			}

			return false;
		}

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			for (Link link : links) {
				link.draw(color);
			}
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();
		}

#endif


		fcl::Transform3f getTransform() const {
			return fcl::Transform3f();
		}

	private:
		StateVars stateVars;
		std::vector<Link> links;
	};

	class Edge {
	public:
		Edge(const State &s) : start(s),
							   end(s),
							   duration(0) {
		}

		Edge(const State &start, const State &end, double cost, const std::vector<double> &controls, double duration)
				: start(start),
				  end(end),
				  cost(cost),
				  treeIndex(0),
				  duration(duration) {
		}

		Edge(const Edge &e) : start(e.start),
							  end(e.end),
							  cost(e.cost),
							  treeIndex(e.treeIndex),
							  duration(e.duration) {
		}

		Edge &operator=(const Edge &e) {
			start = e.start;
			end = e.end;
			cost = e.cost;
			duration = e.duration;
			treeIndex = e.treeIndex;
			return *this;
		}

		void print() {
			start.print();
			end.print();
		}

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
		}

		/* needed for being inserted into NN datastructure */
		const StateVars &getTreeStateVars() const { return end.getStateVars(); }

		int getPointIndex() const { return treeIndex; }

		void setPointIndex(int ptInd) { treeIndex = ptInd; }

		State start, end;
		double cost, duration;
		int treeIndex;
		Edge *parent;

	};

	PlanarLinkage(const InstanceFileMap &args) : workspaceBounds(3) {

		for (int i = 0; i < workspaceBounds.size(); ++i) {
			workspaceBounds[i].first = -M_PI;
			workspaceBounds[i].second = M_PI;
		}

		auto stateVarDomains = getStateVarRanges(workspaceBounds);
		for (auto range : stateVarDomains) {
			distributions.emplace_back(range.first, range.second);
		}

		boost::char_separator<char> sep(" ");
		boost::tokenizer<boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for (auto token : tokens) {
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

		for (int i = 0; i < numberOfLinks; ++i) {
			difference = targetStateVars[i] - goalStateVars[i];
			if (std::abs(difference) > goalThresholds[i]) {
				return false;
			}
		}

		return true;
	}

	Edge randomSteer(const State &start, double dt) const {
		const int numberOfLinks = start.getStateVars().size();
		std::vector<double> controls(numberOfLinks);
		std::vector<double> stateVars(numberOfLinks);

		double sum = 0;
		double max = 0;

		for (int i = 0; i < numberOfLinks; ++i) {
			auto distribution = distributions[i];
			double v = distribution(generator);
			sum += v;
			max = std::max(max, v);
			controls[i] = v;
			stateVars[i] = start.getStateVars()[i] + v;
		}

		return Edge(start, buildState(stateVars), sum, controls, max);
	}

	Edge steerWithControl(const State &start, const Edge &getControlsFromThisEdge, double dt) const {
		assert(false);
	}

	Edge steerWithControl(const State &start, const std::vector<double> controls, double dt) const {
		assert(false);
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
			double>::infinity()) const {
	}

	WorkspaceBounds getControlBounds() const {
		return workspaceBounds;
	}

	Control controlFromVector(const std::vector<double> &controls) const {
		return controls;
	}

	/*** Workspace interface ***/
	WorkspaceBounds getBounds() const {
		return workspaceBounds;
	}

	bool safeEdge(const PlanarLinkage &agent, const Edge &edge, double dt, bool checkSelfCollision = false) const {

	}

	bool safePose(const PlanarLinkage &agent, const fcl::Transform3f &pose, const State &state = State()) const {

	}

	bool safePoses(const PlanarLinkage &agent, const std::vector<
			fcl::Transform3f> &poses, const State &state = State()) const {

	}

	void draw() const {
	}

private:
	std::vector<double> goalThresholds;
	WorkspaceBounds workspaceBounds;

	std::vector<std::uniform_real_distribution<double> > distributions;
	mutable std::default_random_engine generator;
};



