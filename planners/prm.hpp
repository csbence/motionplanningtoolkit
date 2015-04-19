#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <utility>
#include <vector>
#include <map>


#define foreach BOOST_FOREACH


/**
   @anchor gPRM
   @par Short description
   PRM is a planner that constructs a roadmap of milestones
   that approximate the connectivity of the state space. The
   milestones are valid states in the state space. Near-by
   milestones are connected by valid motions. Finding a motion
   plan that connects two given states is reduced to a
   discrete search (this implementation uses A*) in the
   roadmap.
   @par External documentation
   L.E. Kavraki, P.Švestka, J.-C. Latombe, and M.H. Overmars,
   Probabilistic roadmaps for path planning in high-dimensional configuration spaces,
   <em>IEEE Trans. on Robotics and Automation</em>, vol. 12, pp. 566–580, Aug. 1996.
   DOI: [10.1109/70.508439](http://dx.doi.org/10.1109/70.508439)<br>
   [[PDF]](http://ieeexplore.ieee.org/ielx4/70/11078/00508439.pdf?tp=&arnumber=508439&isnumber=11078)
   [[more]](http://www.kavrakilab.org/robotics/prm.html)
*/

/** \brief Probabilistic RoadMap planner */
template<class Workspace, class Agent, class Sampler, class NN>
class PRM {
public:
    typedef typename Agent::State State;
    typedef typename Agent::Edge Edge;

    RRT(const Workspace &workspace, const Agent &agent, const Sampler &sampler, NN &nn, const InstanceFileMap &args)
            : workspace(workspace), agent(agent), sampler(sampler), nn(nn) {
        steeringDT = stod(args.value("Steering Delta t"));
        collisionCheckDT = stod(args.value("Collision Check Delta t"));
    }


    bool query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

#ifdef WITHGRAPHICS
        // Draw start end goal states
        start.draw();
        goal.draw();
#endif

        if (firstInvocation) {
            if (agent.isGoal(start, goal)) {
                return true;
            }

            auto root = pool.construct(start, start, 0);

            nn.insertPoint(root);
        }

        unsigned int iterations = 0;

        while (true) {
            // Sample 10k points
            for (int i = 0; i < 10000; i++) {
                auto sample = sampler.sampleConfiguration();

                // TODO Validate sample
            }

#ifdef WITHGRAPHICS
            sample.draw();
#endif

            //intentionally not in the pool
            auto sampleEdge = Edge(sample, sample, 0);

            // Get the k nearest neighbors
            typename NN::KNNResult result = nn.kNearest(&sampleEdge, 10);

            State nearest = result.elements[0]->end;

            auto edge = agent.steer(nearest, sample);
            //auto edge = agent.randomSteer(nearest, steeringDT);

            if (!workspace.safeEdge(agent, edge, collisionCheckDT)) {
                continue;
            }

            if (agent.isGoal(edge.end, goal)) {
                return true;
            }

            Edge *e = pool.construct(edge.start, edge.end, edge.cost);
            nn.insertPoint(e);

#ifdef WITHGRAPHICS
            // treeEdges.push_back(e);
#endif

            if (iterationsAtATime > 0 && ++iterations > iterationsAtATime) break;
        }

#ifdef WITHGRAPHICS
        for (const Edge *edge : treeEdges) {
            edge->draw();
        }
#endif

        return false;
    }

    struct vertex_state_t {
        typedef boost::vertex_property_tag kind;
    };

    struct vertex_total_connection_attempts_t {
        typedef boost::vertex_property_tag kind;
    };

    struct vertex_successful_connection_attempts_t {
        typedef boost::vertex_property_tag kind;
    };

    /**
     @brief The underlying roadmap graph.

     @par Any BGL graph representation could be used here. Because we
     expect the roadmap to be sparse (m<n^2), an adjacency_list is more
     appropriate than an adjacency_matrix.

     @par Obviously, a ompl::base::State* vertex property is required.
     The incremental connected components algorithm requires
     vertex_predecessor_t and vertex_rank_t properties.
     If boost::vecS is not used for vertex storage, then there must also
     be a boost:vertex_index_t property manually added.

     @par Edges should be undirected and have a weight property.
     */
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::property<vertex_state_t, base::State *, boost::property<vertex_total_connection_attempts_t, unsigned long int, boost::property<vertex_successful_connection_attempts_t, unsigned long int, boost::property<boost::vertex_predecessor_t, unsigned long int, boost::property<boost::vertex_rank_t, unsigned long int> > > > >, boost::property<boost::edge_weight_t, base::Cost> > Graph;

    /** @brief The type for a vertex in the roadmap. */
    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

    /** @brief The type for an edge in the roadmap. */
    typedef boost::graph_traits<Graph>::edge_descriptor Edge;

    /** @brief A nearest neighbors data structure for roadmap vertices. */
    typedef boost::shared_ptr<NearestNeighbors < Vertex>> RoadmapNeighbors;

    /** @brief A function returning the milestones that should be
     * attempted to connect to. */
    typedef boost::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

    /** @brief A function that can reject connections.

     This is called after previous connections from the neighbor list
     have been added to the roadmap.
     */
    typedef boost::function<bool(const Vertex &, const Vertex &)> ConnectionFilter;

    /** \brief Set the connection strategy function that specifies the
     milestones that connection attempts will be make to for a
     given milestone.

     \par The behavior and performance of PRM can be changed drastically
     by varying the number and properties if the milestones that are
     connected to each other.

     \param pdef A function that takes a milestone as an argument and
     returns a collection of other milestones to which a connection
     attempt must be made. The default connection strategy is to connect
     a milestone's 10 closest neighbors.
     */
    void setConnectionStrategy(const ConnectionStrategy &connectionStrategy) {
        connectionStrategy_ = connectionStrategy;
        userSetConnectionStrategy_ = true;
    }

    /** \brief Set the function that can reject a milestone connection.

     \par The given function is called immediately before a connection
     is checked for collision and added to the roadmap. Other neighbors
     may have already been connected before this function is called.
     This allows certain heuristics that use the structure of the
     roadmap (like connected components or useful cycles) to be
     implemented by changing this function.

     \param connectionFilter A function that takes the new milestone,
     a neighboring milestone and returns whether a connection should be
     attempted.
     */
    void setConnectionFilter(const ConnectionFilter &connectionFilter) {
        connectionFilter_ = connectionFilter;
    }

    /** \brief Set a different nearest neighbors datastructure */
    template<template<typename T> class NN>
    void setNearestNeighbors() {
        nn_.reset(new NN<Vertex>());
        if (!userSetConnectionStrategy_)
            connectionStrategy_.clear();
        if (isSetup())
            setup();
    }

    const Graph &getRoadmap() const {
        return g_;
    }

    /** \brief Return the number of milestones currently in the graph */
    unsigned long int milestoneCount() const {
        return boost::num_vertices(g_);
    }

    /** \brief Return the number of edges currently in the graph */
    unsigned long int edgeCount() const {
        return boost::num_edges(g_);
    }

    const RoadmapNeighbors &getNearestNeighbors() {
        return nn_;
    }

protected:

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    double distanceFunction(const Vertex a, const Vertex b) const {
        return si_->distance(stateProperty_[a], stateProperty_[b]);
    }

    /** \brief Constructor */
    PRM(bool starStrategy) : base::Planner(si, "PRM"), starStrategy_(starStrategy),
                             stateProperty_(boost::get(vertex_state_t(), g_)),
                             totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_)),
                             successfulConnectionAttemptsProperty_(
                                     boost::get(vertex_successful_connection_attempts_t(), g_)),
                             weightProperty_(boost::get(boost::edge_weight, g_)),
                             disjointSets_(boost::get(boost::vertex_rank, g_),
                                           boost::get(boost::vertex_predecessor, g_)),
                             userSetConnectionStrategy_(false), addedNewSolution_(false), iterations_(0),
                             bestCost_(std::numeric_limits<double>::quiet_NaN()) {
        specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
        specs_.approximateSolutions = false;
        specs_.optimizingPaths = true;

        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &PRM::setMaxNearestNeighbors,
                                            std::string("8:1000"));

        addPlannerProgressProperty("iterations INTEGER", boost::bind(&PRM::getIterationCount, this));
        addPlannerProgressProperty("best cost REAL", boost::bind(&PRM::getBestCost, this));
        addPlannerProgressProperty("milestone count INTEGER", boost::bind(&PRM::getMilestoneCountString, this));
        addPlannerProgressProperty("edge count INTEGER", boost::bind(&PRM::getEdgeCountString, this));
    }

    ~PRM() {
        freeMemory();
    }

    void setup() {
        Planner::setup();
        if (!nn_) {
            nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(si_->getStateSpace()));
            nn_->setDistanceFunction(boost::bind(&PRM::distanceFunction, this, _1, _2));
        }
        if (!connectionStrategy_) {
            if (starStrategy_)
                connectionStrategy_ = KStarStrategy<Vertex>(boost::bind(&PRM::milestoneCount, this), nn_,
                                                            si_->getStateDimension());
            else
                connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
        }
        if (!connectionFilter_)
            connectionFilter_ = boost::lambda::constant(true);

        // Setup optimization objective
        //
        // If no optimization objective was specified, then default to
        // optimizing path length as computed by the distance() function
        // in the state space.
        if (pdef_) {
            if (pdef_->hasOptimizationObjective())
                opt_ = pdef_->getOptimizationObjective();
            else {
                opt_.reset(new base::PathLengthOptimizationObjective(si_));
                if (!starStrategy_)
                    opt_->setCostThreshold(opt_->infiniteCost());
            }
        } else {
            OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
            setup_ = false;
        }
    }

    /** \brief Convenience function that sets the connection strategy to the
    default one with k nearest neighbors.
    */
    void setMaxNearestNeighbors(unsigned int k) {
        if (starStrategy_)
            throw Exception("Cannot set the maximum nearest neighbors for " + getName());
        if (!nn_) {
            nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(si_->getStateSpace()));
            nn_->setDistanceFunction(boost::bind(&PRM::distanceFunction, this, _1, _2));
        }
        if (!userSetConnectionStrategy_)
            connectionStrategy_.clear();
        if (isSetup())
            setup();
    }

    void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) {
        Planner::setProblemDefinition(pdef);
        clearQuery();
    }

    /** \brief Clear the query previously loaded from the ProblemDefinition.
    Subsequent calls to solve() will reuse the previously computed roadmap,
    but will clear the set of input states constructed by the previous call to solve().
    This enables multi-query functionality for PRM. */
    void clearQuery() {
        startM_.clear();
        goalM_.clear();
        pis_.restart();
    }

    void clear() {
        Planner::clear();
        sampler_.reset();
        simpleSampler_.reset();
        freeMemory();
        if (nn_)
            nn_->clear();
        clearQuery();

        iterations_ = 0;
        bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    }

    /** \brief Free all the memory allocated by the planner */
    void freeMemory() {
                foreach(Vertex
                                v, boost::vertices(g_))si_->freeState(stateProperty_[v]);
        g_.clear();
    }


    /** \brief Attempt to connect disjoint components in the roadmap
    using random bouncing motions (the PRM expansion step) for the
    given time (seconds). */
    void expandRoadmap(double expandTime) {
        expandRoadmap(base::timedPlannerTerminationCondition(expandTime));
    }

    /** \brief Attempt to connect disjoint components in the roadmap
    using random bouncing motions (the PRM expansion step) until the
    given condition evaluates true. */
    void expandRoadmap(const base::PlannerTerminationCondition &ptc) {
        if (!simpleSampler_)
            simpleSampler_ = si_->allocStateSampler();

        std::vector<base::State *> states(magic::MAX_RANDOM_BOUNCE_STEPS);
        si_->allocStates(states);
        expandRoadmap(ptc, states);
        si_->freeStates(states);
    }

    /** \brief Attempt to connect disjoint components in the
    roadmap using random bounding motions (the PRM
    expansion step) */
    void expandRoadmap(const base::PlannerTerminationCondition &ptc, std::vector<base::State *> &workStates) {
        // construct a probability distribution over the vertices in the roadmap
        // as indicated in
        //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
        //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

        PDF <Vertex> pdf;
                foreach(Vertex
                                v, boost::vertices(g_)) {
                        const unsigned long int t = totalConnectionAttemptsProperty_[v];
                        pdf.add(v, (double) (t - successfulConnectionAttemptsProperty_[v]) / (double) t);
                    }

        if (pdf.empty())
            return;

        while (ptc == false) {
            iterations_++;
            Vertex v = pdf.sample(rng_.uniform01());
            unsigned int s = si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates,
                                                     false);
            if (s > 0) {
                s--;
                Vertex last = addMilestone(si_->cloneState(workStates[s]));

                graphMutex_.lock();
                for (unsigned int i = 0; i < s; ++i) {
                    // add the vertex along the bouncing motion
                    Vertex m = boost::add_vertex(g_);
                    stateProperty_[m] = si_->cloneState(workStates[i]);
                    totalConnectionAttemptsProperty_[m] = 1;
                    successfulConnectionAttemptsProperty_[m] = 0;
                    disjointSets_.make_set(m);

                    // add the edge to the parent vertex
                    const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[m]);
                    const Graph::edge_property_type properties(weight);
                    boost::add_edge(v, m, properties, g_);
                    uniteComponents(v, m);

                    // add the vertex to the nearest neighbors data structure
                    nn_->add(m);
                    v = m;
                }

                // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
                // we add an edge
                if (s > 0 || !sameComponent(v, last)) {
                    // add the edge to the parent vertex
                    const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[last]);
                    const Graph::edge_property_type properties(weight);
                    boost::add_edge(v, last, properties, g_);
                    uniteComponents(v, last);
                }
                graphMutex_.unlock();
            }
        }
    }

    /** \brief If the user desires, the roadmap can be
    improved for the given time (seconds). The solve()
    method will also improve the roadmap, as needed.*/
    void growRoadmap(double growTime) {
        growRoadmap(base::timedPlannerTerminationCondition(growTime));
    }

    /** \brief If the user desires, the roadmap can be
    improved until a given condition is true. The solve()
    method will also improve the roadmap, as needed.*/
    void growRoadmap(const base::PlannerTerminationCondition &ptc) {
        if (!isSetup())
            setup();
        if (!sampler_)
            sampler_ = si_->allocValidStateSampler();

        base::State *workState = si_->allocState();
        growRoadmap(ptc, workState);
        si_->freeState(workState);
    }

    /** \brief Randomly sample the state space, add and connect milestones
     in the roadmap. Stop this process when the termination condition
     \e ptc returns true.  Use \e workState as temporary memory. */
    void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState) {
        /* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
        while (ptc == false) {
            iterations_++;
            // search for a valid state
            bool found = false;
            while (!found && ptc == false) {
                unsigned int attempts = 0;
                do {
                    found = sampler_->sample(workState);
                    attempts++;
                } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
            }
            // add it as a milestone
            if (found)
                addMilestone(si_->cloneState(workState));
        }
    }

    /** Thread that checks for solution */
    void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution) {
        base::GoalSampleableRegion *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
        while (!ptc && !addedNewSolution_) {
            // Check for any new goal states
            if (goal->maxSampleCount() > goalM_.size()) {
                const base::State *st = pis_.nextGoal();
                if (st)
                    goalM_.push_back(addMilestone(si_->cloneState(st)));
            }

            // Check for a solution
            addedNewSolution_ = maybeConstructSolution(startM_, goalM_, solution);
            // Sleep for 1ms
            if (!addedNewSolution_)
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }

    /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, it is constructed in the \e solution argument. */
    bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution) {
        base::Goal *g = pdef_->getGoal().get();
        base::Cost sol_cost(opt_->infiniteCost());
                foreach(Vertex
                                start, starts) {
                                foreach(Vertex
                                                goal, goals) {
                                        // we lock because the connected components algorithm is incremental and may change disjointSets_
                                        graphMutex_.lock();
                                        bool same_component = sameComponent(start, goal);
                                        graphMutex_.unlock();

                                        if (same_component &&
                                            g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start])) {
                                            base::PathPtr p = constructSolution(start, goal);
                                            if (p) {
                                                base::Cost pathCost = p->cost(opt_);
                                                if (opt_->isCostBetterThan(pathCost, bestCost_))
                                                    bestCost_ = pathCost;
                                                // Check if optimization objective is satisfied
                                                if (opt_->isSatisfied(pathCost)) {
                                                    solution = p;
                                                    return true;
                                                } else if (opt_->isCostBetterThan(pathCost, sol_cost)) {
                                                    solution = p;
                                                    sol_cost = pathCost;
                                                }
                                            }
                                        }
                                    }
                    }

        return false;
    }

    /** \brief Returns the value of the addedNewSolution_ member. */
    bool addedNewSolution() const {
        return addedNewSolution_;
    }

    /** \brief Function that can solve the motion planning
    problem. Grows a roadmap using
    constructRoadmap(). This function can be called
    multiple times on the same problem, without calling
    clear() in between. This allows the planner to
    continue work for more time on an unsolved problem,
    for example. Start and goal states from the currently
    specified ProblemDefinition are cached. This means
    that between calls to solve(), input states are only
    added, not removed. When using PRM as a multi-query
    planner, the input states should be however cleared,
    without clearing the roadmap itself. This can be done
    using the clearQuery() function. */
    ompl::base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
        checkValidity();
        base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

        if (!goal) {
            OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
            return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
        }

        // Add the valid start states as milestones
        while (const base::State *st = pis_.nextStart())
            startM_.push_back(addMilestone(si_->cloneState(st)));

        if (startM_.size() == 0) {
            OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
            return base::PlannerStatus::INVALID_START;
        }

        if (!goal->couldSample()) {
            OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }

        // Ensure there is at least one valid goal state
        if (goal->maxSampleCount() > goalM_.size() || goalM_.empty()) {
            const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
                goalM_.push_back(addMilestone(si_->cloneState(st)));

            if (goalM_.empty()) {
                OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
                return base::PlannerStatus::INVALID_GOAL;
            }
        }

        unsigned long int nrStartStates = boost::num_vertices(g_);
        OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

        // Reset addedNewSolution_ member and create solution checking thread
        addedNewSolution_ = false;
        base::PathPtr sol;
        boost::thread slnThread(boost::bind(&PRM::checkForSolution, this, ptc, boost::ref(sol)));

        // construct new planner termination condition that fires when the given ptc is true, or a solution is found
        base::PlannerTerminationCondition ptcOrSolutionFound = base::plannerOrTerminationCondition(ptc,
                                                                                                   base::PlannerTerminationCondition(
                                                                                                           boost::bind(
                                                                                                                   &PRM::addedNewSolution,
                                                                                                                   this)));

        constructRoadmap(ptcOrSolutionFound);

        // Ensure slnThread is ceased before exiting solve
        slnThread.join();

        OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

        if (sol) {
            base::PlannerSolution psol(sol);
            psol.setPlannerName(getName());
            // if the solution was optimized, we mark it as such
            psol.setOptimized(opt_, bestCost_, addedNewSolution());
            pdef_->addSolutionPath(psol);
        }

        return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
    }

    /** \brief While the termination condition allows, this function will construct the roadmap (using growRoadmap() and expandRoadmap(),
    maintaining a 2:1 ratio for growing/expansion of roadmap) */
    void constructRoadmap(const base::PlannerTerminationCondition &ptc) {
        if (!isSetup())
            setup();
        if (!sampler_)
            sampler_ = si_->allocValidStateSampler();
        if (!simpleSampler_)
            simpleSampler_ = si_->allocStateSampler();

        std::vector<base::State *> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
        si_->allocStates(xstates);
        bool grow = true;

        bestCost_ = opt_->infiniteCost();
        while (ptc() == false) {
            // maintain a 2:1 ratio for growing/expansion of roadmap
            // call growRoadmap() twice as long for every call of expandRoadmap()
            if (grow)
                growRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(
                        2.0 * magic::ROADMAP_BUILD_TIME)), xstates[0]);
            else
                expandRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(
                        magic::ROADMAP_BUILD_TIME)), xstates);
            grow = !grow;
        }

        si_->freeStates(xstates);
    }

    /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data structure
    and then connect it to the roadmap in accordance to the connection strategy. */
    Vertex addMilestone(base::State *state) {
        boost::mutex::scoped_lock _(graphMutex_);

        Vertex m = boost::add_vertex(g_);
        stateProperty_[m] = state;
        totalConnectionAttemptsProperty_[m] = 1;
        successfulConnectionAttemptsProperty_[m] = 0;

        // Initialize to its own (dis)connected component.
        disjointSets_.make_set(m);

        // Which milestones will we attempt to connect to?
        const std::vector<Vertex> &neighbors = connectionStrategy_(m);

                foreach(Vertex n, neighbors)if (connectionFilter_(n, m)) {
                            totalConnectionAttemptsProperty_[m]++;
                            totalConnectionAttemptsProperty_[n]++;
                            if (si_->checkMotion(stateProperty_[n], stateProperty_[m])) {
                                successfulConnectionAttemptsProperty_[m]++;
                                successfulConnectionAttemptsProperty_[n]++;
                                const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
                                const Graph::edge_property_type properties(weight);
                                boost::add_edge(n, m, properties, g_);
                                uniteComponents(n, m);
                            }
                        }

        nn_->add(m);

        return m;
    }

    /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
    void uniteComponents(Vertex m1, Vertex m2) {
        disjointSets_.union_set(m1, m2);
    }

    /** \brief Check if two milestones (\e m1 and \e m2) are part of the same connected component. This is not a const function since we use incremental connected components from boost */
    bool sameComponent(Vertex m1, Vertex m2) {
        return boost::same_component(m1, m2, disjointSets_);
    }

    /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
    ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal) {
        boost::mutex::scoped_lock _(graphMutex_);
        boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

        try {
            // Consider using a persistent distance_map if it's slow
            boost::astar_search(g_, start, boost::bind(&PRM::costHeuristic, this, _1, goal),
                                boost::predecessor_map(prev).distance_compare(
                                        boost::bind(&base::OptimizationObjective::isCostBetterThan, opt_.get(), _1,
                                                    _2)).distance_combine(
                                        boost::bind(&base::OptimizationObjective::combineCosts, opt_.get(), _1,
                                                    _2)).distance_inf(opt_->infiniteCost()).distance_zero(
                                        opt_->identityCost()).visitor(AStarGoalVisitor<Vertex>(goal)));
        } catch (AStarFoundGoal &) {
        }

        if (prev[goal] == goal)
            throw Exception(name_, "Could not find solution path");

        PathGeometric *p = new PathGeometric(si_);
        for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
            p->append(stateProperty_[pos]);
        p->append(stateProperty_[start]);
        p->reverse();

        return base::PathPtr(p);
    }

    void getPlannerData(base::PlannerData &data) const {
        Planner::getPlannerData(data);

        // Explicitly add start and goal states:
        for (size_t i = 0; i < startM_.size(); ++i)
            data.addStartVertex(base::PlannerDataVertex(stateProperty_[startM_[i]],
                                                        const_cast<PRM *>(this)->disjointSets_.find_set(startM_[i])));

        for (size_t i = 0; i < goalM_.size(); ++i)
            data.addGoalVertex(base::PlannerDataVertex(stateProperty_[goalM_[i]],
                                                       const_cast<PRM *>(this)->disjointSets_.find_set(goalM_[i])));

        // Adding edges and all other vertices simultaneously
                foreach(const Edge e, boost::edges(g_)) {
                        const Vertex v1 = boost::source(e, g_);
                        const Vertex v2 = boost::target(e, g_);
                        data.addEdge(base::PlannerDataVertex(stateProperty_[v1]),
                                     base::PlannerDataVertex(stateProperty_[v2]));

                        // Add the reverse edge, since we're constructing an undirected roadmap
                        data.addEdge(base::PlannerDataVertex(stateProperty_[v2]),
                                     base::PlannerDataVertex(stateProperty_[v1]));

                        // Add tags for the newly added vertices
                        data.tagState(stateProperty_[v1], const_cast<PRM *>(this)->disjointSets_.find_set(v1));
                        data.tagState(stateProperty_[v2], const_cast<PRM *>(this)->disjointSets_.find_set(v2));
                    }
    }

    /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
    This method wraps OptimizationObjective::motionCostHeuristic */
    ompl::base::Cost costHeuristic(Vertex u, Vertex v) const {
        return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
    }

    ///////////////////////////////////////
    // Planner progress property functions
    std::string getIterationCount() const {
        return boost::lexical_cast<std::string>(iterations_);
    }

    std::string getBestCost() const {
        return boost::lexical_cast<std::string>(bestCost_);
    }

    std::string getMilestoneCountString() const {
        return boost::lexical_cast<std::string>(milestoneCount());
    }

    std::string getEdgeCountString() const {
        return boost::lexical_cast<std::string>(edgeCount());
    }

    /** \brief Flag indicating whether the default connection strategy is the Star strategy */
    bool starStrategy_;

    /** \brief Sampler user for generating valid samples in the state space */
    base::ValidStateSamplerPtr sampler_; // TODO replace

    /** \brief Sampler user for generating random in the state space */
    base::StateSamplerPtr simpleSampler_; // TODO replace

    /** \brief Nearest neighbors data structure */
    RoadmapNeighbors nn_; // TODO replace

    /** \brief Connectivity graph */
    Graph g_;

    /** \brief Array of start milestones */
    std::vector<Vertex> startM_;

    /** \brief Array of goal milestones */
    std::vector<Vertex> goalM_;

    /** \brief Access to the internal base::state at each Vertex */
    boost::property_map<Graph, vertex_state_t>::type stateProperty_;

    /** \brief Access to the number of total connection attempts for a vertex */
    boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;

    /** \brief Access to the number of successful connection attempts for a vertex */
    boost::property_map<Graph, vertex_successful_connection_attempts_t>::type successfulConnectionAttemptsProperty_;

    /** \brief Access to the weights of each Edge */
    boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

    /** \brief Data structure that maintains the connected components */
    boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type, boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets_;

    /** \brief Function that returns the milestones to attempt connections with */
    ConnectionStrategy connectionStrategy_;

    /** \brief Function that can reject a milestone connection */
    ConnectionFilter connectionFilter_;

    /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are assumed) */
    bool userSetConnectionStrategy_;

    /** \brief Random number generator */
    RNG rng_;

    /** \brief A flag indicating that a solution has been added during solve() */
    bool addedNewSolution_;

    /** \brief Mutex to guard access to the Graph member (g_) */
    mutable boost::mutex graphMutex_;

    /** \brief Objective cost function for PRM graph edges */
    base::OptimizationObjectivePtr opt_; // TODO replace

    //////////////////////////////
    // Planner progress properties
    /** \brief Number of iterations the algorithm performed */
    unsigned long int iterations_;
    /** \brief Best cost found so far by algorithm */
    base::Cost bestCost_; // TODO replace

    // Motion Planning Toolkit //

    const Workspace &workspace;
    const Agent &agent;
    const Sampler &sampler;
    NN &nn;
    boost::object_pool<Edge> pool;
    std::vector<const Edge *> treeEdges;

    double steeringDT, collisionCheckDT;
};
