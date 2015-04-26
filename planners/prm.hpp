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

#include "../utilities/flannkdtreewrapper.hpp"


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
template<class Workspace, class Agent, class Sampler>
class PRM {
public:
    typedef typename Agent::State AgentState;
    typedef typename Agent::Edge AgentEdge;

    struct InternalState {
        typedef boost::vertex_property_tag kind;
    };

    struct InternalEdge {
        typedef boost::edge_property_tag kind;
    };

    struct VertexTotalConnectionAttempts {
        typedef boost::vertex_property_tag kind;
    };

    struct VertexSuccessfulConnectionAttempts {
        typedef boost::vertex_property_tag kind;
    };

    /**
     @brief The underlying roadmap graph.

     @par Any BGL graph representation could be used here. Because we
     expect the roadmap to be sparse (m<n^2), an adjacency_list is more
     appropriate than an adjacency_matrix.

     @par Obviously, a AgentEdge* vertex property is required.
     The incremental connected components algorithm requires
     vertex_predecessor_t and vertex_rank_t properties.
     If boost::vecS is not used for vertex storage, then there must also
     be a boost:vertex_index_t property manually added.

     @par Edges should be undirected and have a weight property.
     */
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                  boost::property<InternalState, AgentState,
                                                  boost::property<VertexTotalConnectionAttempts, unsigned long int,
                                                                  boost::property<VertexSuccessfulConnectionAttempts,
                                                                                  unsigned long int, boost::property<
                                                                                  boost::vertex_predecessor_t,
                                                                                  unsigned long int,
                                                                                  boost::property<boost::vertex_rank_t,
                                                                                                  unsigned long int> > > > >,
                                  boost::property<InternalEdge, AgentEdge *> > Graph;

    /** @brief The type for a vertex in the roadmap. */
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

    /** @brief The type for an edge in the roadmap. */
    typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;

    template<typename Graph>
    class VertexWrapper {
    public:
        VertexWrapper(Vertex vertex, Graph &graph) : vertex(vertex),
                                                     graph(graph) {
        }

        AgentState foo() {
            return AgentState();
        };

        /* needed for being inserted into NN datastructure */
        const typename Agent::StateVars &getStateVars() const {
            return boost::get(InternalState(), graph, vertex).getStateVars();
        }

        int getPointIndex() const {
            return boost::get(InternalState(), graph, vertex).getPointIndex();
        }

        void setPointIndex(int value) {
            boost::get(InternalState(), graph, vertex).setPointIndex(value);
        }

        Vertex getVertex() const {
            return vertex;
        }

//        static void setGraph(Graph &g) {
//            graph = g;
//        }

//        static void setStateProperty(typename boost::property_map<Graph, InternalState>::type &propertyMap) {
//            stateProperty = propertyMap;
//        }

    private:
//        static typename boost::property_map<Graph, InternalState>::type &stateProperty;
        const Vertex vertex;
        Graph &graph;
    };

    typedef flann::KDTreeSingleIndexParams KDTreeType;

    typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, VertexWrapper<Graph>> KDTree;

    template<typename A>
    class T {
    private:
        static int foo;
    };

    PRM(const Workspace &workspace, const Agent &agent, const Sampler &sampler, const InstanceFileMap &args)
            : workspace(workspace),
              agent(agent),
              sampler(sampler),
              nn(KDTree(KDTreeType(), 3)),
              stateProperty(boost::get(InternalState(), graph)),
              totalConnectionAttemptsProperty(boost::get(VertexTotalConnectionAttempts(), graph)),
              successfulConnectionAttemptsProperty(boost::get(VertexSuccessfulConnectionAttempts(), graph)),
              disjointSets(boost::get(boost::vertex_rank, graph), boost::get(boost::vertex_predecessor, graph)),
              internalEdgeProperty(boost::get(InternalEdge(), graph)),
              addedNewSolution_(false),
              iterations_(0) {

        steeringDT = stod(args.value("Steering Delta t"));
        collisionCheckDT = stod(args.value("Collision Check Delta t"));

    }

    ~PRM() {
        graph.clear();
    }

    bool query(const AgentState &start, const AgentState &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

#ifdef WITHGRAPHICS
        // Draw start end goal states
        start.draw();
        goal.draw();
#endif

        fprintf(stdout, "Solving PRM...!\n");


        if (firstInvocation && agent.isGoal(start, goal)) {
            return true;
        }

        // Sample 10k points
        for (int i = 0; i < 10; i++) {


            addMilestone(sampler.sampleConfiguration());
            // TODO Validate sample

        }

        fprintf(stdout, "PRM solved!\n");


//#ifdef WITHGRAPHICS
//        for (const Edge *edge : treeEdges) {
//            edge->draw();
//        }
//#endif

        return false;
    }

    const Graph &getRoadmap() const {
        return graph;
    }

    /** \brief Return the number of milestones currently in the graph */
    unsigned long int milestoneCount() const {
        return boost::num_vertices(graph);
    }

    /** \brief Return the number of edges currently in the graph */
    unsigned long int edgeCount() const {
        return boost::num_edges(graph);
    }

protected:

//    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
//    double distanceFunction(const Vertex a, const Vertex b) const {
//        return si_->distance(stateProperty_[a], stateProperty_[b]);
//    }

    /** \brief Attempt to connect disjoint components in the
    roadmap using random bounding motions (the PRM
    expansion step) */
/*    void expandRoadmap(const base::PlannerTerminationCondition &ptc, std::vector<base::State *> &workStates) {
        // construct a probability distribution over the vertices in the roadmap
        // as indicated in
        //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
        //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars


        PDF <Vertex> pdf;
                foreach(Vertex v, boost::vertices(graph)) {
                        const unsigned long int t = totalConnectionAttemptsProperty[v];
                        pdf.add(v, (double) (t - successfulConnectionAttemptsProperty[v]) / (double) t);
                    }

        if (pdf.empty())
            return;

        while (ptc == false) {
            iterations_++;
            Vertex pdfVertex = pdf.sample(rngraph.uniform01());
            unsigned int s = si_
                    ->randomBounceMotion(simpleSampler_, stateProperty_[pdfVertex], workStates.size(), workStates,
                                         false);
            if (s > 0) {
                s--;
                Vertex last = addMilestone(si_->cloneState(workStates[s]));

                graphMutex_.lock();
                for (unsigned int i = 0; i < s; ++i) {
                    // add the vertex along the bouncing motion
                    Vertex m = boost::add_vertex(graph);
                    stateProperty_[m] = si_->cloneState(workStates[i]);
                    totalConnectionAttemptsProperty[m] = 1;
                    successfulConnectionAttemptsProperty[m] = 0;
                    disjointSets.make_set(m);

                    // add the edge to the parent vertex
                    const base::Cost weight = opt_->motionCost(stateProperty_[pdfVertex], stateProperty_[m]);
                    const Graph::edge_property_type properties(weight);
                    boost::add_edge(pdfVertex, m, properties, graph);
                    uniteComponents(pdfVertex, m);

                    // add the vertex to the nearest neighbors data structure
                    nn_->add(m);
                    pdfVertex = m;
                }

                // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
                // we add an edge
                if (s > 0 || !sameComponent(pdfVertex, last)) {
                    // add the edge to the parent vertex
                    const base::Cost weight = opt_->motionCost(stateProperty_[pdfVertex], stateProperty_[last]);
                    const Graph::edge_property_type properties(weight);
                    boost::add_edge(pdfVertex, last, properties, graph);
                    uniteComponents(pdfVertex, last);
                }
                graphMutex_.unlock();
            }
        }
    } */

//    /** \brief Randomly sample the state space, add and connect milestones
//     in the roadmap. Stop this process when the termination condition
//     \e ptc returns true.  Use \e workState as temporary memory. */
//    void growRoadmap(const unsigned int interations) {
//        /* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
//        for (int i = 0; i < iterations; i++) {
//            iterations_++;
//            // search for a valid state
//            bool found = false;
//            while (!found) {
//                found = sampler.sampleConfiguration();
//            }
//            // add it as a milestone
//            if (found)
//                addMilestone(si_->cloneState(workState));
//        }
//    }

//    /** Thread that checks for solution */
//    void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution) {
//        base::GoalSampleableRegion *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
//        while (!ptc && !addedNewSolution_) {
//            // Check for any new goal states
//            if (goal->maxSampleCount() > goalM_.size()) {
//                const base::State *st = pis_.nextGoal();
//                if (st)
//                    goalM_.push_back(addMilestone(si_->cloneState(st)));
//            }
//
//            // Check for a solution
//            addedNewSolution_ = maybeConstructSolution(startM_, goalM_, solution);
//            // Sleep for 1ms
//            if (!addedNewSolution_)
//                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
//        }
//    }

//    /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, it is constructed in the \e solution argument. */
//    bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<
//            Vertex> &goals, base::PathPtr &solution) {
//        base::Goal *g = pdef_->getGoal().get();
//        base::Cost sol_cost(opt_->infiniteCost());
//                foreach(Vertex
//                                start, starts) {
//                                foreach(Vertex
//                                                goal, goals) {
//                                        // we lock because the connected components algorithm is incremental and may change disjointSets
//                                        graphMutex_.lock();
//                                        bool same_component = sameComponent(start, goal);
//                                        graphMutex_.unlock();
//
//                                        if (same_component &&
//                                            g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start])) {
//                                            base::PathPtr p = constructSolution(start, goal);
//                                            if (p) {
//                                                base::Cost pathCost = p->cost(opt_);
//                                                if (opt_->isCostBetterThan(pathCost, bestCost_))
//                                                    bestCost_ = pathCost;
//                                                // Check if optimization objective is satisfied
//                                                if (opt_->isSatisfied(pathCost)) {
//                                                    solution = p;
//                                                    return true;
//                                                } else if (opt_->isCostBetterThan(pathCost, sol_cost)) {
//                                                    solution = p;
//                                                    sol_cost = pathCost;
//                                                }
//                                            }
//                                        }
//                                    }
//                    }
//
//        return false;
//    }

//    /** \brief Returns the value of the addedNewSolution_ member. */
//    bool addedNewSolution() const {
//        return addedNewSolution_;
//    }

//    /** \brief Function that can solve the motion planning
//    problem. Grows a roadmap using
//    constructRoadmap(). This function can be called
//    multiple times on the same problem, without calling
//    clear() in between. This allows the planner to
//    continue work for more time on an unsolved problem,
//    for example. Start and goal states from the currently
//    specified ProblemDefinition are cached. This means
//    that between calls to solve(), input states are only
//    added, not removed. When using PRM as a multi-query
//    planner, the input states should be however cleared,
//    without clearing the roadmap itself. This can be done
//    using the clearQuery() function. */
//    ompl::base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
//        base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
//
//        if (!goal) {
//            OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
//            return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
//        }
//
//        // Add the valid start states as milestones
//        while (const base::State *st = pis_.nextStart())
//            startM_.push_back(addMilestone(si_->cloneState(st)));
//
//        if (startM_.size() == 0) {
//            OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
//            return base::PlannerStatus::INVALID_START;
//        }
//
//        if (!goal->couldSample()) {
//            OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
//            return base::PlannerStatus::INVALID_GOAL;
//        }
//
//        // Ensure there is at least one valid goal state
//        if (goal->maxSampleCount() > goalM_.size() || goalM_.empty()) {
//            const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
//            if (st)
//                goalM_.push_back(addMilestone(si_->cloneState(st)));
//
//            if (goalM_.empty()) {
//                OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
//                return base::PlannerStatus::INVALID_GOAL;
//            }
//        }
//
//        unsigned long int nrStartStates = boost::num_vertices(graph);
//        OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);
//
//        // Reset addedNewSolution_ member and create solution checking thread
//        addedNewSolution_ = false;
//        base::PathPtr solutionPathPrt;
//        boost::thread slnThread(boost::bind(&PRM::checkForSolution, this, ptc, boost::ref(solutionPathPrt)));
//
//
//        // construct new planner termination condition that fires when the given ptc is true, or a solution is found
//        base::PlannerTerminationCondition ptcOrSolutionFound = base::plannerOrTerminationCondition(ptc,
//                                                                                                   base::PlannerTerminationCondition(
//                                                                                                           boost::bind(
//                                                                                                                   &PRM::addedNewSolution,
//                                                                                                                   this)));
//
//        constructRoadmap(ptcOrSolutionFound);
//
//        // Ensure slnThread is ceased before exiting solve
//        slnThread.join();
//
//        return solutionPathPrt ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
//    }

//    /** \brief While the termination condition allows, this function will construct the roadmap (using growRoadmap() and expandRoadmap(),
//    maintaining a 2:1 ratio for growing/expansion of roadmap) */
//    void constructRoadmap() {
//        std::vector<base::State *> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
//        si_->allocStates(xstates);
//        bool grow = true;
//
//        bestCost_ = opt_->infiniteCost();
//        while (ptc() == false) {
//            // maintain a 2:1 ratio for growing/expansion of roadmap
//            // call growRoadmap() twice as long for every call of expandRoadmap()
//            if (grow)
//                growRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(
//                        2.0 * magic::ROADMAP_BUILD_TIME)), xstates[0]);
//            else
//                expandRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(
//                        magic::ROADMAP_BUILD_TIME)), xstates);
//            grow = !grow;
//        }
//
//        si_->freeStates(xstates);
//    }

    /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data structure
    and then connect it to the roadmap in accordance to the connection strategy. */
    Vertex addMilestone(AgentState sourceState) {
//        boost::mutex::scoped_lock _(graphMutex_);

        // Create a new vertex in the graph
        Vertex sourceVertex = boost::add_vertex(graph);

        // Set the internal state to the target state
        stateProperty[sourceVertex] = sourceState;
        totalConnectionAttemptsProperty[sourceVertex] = 1;
        successfulConnectionAttemptsProperty[sourceVertex] = 0;

        // Initialize to its own (dis)connected component.
        disjointSets.make_set(sourceVertex);

        auto *sourceWrapper = vertexWrapperPool.construct(sourceVertex, graph);

        // Which milestones will we attempt to connect to?
        typename KDTree::KNNResult near = nn.kNearest(sourceWrapper, 10);

        sourceState.draw();

        const std::vector<VertexWrapper<Graph> *> neighbors = near.elements;
                foreach(VertexWrapper<Graph> *neighbor, neighbors) {

                        auto targetVertex = neighbor->getVertex();

                        totalConnectionAttemptsProperty[sourceVertex]++;
                        totalConnectionAttemptsProperty[targetVertex]++;

                        auto edge = agent.steer(sourceState, stateProperty[targetVertex], steeringDT);

                        // Validate edge
                        if (workspace.safeEdge(agent, edge, collisionCheckDT)) {

                            edge.draw();

                            // Increment the successful connection attempts on both ends 
                            successfulConnectionAttemptsProperty[sourceVertex]++;
                            successfulConnectionAttemptsProperty[targetVertex]++;

                            auto *targetEdge = agentEdgePool.construct(edge.start, edge.end, edge.cost);
                            typename Graph::edge_property_type properties(targetEdge);

                            boost::add_edge(targetVertex, sourceVertex, properties, graph);
                            uniteComponents(targetVertex, sourceVertex);
                        }
                    }


        nn.insertPoint(sourceWrapper);

        return sourceVertex;
    }

    /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
    inline void uniteComponents(Vertex m1, Vertex m2) {
        disjointSets.union_set(m1, m2);
    }

    /** \brief Check if two milestones (\e m1 and \e m2) are part of the same connected component. This is not a const function since we use incremental connected components from boost */
    inline bool sameComponent(Vertex m1, Vertex m2) {
        return boost::same_component(m1, m2, disjointSets);
    }

//    /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
//    ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal) {
//        boost::mutex::scoped_lock _(graphMutex_);
//        boost::vector_property_map<Vertex> prev(boost::num_vertices(graph));
//
//        try {
//            // Consider using a persistent distance_map if it's slow
//            boost::astar_search(graph, start, boost::bind(&PRM::costHeuristic, this, _1, goal),
//                                boost::predecessor_map(prev).distance_compare(
//                                                boost::bind(&base::OptimizationObjective::isCostBetterThan, opt_.get(),
//                                                            _1, _2)).distance_combine(
//                                                boost::bind(&base::OptimizationObjective::combineCosts, opt_.get(), _1,
//                                                            _2)).distance_inf(opt_->infiniteCost())
//                                        .distance_zero(opt_->identityCost()).visitor(AStarGoalVisitor<Vertex>(goal)));
//        } catch (AStarFoundGoal &) {
//        }
//
//        if (prev[goal] == goal)
//            throw Exception(name_, "Could not find solution path");
//
//        PathGeometric *p = new PathGeometric(si_);
//        for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
//            p->append(stateProperty_[pos]);
//        p->append(stateProperty_[start]);
//        p->reverse();
//
//        return base::PathPtr(p);
//    }

//    /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
//    This method wraps OptimizationObjective::motionCostHeuristic */
//    ompl::base::Cost costHeuristic(Vertex u, Vertex v) const {
//        return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
//    }

    ///////////////////////////////////////
    // Planner progress property functions
    std::string getIterationCount() const {
        return boost::lexical_cast<std::string>(iterations_);
    }

    std::string getMilestoneCountString() const {
        return boost::lexical_cast<std::string>(milestoneCount());
    }

    std::string getEdgeCountString() const {
        return boost::lexical_cast<std::string>(edgeCount());
    }

    /** \brief Connectivity graph */
    Graph graph;

    /** \brief Access to the internal Agent::State at each Vertex */
    typename boost::property_map<Graph, InternalState>::type stateProperty;

    /** \brief Access to the number of total connection attempts for a vertex */
    typename boost::property_map<Graph, VertexTotalConnectionAttempts>::type totalConnectionAttemptsProperty;

    /** \brief Access to the number of successful connection attempts for a vertex */
    typename boost::property_map<Graph, VertexSuccessfulConnectionAttempts>::type successfulConnectionAttemptsProperty;

    /** \brief Access to the internal Agent::Edge at each Edge */
    typename boost::property_map<Graph, InternalEdge>::type internalEdgeProperty;

    /** \brief Data structure that maintains the connected components */
    typename boost::disjoint_sets<typename boost::property_map<Graph, boost::vertex_rank_t>::type,
                                  typename boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets;

    /** \brief Random number generator */
//    RNG rngraph;

    /** \brief A flag indicating that a solution has been added during solve() */
    bool addedNewSolution_;

    /** \brief Mutex to guard access to the Graph member (graph) */
    mutable boost::mutex graphMutex_;

    //////////////////////////////
    // Planner progress properties
    /** \brief Number of iterations the algorithm performed */
    unsigned long int iterations_;

    /** \brief Best cost found so far by algorithm */
//    base::Cost bestCost_; // TODO replace

    // Motion Planning Toolkit //

    const Workspace &workspace;
    const Agent &agent;
    const Sampler &sampler;
    KDTree nn;
    boost::object_pool<AgentEdge> agentEdgePool;
    boost::object_pool<VertexWrapper<Graph>> vertexWrapperPool;

    std::vector<const Edge *> treeEdges;

    double steeringDT, collisionCheckDT;
};