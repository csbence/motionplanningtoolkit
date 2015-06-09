#ifndef MOTIONPLANNING_ANYTIMEHYBRIDSEARCH_HPP
#define MOTIONPLANNING_ANYTIMEHYBRIDSEARCH_HPP

#include "../utilities/ProbabilisticSampler.hpp"
#include "../utilities/Clock.hpp"
#include "prm.hpp"
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <random>
#include <iostream>
#include <map>
#include <set>


template<class Workspace, class Agent, class Sampler>
class AnytimeHybridSearch {
public:

    typedef PRM<Workspace, Agent, Sampler> Prm;
    typedef typename Prm::Vertex Region;

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
                                                  boost::property<boost::vertex_predecessor_t, unsigned long int,
                                                                  boost::property<boost::vertex_rank_t,
                                                                                  unsigned long int>>>,
                                  boost::property<InternalEdge, AgentEdge *,
                                                  boost::property<boost::edge_weight_t, double> > > Graph;

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

        /* needed for being inserted into NN datastructure */
        const typename Agent::StateVars &getTreeStateVars() const {
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

    private:
        const Vertex vertex;
        Graph &graph;
    };

    typedef flann::KDTreeSingleIndexParams KDTreeType;

    typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, VertexWrapper<Graph>> KDTree;

    typedef std::chrono::duration<int, std::milli> Millisecond;

    template<typename key, typename value> using TreeGroupMap = std::unordered_map<key, std::vector<value>>;

    AnytimeHybridSearch(const Workspace &workspace, const Agent &agent, const Sampler &sampler, const InstanceFileMap &args)
            : workspace(workspace),
              agent(agent),
              sampler(sampler),
              nn(KDTree(KDTreeType(), 3)),
              stateProperty(boost::get(InternalState(), graph)),
              edgeProperty(boost::get(InternalEdge(), graph)),
              edgeWeightProperty(boost::get(boost::edge_weight, graph)),
              solutionFound(false),
              solutionCost(-1),
              poseNumber(-1),
              prm(workspace, agent, sampler, args),
              openRegions(),
              openRegionSet(),
              treeGroupMap(),
              distribution(0, 1),
              clock() {

        steeringDT = stod(args.value("Steering Delta t"));
        collisionCheckDT = stod(args.value("Collision Check Delta t"));
    }

    void expandMotionTree(const AgentState &goal) {
        for (int i = 0; i < 100; i++) {
            // Sample source region
            Region region = openRegions.sample(distribution(generator));

            // Get the vertexes in the sampled region
            std::__1::vector<Vertex> verticesInRegion = treeGroupMap.find(region)->second;

            // Select a random state in the region
            std::__1::uniform_int_distribution<> intDistribution(0, verticesInRegion.size() - 1);
            Vertex &sourceVertex = verticesInRegion[intDistribution(generator)];

            auto edge = agent.randomSteer(stateProperty[sourceVertex], steeringDT);

            if (!workspace.safeEdge(agent, edge, collisionCheckDT)) {
                continue;
            }

            if (agent.isGoal(edge.end, goal)) {
                fprintf(stdout, "AHS solution found!");
                solutionFound = true;
                break;
            }

            // Expand tree
            AgentEdge *targetEdge = agentEdgePool.construct(edge);
            AgentState targetState = edge.end;
            Vertex targetVertex = addState(targetState);

            // Add new edge to the tree
            typename Graph::edge_property_type properties(targetEdge, edge.cost);
            auto edgePair = add_edge(targetVertex, sourceVertex, properties, graph);

            const Region targetRegion = prm.getContainingRegion(targetState);
            const double cost = prm.getRegionCost(targetRegion);

            if (openRegionSet.find(targetRegion) == openRegionSet.end()) {
                // Region not found

                openRegionSet.insert(targetRegion);
                openRegions.add(targetRegion, 1.0 / cost);
            }

            assignVertexToRegion(targetRegion, targetVertex);
        }
    }

    bool query(const AgentState &start, const AgentState &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

#ifdef WITHGRAPHICS
        drawCurrentState();
#endif

        if (solutionFound) {
            return true;
        }

        if (firstInvocation) {
            startVertex = addState(start);
            goalVertex = addState(goal);
        }

        // First create the PRM
        if (!prm.query(start, goal, iterationsAtATime, firstInvocation)) {
            return false;
        }

        clock.start();

        // Initialize tree
        if (openRegions.empty()) {

            // Add start to the tree and the corresponding group to the open group list
            const Region startRegion = prm.getContainingRegion(start);
            const double cost = prm.getRegionCost(startRegion);

            openRegions.add(startRegion, 1.0 / cost);
            openRegionSet.insert(startRegion);
            assignVertexToRegion(startRegion, startVertex);
        }

        expandMotionTree(goal);

        clock.stop();

        if (solutionFound) {
            fprintf(stdout, "Guided RRT solved in %d[ms]\n", clock.getDurationInMillis());
        }

        return solutionFound;
    }

    void drawCurrentState() {
        // Draw
        for (auto vertex : boost::make_iterator_range(boost::vertices(graph))) {
            const AgentState agentState = stateProperty(vertex);
            agentState.draw();
        }

        for (auto edge : boost::make_iterator_range(boost::edges(graph))) {
            const AgentEdge *const agentEdge = edgeProperty(edge);
            agentEdge->draw();
        }
    }

    Vertex addState(const AgentState &state) {

        // Create a new vertex in the graph
        Vertex vertex = boost::add_vertex(graph);
        stateProperty[vertex] = state;

        return vertex;
    }

    void assignVertexToRegion(const Region region, const Vertex vertex) {
        auto verticesInRegionIterator = treeGroupMap.find(region);

        if (verticesInRegionIterator == treeGroupMap.end()) {
            treeGroupMap.emplace(region, std::vector<Vertex>{vertex});
        } else {
            verticesInRegionIterator->second.push_back(vertex);
        }
    }

private:

    /** \brief Connectivity graph */
    Graph graph;

    /** \brief Access to the internal Agent::State at each Vertex */
    typename boost::property_map<Graph, InternalState>::type stateProperty;

    /** \brief Access to the internal Agent::Edge at each Edge */
    typename boost::property_map<Graph, InternalEdge>::type edgeProperty;

    typename boost::property_map<Graph, boost::edge_weight_t>::type edgeWeightProperty;

    // Motion Planning Toolkit //
    const Workspace &workspace;
    const Agent &agent;
    const Sampler &sampler;
    KDTree nn;
    boost::object_pool<AgentEdge> agentEdgePool;
    boost::object_pool<VertexWrapper<Graph>> vertexWrapperPool;

    std::vector<const AgentEdge *> solution;
    double solutionCost;
    int poseNumber;

//    std::vector<const Edge *> treeEdges;
    Vertex startVertex;
    Vertex goalVertex;
    bool solutionFound;

    double steeringDT, collisionCheckDT;

    /**
     * Probabilistic roadmap.
     * Used as a heuristic to guide the sampling.
     */
    Prm prm;

    /**
     * Probability distribution of the open regions.
     */
    PDF<Region> openRegions;

    /**
     * Set of open regions for fast region lookup.
     * The probability distribution does not support this feature.
     */
    std::set<Region> openRegionSet;

    TreeGroupMap<Region, Vertex> treeGroupMap;

    std::uniform_real_distribution<> distribution;
    std::default_random_engine generator;

    Clock clock;

};


#endif //MOTIONPLANNING_ANYTIMEHYBRIDSEARCH_HPP
