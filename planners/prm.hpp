#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <utility>
#include <vector>
#include <map>

#include "../utilities/flannkdtreewrapper.hpp"

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

    private:
//        static typename boost::property_map<Graph, InternalState>::type &stateProperty;
        const Vertex vertex;
        Graph &graph;
    };

    typedef flann::KDTreeSingleIndexParams KDTreeType;

    typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, VertexWrapper<Graph>> KDTree;

    typedef std::chrono::duration<int, std::milli> Millisecond;

    PRM(const Workspace &workspace, const Agent &agent, const Sampler &sampler, const InstanceFileMap &args)
            : workspace(workspace),
              agent(agent),
              sampler(sampler),
              nn(KDTree(KDTreeType(), 3)),
              stateProperty(boost::get(InternalState(), graph)),
              totalConnectionAttemptsProperty(boost::get(VertexTotalConnectionAttempts(), graph)),
              successfulConnectionAttemptsProperty(boost::get(VertexSuccessfulConnectionAttempts(), graph)),
              disjointSets(boost::get(boost::vertex_rank, graph), boost::get(boost::vertex_predecessor, graph)),
              edgeProperty(boost::get(InternalEdge(), graph)),
              edgeWeightProperty(boost::get(boost::edge_weight, graph)),
              solutionFound(false),
              solutionCost(-1),
              poseNumber(-1) {

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

        for (auto vertex : boost::make_iterator_range(boost::vertices(graph))) {
            const AgentState agentState = stateProperty(vertex);
            agentState.draw();
        }

        for (auto edge : boost::make_iterator_range(boost::edges(graph))) {
            const AgentEdge *const agentEdge = edgeProperty(edge);
            agentEdge->draw();
        }
#endif

        if (firstInvocation && agent.isGoal(start, goal)) {
            return true;
        }

        std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();

        if (firstInvocation) {
            startVertex = addMilestone(start);
            goalVertex = addMilestone(goal);
        }

        if (solution.size() > 0) {
            auto red = OpenGLWrapper::Color::Red();
            for (const AgentEdge *edge : solution) {
                edge->draw(red);
            }

            // agent.drawSolution(solution);
            if (poseNumber >= solution.size() * 2) poseNumber = -1;
            if (poseNumber >= 0)
                agent.animateSolution(solution, poseNumber++);

            return false;
        }

        if (solutionFound) {
//            fprintf(stdout, "Solution found. \n");

            boost::vector_property_map<Vertex> predecessorMap(boost::num_vertices(graph));
            boost::vector_property_map<double> distanceMap(boost::num_vertices(graph));

            boost::dijkstra_shortest_paths(graph, goalVertex,
                                           boost::predecessor_map(predecessorMap).weight_map(edgeWeightProperty)
                                                   .distance_map(distanceMap));

            fprintf(stdout, "Dijkstra completed. \n");


            // Construct the shorthest path from the start to the goal.
            Vertex currentVertex = startVertex;

            int counter = 0;
            solutionCost = 0;
            solution.clear();

            while (currentVertex != goalVertex) {
                auto nextVertex = predecessorMap[currentVertex];
                auto edgePair = boost::edge(currentVertex, nextVertex, graph);

                if (!edgePair.second) {
                    fprintf(stderr, "Non existent edge is in the solution. \n");
                    return false;
                }

                const Edge edge = edgePair.first;

                AgentEdge *agentEdge = edgeProperty[edge];
                solutionCost += edgeWeightProperty[edge];

                solution.push_back(agentEdge);

                currentVertex = nextVertex;

                counter++;
            }


            fprintf(stdout, "Steps: %d \n", counter);

            return false;
        } else {

            // Sample 10k points
            for (int i = 0; i < 10; i++) {
                addMilestone(sampler.sampleConfiguration());
                // TODO Validate sample

            }

            solutionFound = sameComponent(startVertex, goalVertex);
        }

        std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();

        Millisecond duration(std::chrono::duration_cast<Millisecond>(endTime - startTime));

        fprintf(stdout, "PRM solved[%s] in %d[ms]\n", solutionFound ? "true" : "false", duration.count());

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


    /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data structure
    and then connect it to the roadmap in accordance to the connection strategy. */
    Vertex addMilestone(AgentState sourceState) {
//        boost::mutex::scoped_lock _(graphMutex_);

        // Create a new vertex in the graph
        Vertex sourceVertex = boost::add_vertex(graph);

        // Set the internal state of the source vertex to the source state
        stateProperty[sourceVertex] = sourceState;
        totalConnectionAttemptsProperty[sourceVertex] = 1;
        successfulConnectionAttemptsProperty[sourceVertex] = 0;

        // Initialize to its own (dis)connected component.
        disjointSets.make_set(sourceVertex);

        auto *sourceWrapper = vertexWrapperPool.construct(sourceVertex, graph);

        // Which milestones will we attempt to connect to?
        typename KDTree::KNNResult near = nn.kNearest(sourceWrapper, 10);

        sourceState.draw();

        const std::vector<VertexWrapper<Graph> *> &neighbors = near.elements;

        for (const VertexWrapper<Graph> *neighborWrapper : neighbors) {

            auto targetVertex = neighborWrapper->getVertex();

            totalConnectionAttemptsProperty[sourceVertex]++;
            totalConnectionAttemptsProperty[targetVertex]++;

            auto edge = agent.steer(sourceState, stateProperty[targetVertex], 1000);

            // Validate edge
            if (workspace.safeEdge(agent, edge, collisionCheckDT)) {

                edge.draw();

                // Increment the successful connection attempts on both ends
                successfulConnectionAttemptsProperty[sourceVertex]++;
                successfulConnectionAttemptsProperty[targetVertex]++;

                auto *targetEdge = agentEdgePool.construct(edge.start, edge.end, edge.cost);
                typename Graph::edge_property_type properties(targetEdge, edge.cost);

                auto edgePair = boost::add_edge(targetVertex, sourceVertex, properties, graph);
                uniteComponents(targetVertex, sourceVertex);

                fprintf(stdout, "Edge: %f - %f \n", edge.cost, edgeWeightProperty[edgePair.first]);
            };
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

    /** \brief Connectivity graph */
    Graph graph;

    /** \brief Access to the internal Agent::State at each Vertex */
    typename boost::property_map<Graph, InternalState>::type stateProperty;

    /** \brief Access to the number of total connection attempts for a vertex */
    typename boost::property_map<Graph, VertexTotalConnectionAttempts>::type totalConnectionAttemptsProperty;

    /** \brief Access to the number of successful connection attempts for a vertex */
    typename boost::property_map<Graph, VertexSuccessfulConnectionAttempts>::type successfulConnectionAttemptsProperty;

    /** \brief Access to the internal Agent::Edge at each Edge */
    typename boost::property_map<Graph, InternalEdge>::type edgeProperty;

    typename boost::property_map<Graph, boost::edge_weight_t>::type edgeWeightProperty;

    /** \brief Data structure that maintains the connected components */
    typename boost::disjoint_sets<typename boost::property_map<Graph, boost::vertex_rank_t>::type,
                                  typename boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets;

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


    std::vector<const Edge *> treeEdges;
    Vertex startVertex;
    Vertex goalVertex;
    bool solutionFound;

    double steeringDT, collisionCheckDT;
};