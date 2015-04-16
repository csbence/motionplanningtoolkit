//#pragma once
//
//#include <flann/flann.hpp>
//#include <boost/pool/object_pool.hpp>
//
//template<class Workspace, class Agent, class Sampler, class NN>
//class PRM {
//public:
//
//    typedef typename Agent::State State;
//    typedef typename Agent::Edge Edge;
//
//    RRT(const Workspace &workspace, const Agent &agent, const Sampler &sampler, NN &nn, const InstanceFileMap &args) :
//            workspace(workspace), agent(agent), sampler(sampler), nn(nn) {
//        steeringDT = stod(args.value("Steering Delta t"));
//        collisionCheckDT = stod(args.value("Collision Check Delta t"));
//    }
//
//
//    bool query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {
//
//#ifdef WITHGRAPHICS
//        start.draw();
//        goal.draw();
//#endif
//
//        if (firstInvocation) {
//            if (agent.isGoal(start, goal)) {
//                return true;
//            }
//
//            auto root = pool.construct(start, start, 0);
//
//            nn.insertPoint(root);
//        }
//
//        unsigned int iterations = 0;
//
//        while (true) {
//            // Sample 10k points
//            for (int i = 0; i < 10000; i++) {
//                auto sample = sampler.sampleConfiguration();
//
//                // TODO Validate sample
//
//
//
//
//            }
//
//#ifdef WITHGRAPHICS
//            sample.draw();
//#endif
//
//            //intentionally not in the pool
//            auto sampleEdge = Edge(sample, sample, 0);
//
//            // Get the k nearest neighbors
//            typename NN::KNNResult result = nn.kNearest(&sampleEdge, 10);
//
//            State nearest = result.elements[0]->end;
//
//            auto edge = agent.steer(nearest, sample);
//            //auto edge = agent.randomSteer(nearest, steeringDT);
//
//            if (!workspace.safeEdge(agent, edge, collisionCheckDT)) {
//                continue;
//            }
//
//            if (agent.isGoal(edge.end, goal)) {
//                return true;
//            }
//
//            Edge *e = pool.construct(edge.start, edge.end, edge.cost);
//            nn.insertPoint(e);
//
//#ifdef WITHGRAPHICS
//            // treeEdges.push_back(e);
//#endif
//
//            if (iterationsAtATime > 0 && ++iterations > iterationsAtATime) break;
//        }
//
//#ifdef WITHGRAPHICS
//        for (const Edge *edge : treeEdges) {
//            edge->draw();
//        }
//#endif
//
//        return false;
//    }
//
//private:
//    const Workspace &workspace;
//    const Agent &agent;
//    const Sampler &sampler;
//    NN &nn;
//    boost::object_pool<Edge> pool;
//    std::vector<const Edge *> treeEdges;
//
//    double steeringDT, collisionCheckDT;
//};