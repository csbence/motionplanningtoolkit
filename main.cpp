#ifdef WITHGRAPHICS

#include "utilities/openglwrapper.hpp"

#endif

#include "workspaces/map3d.hpp"

#include "agents/omnidirectional.hpp"
#include "agents/dubins.hpp"
#include "agents/snake_trailers.hpp"
#include "agents/blimp.hpp"

#include "planners/rrt.hpp"
#include "planners/prm.hpp"
#include "planners/AnytimeHybridSearch.hpp"

#include "samplers/uniformsampler.hpp"
#include "samplers/normalsampler.hpp"
#include "samplers/fbiasedsampler.hpp"

#include "discretizations/workspace/griddiscretization.hpp"

#include "utilities/flannkdtreewrapper.hpp"
#include "utilities/instancefilemap.hpp"

<<<<<<< HEAD
#include "bullet_interface/bullet_raycast_vehicle.hpp"

typedef Omnidirectional Agent;
//typedef Dubins Agent;
typedef Map3D<Agent> Workspace;
typedef GridDiscretization<Workspace, Agent> Discretization;
typedef UniformSampler<Workspace, Agent> Sampler;
//typedef FBiasedSampler<Workspace, Agent, Discretization> Sampler;

typedef flann::KDTreeSingleIndexParams KDTreeType;
typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

typedef PRM<Workspace, Agent, Sampler> PrmPlanner;
typedef AnytimeHybridSearch<Workspace, Agent, Sampler> HybridPlanner;

=======
>>>>>>> master
std::vector<double> parseDoubles(const std::string &str) {
    std::vector<double> values;
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tokens(str, sep);
    for (auto token : tokens) {
        values.push_back(std::stod(token));
    }
    return values;
}

<<<<<<< HEAD
int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "no instance file provided!\n");
        exit(1);
    }

    InstanceFileMap args(argv[1]);

    Agent agent(args);
    Workspace workspace(args);
=======

void omnidirectional(const InstanceFileMap& args) {
	typedef Omnidirectional Agent;
	typedef Map3D<Agent> Workspace;
	typedef GridDiscretization<Workspace, Agent> Discretization;
	typedef UniformSampler<Workspace, Agent> Sampler;
	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;
>>>>>>> master

    Agent::State start(parseDoubles(args.value("Agent Start Location")));
    auto goalVars = parseDoubles(args.value("Agent Goal Location"));
    Agent::State goal(goalVars);

    std::vector<double> discretizations(3, 0.1);
    Discretization discretization(workspace, agent, discretizations);

<<<<<<< HEAD
//	Sampler sampler(workspace, agent, discretization, start, goal, 4);

    Sampler sampler(workspace, agent);

    KDTreeType kdtreeType;
    KDTree kdtree(kdtreeType, 3);
//
    Planner planner(workspace, agent, sampler, kdtree, args);

//    PrmPlanner planner(workspace, agent, sampler, args);
//    HybridPlanner planner(workspace, agent, sampler, args);
=======
	Sampler sampler(workspace, agent);

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

	Planner planner(workspace, agent, sampler, kdtree, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		unsigned int i = 0; 
		auto lambda = [&](){
			agent.draw();
			workspace.draw();
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
	#endif
}

void dubins(const InstanceFileMap& args) {
	typedef Dubins Agent;
	typedef Map3D<Agent> Workspace;
	typedef GridDiscretization<Workspace, Agent> Discretization;
	typedef UniformSampler<Workspace, Agent> Sampler;
	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(parseDoubles(args.value("Agent Start Location")));
	auto goalVars = parseDoubles(args.value("Agent Goal Location"));
	Agent::State goal(goalVars);

	Sampler sampler(workspace, agent);

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());
>>>>>>> master

    bool firstInvocation = true;

<<<<<<< HEAD
#ifdef WITHGRAPHICS
    auto lambda = [&]() {
        planner.query(start, goal, 100, firstInvocation);
        firstInvocation = false;
        agent.draw();
        workspace.draw();
    };
    OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
#else
		while(!planner.query(start, goal, 100, firstInvocation));
=======
	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		unsigned int i = 0; 
		auto lambda = [&](){
			agent.draw();
			workspace.draw();
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
	#endif
}

void snake(const InstanceFileMap& args) {
	typedef SnakeTrailers Agent;
	typedef Map3D<Agent> Workspace;
	typedef GridDiscretization<Workspace, Agent> Discretization;
	typedef UniformSampler<Workspace, Agent> Sampler;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(parseDoubles(args.value("Agent Start Location")));
	auto goalVars = parseDoubles(args.value("Agent Goal Location"));
	Agent::State goal(goalVars);

	Sampler sampler(workspace, agent);

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

	Planner planner(workspace, agent, sampler, kdtree, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		unsigned int i = 0; 
		auto lambda = [&](){
			// agent.draw();
			// workspace.draw();
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
	#endif
}

void blimp(const InstanceFileMap& args) {
	typedef Blimp Agent;
	typedef Map3D<Agent> Workspace;
	typedef GridDiscretization<Workspace, Agent> Discretization;
	typedef UniformSampler<Workspace, Agent> Sampler;
	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(parseDoubles(args.value("Agent Start Location")));
	auto goalVars = parseDoubles(args.value("Agent Goal Location"));
	Agent::State goal(goalVars);

	Sampler sampler(workspace, agent);

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

	Planner planner(workspace, agent, sampler, kdtree, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		unsigned int i = 0; 
		auto lambda = [&](){
			agent.draw();
			workspace.draw();
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
>>>>>>> master
	#endif
}

int main(int argc, char *argv[]) {
	if(argc < 2) {
		fprintf(stderr, "no instance file provided!\n");
		exit(1);
	}

	InstanceFileMap args(argv[1]);

	if(args.value("Agent Type").compare("Omnidirectional") == 0)
		omnidirectional(args);
	else if(args.value("Agent Type").compare("Dubins") == 0)
		dubins(args);
	else if(args.value("Agent Type").compare("Snake") == 0)
		snake(args);
	else if(args.value("Agent Type").compare("Blimp") == 0)
		blimp(args);
	else
		fprintf(stderr, "unrecognized Agent Type\n");

    return 0;
}