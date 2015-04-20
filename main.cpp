#ifdef WITHGRAPHICS
	#include "utilities/openglwrapper.hpp"
#endif

#include "workspaces/map3d.hpp"

#include "agents/omnidirectional.hpp"

#include "planners/rrt.hpp"
#include "planners/prm.hpp"

#include "samplers/uniformsampler.hpp"
#include "samplers/normalsampler.hpp"
#include "samplers/fbiasedsampler.hpp"

#include "discretizations/workspace/griddiscretization.hpp"

#include "utilities/flannkdtreewrapper.hpp"
#include "utilities/instancefilemap.hpp"

typedef Omnidirectional Agent;
typedef Map3D<Omnidirectional> Workspace;
typedef GridDiscretization<Workspace> Discretization;
typedef UniformSampler<Workspace, Agent> Sampler;
// typedef FBiasedSampler<Workspace, Agent, Discretization> Sampler;

typedef flann::KDTreeSingleIndexParams KDTreeType;
typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::State> KDTreeState;
typedef PRM<Workspace, Agent, Sampler, KDTreeState> PrmPlanner;

std::vector<double> parseDoubles(const std::string &str) {
	std::vector<double> values;
	boost::char_separator<char> sep(" ");
	boost::tokenizer< boost::char_separator<char> > tokens(str, sep);
	for(auto token : tokens) {
		values.push_back(std::stod(token));
	}
	return values;
}

int main(int argc, char *argv[]) {
	if(argc < 2) {
		fprintf(stderr, "no instance file provided!\n");
		exit(1);
	}

	InstanceFileMap args(argv[1]);

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(parseDoubles(args.value("Agent Start Location")));
	auto goalVars = parseDoubles(args.value("Agent Goal Location"));
	Agent::State goal(goalVars);

	// std::vector<double> discretizations(3, 2./3.);
	// Discretization discretization(workspace, discretizations);

	// Sampler sampler(workspace, agent, discretization, start, goal);

	Sampler sampler(workspace, agent);	

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, 3);

	Planner planner(workspace, agent, sampler, kdtree, args);

	KDTreeState kdtreePrm(kdtreeType, 3);
	PrmPlanner prmPlanner(workspace, agent, sampler, kdtreePrm, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		bool foundGoal = false;
		auto lambda = [&](){
			if(!foundGoal)
				foundGoal = prmPlanner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
	#endif

	return 0;
}