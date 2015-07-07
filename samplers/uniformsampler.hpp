#pragma once

#include <random>

template <class Workspace, class Agent, class NN>
class UniformSampler {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

public:
	UniformSampler(const Workspace &workspace, const Agent &agent, NN& nn) : workspace(workspace), agent(agent), nn(nn) {
		stateVarDomains = agent.getStateVarRanges(workspace.getBounds());
		for(auto range : stateVarDomains) {
			distributions.emplace_back(range.first, range.second);
		}
	}

	State getTreeSample() const {
		auto sample = sampleConfiguration();
		sample.draw();

		auto sampleEdge = Edge(sample);
		typename NN::KNNResult result = nn.nearest(&sampleEdge, 0, 1);
		return result.elements[0]->end;
	}

	Edge* getTreeEdge(const State& s) const {
		auto edge = Edge(s);
		typename NN::KNNResult result = nn.nearest(&edge);
		return result.elements[0];	
	}

private:
	State sampleConfiguration() const {
		StateVars vars;
		for(auto distribution : distributions) {
			double v = distribution(generator);
			vars.push_back(v);
		}
		return agent.buildState(vars);
	}

	const Workspace &workspace;
	const Agent &agent;
	NN &nn;
	StateVarRanges stateVarDomains;

	std::vector< std::uniform_real_distribution<double> > distributions;
	mutable std::default_random_engine generator;
};