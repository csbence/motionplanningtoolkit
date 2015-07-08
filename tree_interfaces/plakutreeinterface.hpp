#pragma once

#include <limits>
#include <unordered_set>
#include "../utilities/flannkdtreewrapper.hpp"
#include "../utilities/heap.hpp"

template <class Workspace, class Agent, class Discretization>
class PlakuTreeInterface {

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> UniformSampler;

	struct Region {
		Region(unsigned int id, const Agent& agent) : heapIndex(std::numeric_limits<unsigned int>::max()), id(id), numSelections(0), heuristic(std::numeric_limits<double>::infinity()), weight(0), onOpen(false){
			KDTreeType kdtreeType;
			edgesInRegion = new KDTree(kdtreeType, agent.getTreeStateSize());
		}

		~Region() {
			delete edgesInRegion;
		}

		void setHeuristicAndPath(double heur, const std::vector<unsigned int> &path) {
			heuristic = heur;
			regionPath = path;
			regionPath.push_back(id);
		}

		void selected(double alpha) {
			numSelections++;
			weight = pow(alpha, numSelections) / (std::numeric_limits<double>::epsilon() + heuristic);
		}

		unsigned int getRandomRegionAlongPathToGoal(std::uniform_real_distribution<double> &distribution) const {
			unsigned int randomIndex = (unsigned int)(distribution(GlobalRandomGenerator) * regionPath.size());
			return regionPath[randomIndex];
		}

		State getNearestStateInRegion(const State& s) const {
			Edge e(s);
			auto res = edgesInRegion->nearest(&e, 0, 1);
			assert(res.elements.size() > 0);
			return res.elements[0]->end;
		}

		/* used for initial heuristic computation */
		unsigned int heapIndex;
		int sort(const Region* r) const { return (heuristic - r->heuristic) > 0 ? -1 : 1; }
		unsigned int getHeapIndex() const { return heapIndex; }
		void setHeapIndex(unsigned int i) { heapIndex = i; }
		void updateRegionPath(const std::vector<unsigned int> &rp) { regionPath = rp; }

		static bool HeapCompare(const Region *r1, const Region *r2) {
			return r1->weight < r2->weight;
		}

		unsigned int id, numSelections;
		double heuristic, weight;
		std::vector<unsigned int> regionPath;
		bool onOpen;
		KDTree *edgesInRegion;
	};

public:
	PlakuTreeInterface(const Workspace &workspace, const Agent &agent, Discretization& discretization,
		const State& start, const State& goal, double alpha, double b, double stateRadius) : agent(agent), workspace(workspace),
		discretization(discretization), startRegionId(discretization.getCellId(start)),
		goalRegionId(discretization.getCellId(goal)), activeRegion(NULL), alpha(alpha), b(b), stateRadius(stateRadius) {

		assert(alpha > 0 && alpha < 1);

		KDTreeType kdtreeType;
		uniformSamplerBackingKDTree = new KDTree(kdtreeType, agent.getTreeStateSize());

		uniformSampler = new UniformSampler(workspace, agent, *uniformSamplerBackingKDTree);

		unsigned int regionCount = discretization.getCellCount();
		regions.reserve(regionCount);
		for(unsigned int i = 0; i < regionCount; ++i) {
			regions.push_back(new Region(i, agent));
		}

		clock_t startT = clock();

		dijkstra(regions[goalRegionId]);

		double endT = clock();

		dfpair(stdout, "dijkstra search time", "%g", (endT-startT) / CLOCKS_PER_SEC);

		bool connected = false;
		while(!connected) {
			connected = true;
			for(const auto region : regions) {
				if(region->regionPath.size() == 0) {
					connected = false;
					discretization.grow(5000);
					break;
				}
			}
		}
	}

	~PlakuTreeInterface() {
		delete uniformSampler;
		delete uniformSamplerBackingKDTree;
	}

	void draw() {
		std::vector<std::vector<double>> colorLookup(regions.size());

		double min = 1000000000;
		double max = 0;
		for(unsigned int i = 0; i < regions.size(); ++i) {
			min = std::min(regions[i]->heuristic, min);
			if(!isinf(regions[i]->heuristic))
				max = std::max(regions[i]->heuristic, max);
		}

		for(unsigned int i = 0; i < regions.size(); ++i) {
			colorLookup[i] = getColor(min, max, regions[i]->heuristic);
		}

		discretization.draw(true, false, colorLookup);
	}

	State getTreeSample() {
		if(activeRegion != NULL) {
			activeRegion->selected(alpha);

			if(!activeRegion->onOpen) {
				regionHeap.push_back(activeRegion);
				std::push_heap(regionHeap.begin(), regionHeap.end(), Region::HeapCompare);
				activeRegion->onOpen = true;
			}
		}

		if(distribution(GlobalRandomGenerator) < b) {
			assert(!regionHeap.empty());

			activeRegion = regionHeap.front();
			std::pop_heap(regionHeap.begin(), regionHeap.end(), Region::HeapCompare);
			regionHeap.pop_back();

			activeRegion->onOpen = false;

			unsigned int regionAlongPath = activeRegion->getRandomRegionAlongPathToGoal(distribution);
			State p = discretization.getRandomStateNearRegionCenter(regionAlongPath, stateRadius);

			return activeRegion->getNearestStateInRegion(p);
		} else {
			return uniformSampler->getTreeSample();
		}
	}

	void insertIntoTree(Edge* edge) {
		unsigned int newCellId = discretization.getCellId(edge->end);

		if(!regions[newCellId]->onOpen) {
			regionHeap.push_back(regions[newCellId]);
			std::push_heap(regionHeap.begin(), regionHeap.end(), Region::HeapCompare);
			regions[newCellId]->onOpen = true;
		}

		regions[newCellId]->edgesInRegion->insertPoint(edge);
		uniformSamplerBackingKDTree->insertPoint(edge);
	}

	Edge* getTreeEdge(const State& s) const {
		fatal("Not implemented: PlakuTreeInterface::getTreeEdge");
		return NULL;
	}

private:
	std::vector<double> getColor(double min, double max, double value) const {
		std::vector<double> color(3);

		value = ((value - min) / (max - min)) * 510;

		color[0] = 0;
		color[1] = 255;
		color[2] = 0;
		
		if(value >= 255) {
			color[0] = 255;
			value -= 255;
			color[1] -= value;
			if(color[1] < 0) color[1] = 0;
		}
		else {
			color[0] += value;
		}

		for(unsigned int i = 0; i < 3; ++i) {
			color[i] /= 255;
		}

		return color;
	}

	void dijkstra(Region *region) {
		InPlaceBinaryHeap<Region> open;
		region->setHeuristicAndPath(0, std::vector<unsigned int>());
		open.push(region);

		while(!open.isEmpty()) {
			Region *current = open.pop();

			std::vector<unsigned int> kids = discretization.getNeighboringCells(current->id);
			for(unsigned int kid : kids) {
				double newHeuristic = current->heuristic + discretization.getEdgeCostBetweenCells(current->id, kid);
				Region *kidPtr = regions[kid];

				if(newHeuristic < kidPtr->heuristic) {
					kidPtr->setHeuristicAndPath(newHeuristic, current->regionPath);

					if(open.inHeap(kidPtr)) {
						open.siftFromItem(kidPtr);
					} else {
						open.push(kidPtr);
					}
				}
			}
		}
	}

	const Workspace &workspace;
	const Agent &agent;
	const Discretization &discretization;
	
	UniformSampler *uniformSampler;
	KDTree *uniformSamplerBackingKDTree;

	unsigned int startRegionId, goalRegionId;
	std::vector<Region*> regions, regionHeap;
	Region *activeRegion;

	double alpha, b, stateRadius;
	std::uniform_real_distribution<double> distribution;
};