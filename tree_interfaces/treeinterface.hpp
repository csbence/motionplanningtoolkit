template<class Agent, class InsertionInteface, class QueryInterface>
class TreeInterface {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
public:
	TreeInterface(InsertionInteface& insertionInterface, QueryInterface &queryInterface) :
		insertionInterface(insertionInterface), queryInterface(queryInterface) {}

	State getTreeSample() {
		return queryInterface.getTreeSample();
	}

	void insertIntoTree(Edge* edge) {
		insertionInterface.insertPoint(edge);
	}

private:
	InsertionInteface &insertionInterface;
	QueryInterface &queryInterface;
};