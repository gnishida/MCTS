#pragma once

#include <vector>

namespace mcts {

class Node {
public:

	float value;		// distance
	std::vector<Node> children;
};

class MCTS {
public:
	MCTS();

	void uct_tree();
};

}