#pragma once

#include <vector>
#include "ParametricLSystem.h"

namespace mcts {

class MCTSNode {
public:
	float max_score;	// ベストスコア
	std::vector<MCTSNode*> children;
	int num_visited;	// このノードの訪問回数
	int num_playout;
	parametriclsystem::String model;

public:
	MCTSNode();
};

class MCTS {
public:
	static int CREATE_NEW_NODE_LIMIT;

public:
	float param_exploration;

public:
	MCTS();

	float uct_traverse(Node* node);
	float uct(Node* node, int num_playout);
	float mc_one_simulation(Node* node);
};

}