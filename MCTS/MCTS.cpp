#include "MCTS.h"
#include "MLUtils.h"
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace mcts {

Node::Node() {
	num_visited = 0;
}

int MCTS::CREATE_NEW_NODE_LIMIT = 100;

MCTS::MCTS() {
	param_exploration = sqrt(2.0);
}


/**
 * 指定されたノードから、UCTに基づいて1つの子ノードを選択し、再帰的にtraverseし、探索木を構築していく。
 */
float MCTS::uct_traverse(Node* node) {
	// UCT値が最大のノードを選択する
	float max_uct = 0;
	Node* selected_child = NULL;
	for (int i = 0; i < node->children.size(); ++i) {
		float uct_value = uct(node->children[i], node->num_playout);
		if (uct_value > max_uct) {
			max_uct = uct_value;
			selected_child = node->children[i];
		}
	}

	if (selected_child->num_visited >= CREATE_NEW_NODE_LIMIT) {
		// 子ノードを探索木に追加する。
		Node* child_node = new Node();
	}

	float score;
	if (selected_child->num_visited < CREATE_NEW_NODE_LIMIT) {
		score = mc_one_simulation(selected_child);
	} else {
		score = uct_traverse(selected_child);
	}

	selected_child->max_score = std::max(node->max_score, score);
	selected_child->num_visited++;

	node->num_playout++;

	return score;
}

/**
 * UCT値: max_score + c sqrt(ln(n) / n_i) を返却する。
 * ただし、n_i=0の時は、上記関数は∞となり、プログラムではエラーとなるので、代わりに10000 + rand(100)を返す。
 *
 * @param node			ノード
 * @param num_playout	playout回数
 * @return				指定されたノードのUCT値
 */
float MCTS::uct(Node* node, int num_playout) {
	if (node->num_visited == 0) {
		return 10000 + ml::genRand(0, 100);
	} else {
		return node->max_score + param_exploration * sqrt(log((double)num_playout) / node->num_visited);
	}
}

/**
 * 指定されたノードからスタートし、Monte Carlo法でランダムに指定されたstep数だけderivationを実施し、
 * 得られたモデルのスコアを返却する。
 *
 * @param node	ノード
 * @return		ランダムに生成されたモデルのスコア
 */
float MCTS::mc_one_simulation(Node* node) {
	cv::Mat indicator; 

	parametriclsystem::ParametricLSystem pls;
	pls.derive(node->model, indicator);

	return 0;
}

}