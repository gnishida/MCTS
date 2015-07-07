#include "ParametricLSystem.h"
#include <iostream>
#include <time.h>
#include <algorithm>
#include "MLUtils.h"
#include "GLUtils.h"
#include <list>
#include <QGLWidget>

#define MAX_ITERATIONS						30
#define MAX_ITERATIONS_FOR_MC				50
#define NUM_MONTE_CARLO_SAMPLING			500

#define ALPHA								1.0
#define BETA								1.0

#define PARAM_EXPLORATION					0.3 //1
#define PARAM_EXPLORATION_VARIANCE			0.1//10

//#define DEBUG		1

namespace parametriclsystem {

const double M_PI = 3.141592653592;

Literal::Literal(const string& name, int depth) {
	this->name = name;
	this->depth = depth;
	this->param_defined = false;
}

Literal::Literal(const string& name, int depth, double param_value) {
	this->name = name;
	this->depth = depth;
	this->param_values.push_back(param_value);
	this->param_defined = true;
}

Literal::Literal(const string& name, int depth, double param_value1, double param_value2) {
	this->name = name;
	this->depth = depth;
	this->param_values.push_back(param_value1);
	this->param_values.push_back(param_value2);
	this->param_defined = true;
}

Literal::Literal(const string& name, int depth, double param_value1, double param_value2, double param_value3) {
	this->name = name;
	this->depth = depth;
	this->param_values.push_back(param_value1);
	this->param_values.push_back(param_value2);
	this->param_values.push_back(param_value3);
	this->param_defined = true;
}

Literal::Literal(const string& name, int depth, double param_value1, double param_value2, double param_value3, double param_value4) {
	this->name = name;
	this->depth = depth;
	this->param_values.push_back(param_value1);
	this->param_values.push_back(param_value2);
	this->param_values.push_back(param_value3);
	this->param_values.push_back(param_value4);
	this->param_defined = true;
}

Literal::Literal(const string& name, int depth, double param_value1, double param_value2, double param_value3, double param_value4, double param_value5) {
	this->name = name;
	this->depth = depth;
	this->param_values.push_back(param_value1);
	this->param_values.push_back(param_value2);
	this->param_values.push_back(param_value3);
	this->param_values.push_back(param_value4);
	this->param_values.push_back(param_value5);
	this->param_defined = true;
}

Literal::Literal(const string& name, int depth, const std::vector<double>& param_values) {
	this->name = name;
	this->depth = depth;
	this->param_values = param_values;
	this->param_defined = true;
}

String Literal::operator+(const Literal& l) const {
	String ret = *this;
	return ret + l;
}

String::String(const string& str, int depth) {
	this->str.push_back(Literal(str, depth));
}

String::String(const Literal& l) {
	this->str.push_back(l);
}

void String::operator+=(const String& str) {
	for (int i = 0; i < str.length(); ++i) {
		this->str.push_back(str[i]);
	}
}

String String::operator+(const String& str) const {
	String new_str = *this;

	for (int i = 0; i < str.length(); ++i) {
		new_str.str.push_back(str[i]);
	}

	return new_str;
}

void String::replace(int index, const String& str) {
	this->str.erase(this->str.begin() + index);
	this->str.insert(this->str.begin() + index, str.str.begin(), str.str.end());
}

ostream& operator<<(ostream& os, const String& str) {
	os << setprecision(1);
	for (int i = 0; i < str.length(); ++i) {
		os << str[i].name;
		if (str[i].param_defined) {
			os << "(" << fixed;
			for (int j = 0; j < str[i].param_values.size(); ++j) {
				if (j > 0) {
					os << ",";
				}
				os << str[i].param_values[j];
			}
			os << ")";
		}
	}

    return os;
}

Action::Action(int action_index, int index, const String& rule) {
	this->type = ACTION_RULE;
	this->action_index = action_index;
	this->index = index;
	this->rule = rule;
}

Action::Action(int action_index, int index, double value) {
	this->type = ACTION_VALUE;
	this->action_index = action_index;
	this->index = index;
	this->value = value;
}

/**
 * 指定されたモデルに、このアクションを適用する。
 */
String Action::apply(const String& model) {
	String new_model = model;

	if (type == ACTION_RULE) {
		new_model.replace(index, rule);
	} else {
		new_model[index].param_values.push_back(value);
		new_model[index].param_defined = true;
	}

	return new_model;
}

Node::Node(const String& model) {
	this->parent = NULL;
	this->model = model;
	best_score = -std::numeric_limits<double>::max();
	visits = 0;
	fixed = false;
}

Node::Node(const String& model, const Action& action) {
	this->parent = NULL;
	this->model = model;
	this->action = action;
	best_score = -std::numeric_limits<double>::max();
	visits = 0;
	fixed = false;
}

/**
 * ノードに、指定されたmodelの子ノードを追加する。
 */
Node* Node::addChild(const String& model, const Action& action) {
	Node* child = new Node(model, action);
	children.push_back(child);
	child->parent = this;
	return child;
}

/**
 * ノードに、actionを登録する。
 * また、untriedActionsに、登録したactionのindex番号のリストをセットする
 *
 * @param actions	actionのリスト
 */
void Node::setActions(std::vector<Action>& actions) {
	this->actions = actions;
	for (int i = 0; i < actions.size(); ++i) {
		untriedActions.push_back(i);
	}
}

/**
 * untriedActionsから1つをランダムに選択して返却する。
 * 選択されたactionは、リストから削除される。
 *
 * @return		選択されたaction
 */
Action Node::randomlySelectAction() {
	int index = ml::genRand(0, untriedActions.size());
	Action action = actions[untriedActions[index]];
	untriedActions.erase(untriedActions.begin() + index);
	return action;
}

/**
 * untriedActionsの中で、指定されたindex値を持つ要素を削除する。
 */
void Node::removeAction(int index) {
	for (int i = 0; i < untriedActions.size(); ++i) {
		if (untriedActions[i] == index) {
			untriedActions.erase(untriedActions.begin() + i);
			return;
		}
	}
}

/**
 * childrenの中で、指定されたindex値を持つ子ノードを返却する。
 */
Node* Node::getChild(int index) {
	for (int i = 0; i < children.size(); ++i) {
		if (children[i]->action.action_index == index) return children[i];
	}
	return NULL;
}

/**
 * UCTアルゴリズムに従い、子ノードを1つ選択する。
 */
Node* Node::UCTSelectChild() {
	double max_uct = -std::numeric_limits<double>::max();
	Node* best_child = NULL;

	for (int i = 0; i < children.size(); ++i) {
		// スコアが確定済みの子ノードは探索対象外とする
		if (children[i]->fixed) continue;

		double uct;
		if (children[i]->visits == 0) {
			uct = 10000 + ml::genRand(0, 1000);
		} else {
			uct = children[i]->best_score
				+ PARAM_EXPLORATION * sqrt(2 * log((double)visits) / (double)children[i]->visits)
				+ PARAM_EXPLORATION_VARIANCE * sqrt(ml::variance(children[i]->scores) + 0.0 / (double)children[i]->visits);
		}

		if (uct > max_uct) {
			max_uct = uct;
			best_child = children[i];
		}
	}

	return best_child;
}

/**
 * ベストスコアの子ノードを返却する。
 *
 * @return		ベストスコアの子ノード
 */
Node* Node::bestChild() {
	double best_score = -std::numeric_limits<double>::max();
	Node* best_child = NULL;

	for (int i = 0; i < children.size(); ++i) {
		if (children[i]->best_score > best_score) {
			best_score = children[i]->best_score;
			best_child = children[i];
		}
	}

	return best_child;
}

ParametricLSystem::ParametricLSystem(int grid_size, float scale) {
	this->grid_size = grid_size;
	this->scale = scale;

	axiom = String(Literal("X", 0, 6, 1));
	/*
	rules['X'].push_back(Rule("X(l,r,s)", "F(l,r,s)"));
	rules['X'].push_back(Rule("X(l,r,s)", "F(l/2,r,r/2+s/2)[-X(l/2,r/2+s/2,s)][+X(l/2,r/2+s/2,s)]X(l/2,r/2+s/2,s)"));
	*/
}

void ParametricLSystem::draw(const String& model, std::vector<Vertex>& vertices) {
	vertices.clear();

	glm::mat4 modelMat;
	std::list<glm::mat4> stack;

	for (int i = 0; i < model.length(); ++i) {
		if (model[i].name == "[") {
			stack.push_back(modelMat);
		} else if (model[i].name == "]") {
			modelMat = stack.back();
			stack.pop_back();
		} else if (model[i].name == "+" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_values[0]), glm::vec3(0, 1, 0));
		} else if (model[i].name == "-" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(-model[i].param_values[0]), glm::vec3(0, 1, 0));
		} else if (model[i].name == "#" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_values[0]), glm::vec3(0, 1, 0));
		} else if (model[i].name == "\\" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_values[0]), glm::vec3(0, 0, 1));
		} else if (model[i].name == "/" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(-model[i].param_values[0]), glm::vec3(0, 0, 1));
		} else if (model[i].name == "F" && model[i].param_defined) {
			double length = model[i].param_values[0];
			double radius1 = model[i].param_values[1];
			double radius2 = radius1 - length / 20.0f;
			
			glm::vec4 p(0, 0, 0, 1);
			p = modelMat * p;
			//if (p.x >= -grid_size * 0.5 && p.x < grid_size * 0.5 && p.y >= 0 && p.y < grid_size) { // hack: 領域の外は描画しない
				// 線を描画する
				std::vector<Vertex> new_vertices;
				glutils::drawCone(glm::vec3(0, 0, 0), length, radius1, radius2, glm::vec3(ml::genRand(0, 1), 1, 1), modelMat, vertices);
			//}

			modelMat = glm::translate(modelMat, glm::vec3(0, 0, length));
		} else if (model[i].name == "X") {
		} else {
		}
	}
}

/**
 * 適当な乱数シードに基づいて、ランダムにgenerateする。
 *
 * @param random_seed		乱数シード
 * @return					生成されたモデル
 */
String ParametricLSystem::derive(int random_seed, cv::Mat& indicator) {
	std::vector<int> derivation_history;

	ml::initRand(random_seed);
	return derive(axiom, MAX_ITERATIONS, indicator, derivation_history);
}

/**
 * 指定された開始モデルからスタートし、ランダムにgenerateする。
 *
 * @param start_model		開始モデル
 * @param random_seed		乱数シード
 * @param max_iterations	繰り返し数
 * @param build_tree		木を生成するか？
 * @param indicator [OUT]	生成されたモデルのindicator
 * @return					生成されたモデル
 */
String ParametricLSystem::derive(const String& start_model, int max_iterations, cv::Mat& indicator, std::vector<int>& derivation_history) {
	String model = start_model;

	for (int iter = 0; iter < max_iterations; ++iter) {
		std::vector<Action> actions = getActions(model);
		if (actions.size() == 0) break;

		int index = ml::genRand(0, actions.size());
		derivation_history.push_back(index);
		model = actions[index].apply(model);
	}

	// indicatorを計算する
	computeIndicator(model, scale, indicator);

	return model;
}

/**
 * 指定された文字列に基づいてモデルを生成し、indicatorを計算して返却する。
 * 
 * @param rule				モデルを表す文字列
 * @param scale				grid_size * scaleのサイズでindicatorを計算する
 * @param indicator [OUT]	indicator
 */
void ParametricLSystem::computeIndicator(const String& model, float scale, cv::Mat& indicator) {
	int size = grid_size * scale;

	indicator = cv::Mat::zeros(size, size, CV_32F);

	std::list<glm::mat4> stack;

	glm::mat4 modelMat;

	for (int i = 0; i < model.length(); ++i) {
		if (model[i].name == "[") {
			stack.push_back(modelMat);
		} else if (model[i].name == "]") {
			modelMat = stack.back();
			stack.pop_back();
		} else if (model[i].name == "+" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_values[0]), glm::vec3(0, 1, 0));
		} else if (model[i].name == "-" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(-model[i].param_values[0]), glm::vec3(0, 1, 0));
		} else if (model[i].name == "#" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_values[0]), glm::vec3(0, 1, 0));
		} else if (model[i].name == "\\" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_values[0]), glm::vec3(0, 0, 1));
		} else if (model[i].name == "/" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(-model[i].param_values[0]), glm::vec3(0, 0, 1));
		} else if (model[i].name == "F" && model[i].param_defined) {
			double length = model[i].param_values[0] * scale;
			double radius1 = model[i].param_values[1] * scale;
			double radius2 = radius1 - length / 20.0f;
			double radius = (radius1 + radius2) * 0.5f;

			// 線を描画する代わりに、indicatorを更新する
			glm::vec4 p1(0, 0, 0, 1);
			glm::vec4 p2(0, 0, length, 1);
			p1 = modelMat * p1;
			p2 = modelMat * p2;
			int u1 = p1.x + size * 0.5;
			int v1 = p1.z;
			int u2 = p2.x + size * 0.5;
			int v2 = p2.z;

			if (radius * scale > 0.2) {
				int thickness = max(1.0, radius * scale);
				cv::line(indicator, cv::Point(u1, v1), cv::Point(u2, v2), cv::Scalar(1), thickness);
			}

			modelMat = glm::translate(modelMat, glm::vec3(0, 0, length));
		} else if (model[i].name == "X") {
		} else {
		}
	}
}

/**
 * 指定されたターゲットindicatorに近いモデルを生成する。
 *
 * @param target			ターゲットindicator
 * @param indicator [OUT]	生成されたモデルのindicator
 * @return					生成されたモデル
 */
String ParametricLSystem::inverse(const cv::Mat& target) {
	// 白色のピクセルの数をカウント
	int count = 0;
	for (int r = 0; r < target.rows; ++r) {
		for (int c = 0; c < target.cols; ++c) {
			if (ml::mat_get_value(target, r, c) > 0.5) {
				count++;
			}
		}
	}

	// UCTを使って探索木を構築していく
	String model = axiom;
	Node* root = new Node(model);
	root->setActions(getActions(model));
	num_nodes = 1;

	for (int l = 0; l < MAX_ITERATIONS; ++l) {
		// もしノードがリーフノードなら、終了
		if (root->untriedActions.size() == 0 && root->children.size() == 0) break;

		Node* best_child = UCT(root, target, count);

		// ベストスコアの子ノード以外のメモリを解放する
		for (int i = 0; i < root->children.size(); ++i) {
			if (root->children[i] == best_child) continue;
			releaseNodeMemory(root->children[i]);
		}

		// ベストの子ノードをルートノードにする
		root = best_child;
		root->parent = NULL;

		cout << l << ": " << "Best score=" << root->best_score << ", #nodes=" << num_nodes << endl;
	}

	// ベストスコアの子孫を辿ってリーフノードまで行き、モデルを取得する
	Node* node = root;
	while (node->children.size() > 0) {
		node = node->bestChild();
	}
	model = node->model;

	// ノードのメモリを解放
	releaseNodeMemory(root);

	// スコア表示
	cv::Mat indicator;
	computeIndicator(model, scale, indicator);
	cout << score(indicator, target, count) << endl;

	return model;
}

/**
 * 指定されたmodelからUCTをスタートし、最善のoptionを返却する。
 *
 * @param model		モデル
 * @param target	ターゲット
 * @return			最善のoption
 */
Node* ParametricLSystem::UCT(Node* current_node, const cv::Mat& target, int white_count) {
	for (int iter = 0; iter < NUM_MONTE_CARLO_SAMPLING; ++iter) {
		// 現在のノードのスコアが確定したら、終了
		if (current_node->fixed) break;

		Node* node = current_node;

		// 探索木のリーフノードを選択
		while (node->untriedActions.size() == 0 && node->children.size() > 0) {
			node = node->UCTSelectChild();
		}

		// 子ノードがまだ全てexpandされていない時は、1つランダムにexpand
		if (node->untriedActions.size() > 0 && node->children.size() < sqrt(2.0 * log(iter + 1.0))) {
			Action action = node->randomlySelectAction();
			String child_model = action.apply(node->model);

			node = node->addChild(child_model, action);
			node->setActions(getActions(child_model));
			num_nodes++;
		}

		// ランダムにderiveする
		cv::Mat indicator;
		std::vector<int> derivation_history;
		derive(node->model, MAX_ITERATIONS_FOR_MC, indicator, derivation_history);

		// スコアを計算する
		double sc = score(indicator, target, white_count);
		node->best_score = sc;

		// リーフノードなら、スコアを確定する
		if (node->untriedActions.size() == 0 && node->children.size() == 0) {
			node->fixed = true;
		}

		// スコアをbackpropagateする
		bool updated = false;
		Node* leaf = node;
		while (node != NULL) {
			node->visits++;
			node->scores.push_back(sc);
			if (sc > node->best_score) {
				node->best_score = sc;
				updated = true;
			}

			// 子ノードが全て展開済みで、且つ、スコア確定済みなら、このノードのスコアも確定とする
			if (node->untriedActions.size() == 0) {
				bool fixed = true;
				for (int c = 0; c < node->children.size(); ++c) {
					if (!node->children[c]->fixed) {
						fixed = false;
						break;
					}
				}
				node->fixed = fixed;
			}

			node = node->parent;
		}

		// ベストスコアを更新した場合は、このderivation上のノードを全て実体化する
		if (updated) {

			/////// デバッグ ///////
			char filename[256];
			sprintf(filename, "indicator_%lf.png", sc);
			ml::mat_save(filename, indicator);
			/////// デバッグ ///////


			Node* node = leaf;
			for (int di = 0; di < derivation_history.size(); ++di) {
				Node* c = node->getChild(derivation_history[di]);
				if (c == NULL) {
					node->removeAction(derivation_history[di]);
					String child_model = node->actions[derivation_history[di]].apply(node->model);

					c = node->addChild(child_model, node->actions[derivation_history[di]]);
					c->setActions(getActions(child_model));
					num_nodes++;
				}

				c->visits++;
				c->best_score = sc;
				c->scores.push_back(sc);

				node = c;
				
				// リーフノードなら、スコアを確定する
				if (node->untriedActions.size() == 0 && node->children.size() == 0) {
					node->fixed = true;
				}
			}
		}
	}



	////// デバッグ //////
	/*
	for (int i = 0; i < current_node->children.size(); ++i) {
		cout << "  " << i << ":" << endl;
		if (current_node->children[i]->action.type == Action::ACTION_RULE) {
			cout << "   " << current_node->children[i]->action.rule << endl;
		} else {
			cout << "   " << current_node->children[i]->action.value << endl;
		}
		cout << "    - visits: " << current_node->children[i]->visits << endl;
		cout << "    - best score: " << current_node->children[i]->best_score << endl;
		cout << "    - avg score: " << ml::mean(current_node->children[i]->scores) << endl;
		cout << "    - var score: " << ml::variance(current_node->children[i]->scores) << endl;
	}
	*/
	////// デバッグ //////



	// ベストスコアの子ノードを返却する
	return current_node->bestChild();
}

/**
 * indicatorとターゲットとの距離を計算して返却する。
 * ※もう使っていない！
 *
 * @param indicator		indicator
 * @param target		ターゲットindicator
 * @return				距離
 */
double ParametricLSystem::distance(const cv::Mat& indicator, const cv::Mat& target, double alpha, double beta) {
	double dist = 0.0;

	for (int r = 0; r < target.rows; ++r) {
		for (int c = 0; c < target.cols; ++c) {
			double target_val = ml::mat_get_value(target, r, c);
			double val = ml::mat_get_value(indicator, r, c);

			if (target_val > 0) {
				dist += (target_val - val) * (target_val - val) * alpha;
			} else {
				dist += (target_val - val) * (target_val - val) * beta;
			}
		}
	}

	return sqrt(dist);
}

/**
 * indicatorのスコアを計算して返却する。
 *
 * @param indicator		indicator
 * @param target		ターゲットindicator
 * @return				距離
 */
double ParametricLSystem::score(const cv::Mat& indicator, const cv::Mat& target, int white_count) {
	return 1.0 - ml::mat_squared_sum(indicator - target) / white_count;
}

/**
 * 指定されたモデルの、次のderivationの候補を返却する。
 * Hardcoding: 各literalは、4つのパラメータを持つ (このセグメントの長さ、このセグメントの最初の半径)
 *
 * @param model		モデル
 * @return			次のderivationの候補リスト
 */
std::vector<Action> ParametricLSystem::getActions(const String& model) {
	std::vector<Action> actions;

	// 展開するパラメータを決定
	int i = findNextLiteralToDefineValue(model);

	// 新たなderivationがないなら、終了
	if (i == -1) return actions;

	if (model[i].name == "X") {
		/*String rule = Literal("F", model[i].depth + 1, model[i].param_values[0], model[i].param_values[1]);
		actions.push_back(Action(0, i, rule));*/

		if (model[i].param_values[0] >= 0.1f) {
			String rule = Literal("F", model[i].depth + 1, model[i].param_values[0], model[i].param_values[1])
				+ Literal("\\", model[i].depth + 1)
				+ Literal("#", model[i].depth + 1)
				+ Literal("X", model[i].depth + 1, model[i].param_values[0] * 0.7f, model[i].param_values[1] - model[i].param_values[0] / 20.0f);
			actions.push_back(Action(1, i, rule));

			rule = Literal("F", model[i].depth + 1, model[i].param_values[0] * 0.5f, model[i].param_values[1])
				+ Literal("[", model[i].depth + 1)
				+ Literal("+", model[i].depth + 1)
				+ Literal("X", model[i].depth + 1, model[i].param_values[0] * 0.7f, model[i].param_values[1] * 0.7f)
				+ Literal("]", model[i].depth + 1)
				+ Literal("F", model[i].depth + 1, model[i].param_values[0] * 0.5f, model[i].param_values[1] - model[i].param_values[0] / 40.0f)
				+ Literal("\\", model[i].depth + 1)
				+ Literal("#", model[i].depth + 1)
				+ Literal("X", model[i].depth + 1, model[i].param_values[0] * 0.7f, model[i].param_values[1] - model[i].param_values[0] / 20.0f);
			actions.push_back(Action(2, i, rule));
		}
	} else if (model[i].name == "-" || model[i].name == "+") {
		int count = 0;
		for (float k = 40.0f; k <= 80.0f; k += 20.0f, ++count) {
			actions.push_back(Action(count, i, k));
		}
	} else if (model[i].name == "#") {
		int count = 0;
		for (int k = -5; k <= 5; k += 10, ++count) {
			actions.push_back(Action(count, i, k));
		}
	} else if (model[i].name == "\\") {
		int count = 0;
		actions.push_back(Action(count, i, 180));
		/*
		for (float k = 90; k < 150; k += 30, ++count) {
			actions.push_back(Action(count, i, k));
		}
		*/
	}

	return actions;
}

/**
 * 現在のモデルについて、値が未設定のパラメータで最もレベルが低く、最も左側に位置するindexを返却する。
 * もし未設定のパラメータがない場合は、-1を返却する。
 *
 * @param str		現在のモデル
 * @return			インデックス
 */
int ParametricLSystem::findNextLiteralToDefineValue(const String& str) {
	int min_depth1 = std::numeric_limits<int>::max();
	int min_depth2 = std::numeric_limits<int>::max();
	int min_index1 = -1;
	int min_index2 = -1;

	for (int i = 0; i < str.length(); ++i) {
		if (str[i].name == "X") {
			if (str[i].depth < min_depth1) {
				min_depth1 = str[i].depth;
				min_index1 = i;
			}
		} else if ((str[i].name == "+" || str[i].name == "-" || str[i].name == "\\" || str[i].name == "/" || str[i].name == "#") && !str[i].param_defined) {
			if (str[i].depth < min_depth2) {
				min_depth2 = str[i].depth;
				min_index2 = i;
			}
		}
	}

	if (min_depth1 < min_depth2) {
		return min_index1;
	} else {
		return min_index2;
	}
}

/**
 * 指定されたノードをルートとする探索木のメモリを解放する
 *
 * @param node		ノード
 */
void ParametricLSystem::releaseNodeMemory(Node* node) {
	for (int i = 0; i < node->children.size(); ++i) {
		if (node->children[i] != NULL) {
			releaseNodeMemory(node->children[i]);
			num_nodes--;
		}
	}
	delete node;
	node = NULL;
}

float deg2rad(float deg) {
	return deg * M_PI / 180.0;
}

std::vector<std::string> split(const string& str, char delim) {
	std::vector<std::string> res;
	size_t current = 0, found;
	while ((found = str.find_first_of(delim, current)) != string::npos) {
		res.push_back(string(str, current, found - current));
		current = found + 1;
	}
	res.push_back(string(str, current, str.size() - current));
	return res;
}

}
