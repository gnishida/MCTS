#include "ParametricLSystem.h"
#include <iostream>
#include <time.h>
#include <algorithm>
#include "MLUtils.h"
#include "GLUtils.h"
#include <list>
#include <QGLWidget>

#define MAX_ITERATIONS						200
#define MAX_ITERATIONS_FOR_MC				20
#define NUM_MONTE_CARLO_SAMPLING			200

#define ALPHA								1.0
#define BETA								1.0

#define PARAM_EXPLORATION					0.01//0.3 //1

//#define DEBUG		1

namespace parametriclsystem {

const double M_PI = 3.141592653592;

Literal::Literal(char c, int level) {
	this->c = c;
	this->level = level;
	this->param_value = 0.0;
	this->param_defined = false;
}

Literal::Literal(char c, int level, double param_value) {
	this->c = c;
	this->level = level;
	this->param_value = param_value;
	this->param_defined = true;
}

String::String(string str, int level) {
	for (int i = 0; i < str.length(); ++i) {
		this->str.push_back(Literal(str[i], level));
	}
}

String::String(Literal l) {
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
		os << str[i].c;
		if (str[i].param_defined) {
			os << "(" << fixed << str[i].param_value << ")";
		}
	}

    return os;
}

Action::Action(int index, const string& rule) {
	this->type = ACTION_RULE;
	this->index = index;
	this->rule = rule;
}

Action::Action(int index, double value) {
	this->type = ACTION_VALUE;
	this->index = index;
	this->value = value;
}

/**
 * 指定されたモデルに、このアクションを適用する。
 */
String Action::apply(const String& model) {
	String new_model = model;

	if (type == ACTION_RULE) {
		new_model.replace(index, String(rule, model[index].level + 1));
	} else {
		new_model[index].param_value = value;
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
 * untriedActionsから1つをランダムに選択して返却する。
 * 選択されたactionは、リストから削除される。
 *
 * @return		選択されたaction
 */
Action Node::randomlySelectAction() {
	int index = ml::genRand(0, untriedActions.size());
	Action action = untriedActions[index];
	untriedActions.erase(untriedActions.begin() + index);
	return action;
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
			uct = children[i]->best_score + PARAM_EXPLORATION * sqrt(2 * log((double)visits) / (double)children[i]->visits);
		}

		if (uct > max_uct) {
			max_uct = uct;
			best_child = children[i];
		}
	}

	return best_child;
}

ParametricLSystem::ParametricLSystem(int grid_size, float scale) {
	this->grid_size = grid_size;
	this->scale = scale;

	axiom = "X";
	rules['X'].push_back("F");
	rules['X'].push_back("F[-X][+X]");
}

void ParametricLSystem::draw(const String& model, std::vector<Vertex>& vertices) {
	vertices.clear();

	glm::mat4 modelMat;
	std::list<glm::mat4> stack;

	for (int i = 0; i < model.length(); ++i) {
		if (model[i].c == '[') {
			stack.push_back(modelMat);
		} else if (model[i].c == ']') {
			modelMat = stack.back();
			stack.pop_back();
		} else if (model[i].c == '+') {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_value), glm::vec3(0, 0, 1));
		} else if (model[i].c == '-') {
			modelMat = glm::rotate(modelMat, deg2rad(-model[i].param_value), glm::vec3(0, 0, 1));
		} else if (model[i].c == 'F') {
			if (model[i].param_defined) {
				double length = model[i].param_value;
			
				// 線を描画する
				std::vector<Vertex> new_vertices;
				glutils::drawSphere(glm::vec3(0, 0, 0), 1, glm::vec3(1, 1, 1), modelMat, vertices);
				glutils::drawCylinder(glm::vec3(0, 0, 0), length, 1, glm::vec3(1, 1, 1), glm::rotate(modelMat, deg2rad(-90), glm::vec3(1, 0, 0)), vertices);

				modelMat = glm::translate(modelMat, glm::vec3(0, length, 0));
			}
		} else if (model[i].c == 'X') {
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
	ml::initRand(random_seed);
	return derive(String(axiom, 0), MAX_ITERATIONS, indicator);
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
String ParametricLSystem::derive(const String& start_model, int max_iterations, cv::Mat& indicator) {
	//ml::initRand(random_seed);

	String result = start_model;

	for (int iter = 0; iter < max_iterations; ++iter) {
	//for (int iter = 0; ; ++iter) {
		// 展開するパラメータを決定
		int i = findNextLiteralToDefineValue(result);

		// 新たなderivationがないなら、終了
		if (i == -1) break;

		if (rules.find(result[i].c) != rules.end()) {
			int index = chooseRule(result[i]);

			// もしmax_levelを超えたら、終了する
			if (iter >= max_iterations) {
				//break;
			}

			result.replace(i, String(rules[result[i].c][index], result[i].level + 1));
		} else if (result[i].c == 'F') {
			double mean_val = grid_size * 0.5 / pow(1.5, result[i].level);
			double val = ml::genRandInt(mean_val * 0.8, mean_val * 1.2, 3);
			//double val = ml::genRandInt(grid_size * 0.5 / (result[i].level + 1) * 0.8, grid_size * 0.5 / (result[i].level + 1) * 1.2, 3);
			result[i] = Literal(result[i].c, result[i].level, val);
		} else if (result[i].c == '+' || result[i].c == '-') {
			double val = ml::genRandInt(10, 60, 3);
			result[i] = Literal(result[i].c, result[i].level, val);
		}
	}

	// indicatorを計算する
	computeIndicator(result, scale, indicator);

	return result;
}

/**
 * 指定された文字列に基づいてモデルを生成し、indicatorを計算して返却する。
 * 
 * @param rule				モデルを表す文字列
 * @param scale				grid_size * scaleのサイズでindicatorを計算する
 * @param indicator [OUT]	indicator
 */
void ParametricLSystem::computeIndicator(String rule, float scale, cv::Mat& indicator) {
	int size = grid_size * scale;

	indicator = cv::Mat::zeros(size, size, CV_32F);

	std::list<glm::mat4> stack;

	glm::mat4 modelMat;

	for (int i = 0; i < rule.length(); ++i) {
		if (rule[i].c == '[') {
			stack.push_back(modelMat);
		} else if (rule[i].c == ']') {
			modelMat = stack.back();
			stack.pop_back();
		} else if (rule[i].c == '+') {
			modelMat = glm::rotate(modelMat, deg2rad(rule[i].param_value), glm::vec3(0, 0, 1));
		} else if (rule[i].c == '-') {
			modelMat = glm::rotate(modelMat, deg2rad(-rule[i].param_value), glm::vec3(0, 0, 1));
		} else if (rule[i].c == 'F') {
			double length;
			if (rule[i].param_defined) {
				length = rule[i].param_value * scale;
			} else {
				length = 5.0;
			}
			
			// 線を描画する代わりに、indicatorを更新する
			glm::vec4 p1(0, 0, 0, 1);
			glm::vec4 p2(0, length, 0, 1);
			p1 = modelMat * p1;
			p2 = modelMat * p2;
			int u1 = p1.x + size * 0.5;
			int v1 = p1.y;
			int u2 = p2.x + size * 0.5;
			int v2 = p2.y;

			int thickness = max(1.0, 3.0 * scale);
			cv::line(indicator, cv::Point(u1, v1), cv::Point(u2, v2), cv::Scalar(1), thickness);

			modelMat = glm::translate(modelMat, glm::vec3(0, length, 0));
		} else if (rule[i].c == 'X') {
			double length = 3.0;
			
			// 線を描画する代わりに、indicatorを更新する
			glm::vec4 p1(0, 0, 0, 1);
			glm::vec4 p2(0, length, 0, 1);
			p1 = modelMat * p1;
			p2 = modelMat * p2;
			int u1 = p1.x + size * 0.5;
			int v1 = p1.y;
			int u2 = p2.x + size * 0.5;
			int v2 = p2.y;

			int thickness = max(1.0, 3.0 * scale);
			cv::line(indicator, cv::Point(u1, v1), cv::Point(u2, v2), cv::Scalar(1), thickness);

			modelMat = glm::translate(modelMat, glm::vec3(0, length, 0));
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
String ParametricLSystem::inverse(const cv::Mat& target, cv::Mat& indicator) {
	String model(axiom, 0);
	Node* root = new Node(model);
	setUntriedActions(root);

	Node* node = root;

	for (int l = 0; l < MAX_ITERATIONS; ++l) {
		// もしノードがリーフノードなら、終了
		if (node->untriedActions.size() == 0 && node->children.size() == 0) break;

		node = UCT(node, target);
	}

	model = node->model;

	// ノードのメモリを解法
	releaseNodeMemory(root);

	return model;
}

/**
 * 指定されたmodelからUCTをスタートし、最善のoptionを返却する。
 *
 * @param model		モデル
 * @param target	ターゲット
 * @return			最善のoption
 */
Node* ParametricLSystem::UCT(Node* current_node, const cv::Mat& target) {
	for (int iter = 0; iter < NUM_MONTE_CARLO_SAMPLING; ++iter) {
		// 現在のノードのスコアが確定したら、終了
		if (current_node->fixed) break;

		Node* node = current_node;

		// 探索木のリーフノードを選択
		while (node->untriedActions.size() == 0 && node->children.size() > 0) {
			node = node->UCTSelectChild();
		}

		// 子ノードがまだ全てexpandされていない時は、1つランダムにexpand
		if (node->untriedActions.size() > 0) {
			Action action = node->randomlySelectAction();
			String child_model = action.apply(node->model);

			node = node->addChild(child_model, action);
			setUntriedActions(node);
		}

		// ランダムにderiveする
		cv::Mat indicator;
		derive(node->model, MAX_ITERATIONS_FOR_MC, indicator);

		double sc = score(indicator, target);

		// スコア確定なら、マークをつける
		if (node->untriedActions.size() == 0 && node->children.size() == 0) {
			node->fixed = true;
		}

		// スコアをbackpropagateする
		while (node != NULL) {
			node->visits++;
			node->best_score = std::max(node->best_score, sc);

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
	}

	// ベストスコアの子ノードを返却する
	double best_score = -std::numeric_limits<double>::max();
	Node* best_child = NULL;
	for (int i = 0; i < current_node->children.size(); ++i) {
		if (current_node->children[i]->best_score > best_score) {
			best_score = current_node->children[i]->best_score;
			best_child = current_node->children[i];
		}
	}

	return best_child;
}

/**
 * indicatorとターゲットとの距離を計算して返却する。
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
double ParametricLSystem::score(const cv::Mat& indicator, const cv::Mat& target) {	
	return 1.0 - sqrt(ml::mat_squared_sum(indicator - target)) / (double)indicator.rows / (double)indicator.cols;
}

void ParametricLSystem::setUntriedActions(Node* node) {
	// 展開するパラメータを決定
	int i = findNextLiteralToDefineValue(node->model);

	// 新たなderivationがないなら、終了
	if (i == -1) return;

	if (rules.find(node->model[i].c) != rules.end()) {
		for (int k = 0; k < rules[node->model[i].c].size(); ++k) {
			node->untriedActions.push_back(Action(i, rules[node->model[i].c][k]));
		}
	} else if (node->model[i].c == 'F') {
		for (int k = 0; k < 3; ++k) {
			double val = grid_size * 0.5 / (node->model[i].level + 1) * (0.8 + 0.2 * k);
			node->untriedActions.push_back(Action(i, val));
		}
	} else if (node->model[i].c == '+' || node->model[i].c == '-') {
		for (int k = 0; k < 3; ++k) {
			double val = k * 25.0 + 10.0;
			node->untriedActions.push_back(Action(i, val));
		}
	}
}

/**
 * 現在のモデルについて、値が未設定のパラメータで最もレベルが低く、最も左側に位置するindexを返却する。
 * もし未設定のパラメータがない場合は、-1を返却する。
 *
 * @param str		現在のモデル
 * @return			インデックス
 */
int ParametricLSystem::findNextLiteralToDefineValue(const String& str) {
	int min_level1 = std::numeric_limits<int>::max();
	int min_level2 = std::numeric_limits<int>::max();
	int min_index1 = -1;
	int min_index2 = -1;

	for (int i = 0; i < str.length(); ++i) {
		if (rules.find(str[i].c) != rules.end()) {
			if (str[i].level < min_level1) {
				min_level1 = str[i].level;
				min_index1 = i;
			}
		} else if ((str[i].c == 'F' || str[i].c == '+' || str[i].c == '-' || str[i].c == '\\' || str[i].c == '/') && !str[i].param_defined) {
			if (str[i].level < min_level2) {
				min_level2 = str[i].level;
				min_index2 = i;
			}
		}
	}

	if (min_level1 < min_level2) {
		return min_index1;
	} else {
		return min_index2;
	}
}

/**
 * ルールリストから、確率に基づいて１つのルールを選択する。
 * リストの各要素は、<確率、ルール>のペアとなっている。
 *
 * @param rules		ルールリスト
 * @reutnr			選択されたルール
 */
int ParametricLSystem::chooseRule(const Literal& non_terminal) {
	// ハードコーディング
	// 深さ6を超えたら、X->Fとする
	if (non_terminal.level > 6) return 0;

	// ハードコーディング
	// 深さ/6 の確率で、X->Fとする
	if (rand() % 6 <= non_terminal.level) {
		return 0;
	} else {
		return 1;
	}

	// uniform確率で、適用するルールを決定する
	//return rand() % rules[non_terminal.c].size();
}

void ParametricLSystem::releaseNodeMemory(Node* node) {
	for (int i = 0; i < node->children.size(); ++i) {
		releaseNodeMemory(node->children[i]);
	}
	delete node;
}

float deg2rad(float deg) {
	return deg * M_PI / 180.0;
}

}
