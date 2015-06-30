#include "ParametricLSystem.h"
#include <iostream>
#include <time.h>
#include <algorithm>
#include "MLUtils.h"
#include "GLUtils.h"
#include <list>

#define MAX_ITERATIONS						200
#define MAX_ITERATIONS_FOR_MC				50
#define NUM_MONTE_CARLO_SAMPLING			200

#define ALPHA								1.0
#define BETA								1.0

#define CREATE_NEW_NODE_LIMIT				10000
#define PARAM_EXPLORATION					10//1.4142

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

Node::Node() {
	max_score = -std::numeric_limits<double>::max();
	num_visited = 0;
	num_playout = 0;
}

Node::Node(const String& model) {
	this->model = model;
	max_score = -std::numeric_limits<double>::max();
	num_visited = 0;
	num_playout = 0;
}

ParametricLSystem::ParametricLSystem(int grid_size, float scale) {
	this->grid_size = grid_size;
	this->scale = scale;

	axiom = "X";
	rules['X'].push_back("F");
	rules['X'].push_back("F[-X][+X]");
}

/*void ParametricLSystem::draw(const String& model, std::vector<Vertex>& vertices) {
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
}*/

/**
 * 適当な乱数シードに基づいて、ランダムにgenerateする。
 *
 * @param random_seed		乱数シード
 * @return					生成されたモデル
 */
String ParametricLSystem::derive(int random_seed, cv::Mat& indicator) {
	return derive(String(axiom, 0), random_seed, MAX_ITERATIONS, indicator);
}

/**
 * 指定された開始モデルからスタートし、適当な乱数シードに基づいて、ランダムにgenerateする。
 *
 * @param start_model		開始モデル
 * @param random_seed		乱数シード
 * @param max_iterations	繰り返し数
 * @param build_tree		木を生成するか？
 * @param indicator [OUT]	生成されたモデルのindicator
 * @return					生成されたモデル
 */
String ParametricLSystem::derive(const String& start_model, int random_seed, int max_iterations, cv::Mat& indicator) {
	ml::initRand(random_seed);

	String result = start_model;

	//for (int iter = 0; iter < max_iterations; ++iter) {
	for (int iter = 0; ; ++iter) {
		// 展開するパラメータを決定
		int i = findNextLiteralToDefineValue(result);

		// 新たなderivationがないなら、終了
		if (i == -1) break;

		if (rules.find(result[i].c) != rules.end()) {
			int index = chooseRule(result[i]);

			// もしmax_levelを超えたら、終了する
			if (iter >= max_iterations) {
				break;
			}

			result.replace(i, String(rules[result[i].c][index], result[i].level + 1));
		} else if (result[i].c == 'F') {
			double mean_val = grid_size * 0.5 / pow(1.5, result[i].level);
			double val = ml::genRandInt(mean_val * 0.8, mean_val * 1.2, 3);
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
 * @param threshold			しきい値
 * @param indicator [OUT]	生成されたモデルのindicator
 * @return					生成されたモデル
 */
String ParametricLSystem::inverse(const cv::Mat& target, cv::Mat& indicator) {
	// ルートノードの作成
	Node* root = new Node(String(axiom, 0));

	Node* node = root;

	for (int l = 0; l < MAX_ITERATIONS; ++l) {
		// 子ノードがまだない場合は、expand
		if (node->children.size() == 0) {
			if (!expand(node)) break;
		}

		// playoutを実施
		for (int iter = 0; iter < NUM_MONTE_CARLO_SAMPLING; ++iter) {
			uct_traverse(node, target);
		}

		// ハイスコアの子ノードを選択
		double max_score = 0;
		Node* selected_child = NULL;
		for (int i = 0; i < node->children.size(); ++i) {
			if (node->children[i]->max_score > max_score) {
				max_score = node->children[i]->max_score;
				selected_child = node->children[i];
			}
		}
		node = selected_child;

		cout << l << ": " << node->max_score << endl;
	}

	computeIndicator(node->model, scale, indicator);

	return node->model;
}

/**
 * indicatorとターゲットとの距離を計算して返却する。
 *
 * @param indicator		indicator
 * @param target		ターゲットindicator
 * @param threshold		しきい値
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

	return dist;
}

/**
 * 指定されたノードから、UCTに基づいて1つの子ノードを選択し、再帰的にtraverseし、探索木を構築していく。
 */
double ParametricLSystem::uct_traverse(Node* node, const cv::Mat& target) {
	// UCT値が最大のノードを選択する
	double max_uct = 0;
	Node* selected_child = NULL;

	// UCTアルゴリズムで、子ノードを1つ選択する
	for (int i = 0; i < node->children.size(); ++i) {
		float uct_value = uct(node->children[i], node->num_playout);
		if (uct_value > max_uct) {
			max_uct = uct_value;
			selected_child = node->children[i];
		}
	}

	// 子ノードへの訪問数がしきい値を超えたら、子ノードを探索木に追加する
	if (selected_child != NULL && selected_child->num_visited >= CREATE_NEW_NODE_LIMIT) {
		expand(selected_child);
	}

	double score;
	if (selected_child->children.size() == 0) {
		cv::Mat indicator;
		derive(selected_child->model, (int)ml::genRand(0, 10000), MAX_ITERATIONS_FOR_MC, indicator);

		score = 10 * exp(-distance(indicator, target, ALPHA, BETA) * 0.01);
	} else {
		score = uct_traverse(selected_child, target);
	}
	
	selected_child->max_score = std::max(selected_child->max_score, score);
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
double ParametricLSystem::uct(Node* node, int num_playout) {
	if (node->num_visited == 0) {
		return 10000 + ml::genRand(0, 100);
	} else {
		return node->max_score + PARAM_EXPLORATION * sqrt(log((double)num_playout) / node->num_visited);
	}
}

/**
 * 指定されたノードの子ノードを全て追加する。
 *
 * @param node		ノード
 * @return			true - 成功 / false - ノードがリーフノードのため、これ以上展開できない
 */
bool ParametricLSystem::expand(Node* node) {
	// 展開するパラメータを決定
	int i = findNextLiteralToDefineValue(node->model);

	// 新たなderivationがないなら、終了
	if (i == -1) return false;

	if (rules.find(node->model[i].c) != rules.end()) {
		for (int k = 0; k < rules[node->model[i].c].size(); ++k) {
			// この値を選択した時のモデルを作成
			String model = node->model;
			model.replace(i, String(rules[model[i].c][k], model[i].level + 1));

			// 子ノードを作成
			Node* child = new Node(model);
			node->children.push_back(child);
		}
	} else if (node->model[i].c == 'F') {
		for (int k = 0; k < 3; ++k) {
			double mean_val = grid_size * 0.5 / pow(1.5, node->model[i].level);
			double val = mean_val * (0.8 + k * 0.2);

			// この値を選択した時のモデルを生成
			String model = node->model;
			model[i] = Literal(model[i].c, model[i].level, val);

			// 子ノードを作成
			Node* child = new Node(model);
			node->children.push_back(child);
		}
	} else if (node->model[i].c == '-' || node->model[i].c == '+') {
		for (int k = 0; k < 3; ++k) {
			double val = k * 25.0 + 10.0;

			// この値を選択した時のモデルを生成
			String model = node->model;
			model[i] = Literal(model[i].c, model[i].level, val);

			// 子ノードを作成
			Node* child = new Node(model);
			node->children.push_back(child);
		}
	}

	return true;
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
	// uniform確率で、適用するルールを決定する
	return rand() % rules[non_terminal.c].size();

	/*
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
	*/


	//return rand() % rules[non_terminal.c].size();
}

float deg2rad(float deg) {
	return deg * M_PI / 180.0;
}

}
