#include "ParametricLSystem.h"
#include <iostream>
#include <time.h>
#include <algorithm>
#include "MLUtils.h"
#include "GLUtils.h"
#include <list>

#define SQR(x)	((x) * (x))


#define MAX_ITERATIONS						400//1400//200
#define MAX_ITERATIONS_FOR_MC				15
#define NUM_MONTE_CARLO_SAMPLING			100

#define PARAM_EXPLORATION					0.3 //1
#define PARAM_EXPLORATION_VARIANCE			0.1//10

#define LENGTH_ATTENUATION					0.95
#define SIZE_ATTENUATION					0.034 //0.04
#define MASK_RADIUS							12

//#define DEBUG		1

namespace parametriclsystem {

const double M_PI = 3.141592653592;

Literal::Literal(const string& name, int depth, bool param_defined) {
	this->name = name;
	this->depth = depth;
	this->param_defined = param_defined;
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

int Literal::type() {
	if (name == "F" || name == "f" || name == "[" || name == "]" || name == "+" || name == "-" || name == "\\" || name == "/" || name == "&" || name == "^" || name == "#") {
		return TYPE_TERMINAL;
	} else {
		return TYPE_NONTERMINAL;
	}
}

String::String() {
	this->cursor = -1;
}

String::String(const string& str, int depth) {
	this->str.push_back(Literal(str, depth));
	this->cursor = 0;
}

String::String(const Literal& l) {
	this->str.push_back(l);
	this->cursor = 0;
}

void String::operator+=(const Literal& l) {
	str.push_back(l);

	if (cursor < 0) cursor = 0;
}

void String::operator+=(const String& str) {
	for (int i = 0; i < str.length(); ++i) {
		this->str.push_back(str[i]);
	}

	if (cursor < 0) cursor = 0;
}

String String::operator+(const String& str) const {
	String new_str = *this;

	for (int i = 0; i < str.length(); ++i) {
		new_str.str.push_back(str[i]);
	}

	if (new_str.cursor < 0 && new_str.length() > 0) new_str.cursor = 0;

	return new_str;
}

void String::setValue(double value) {
	str[cursor].param_values.push_back(value);
	str[cursor].param_defined = true;

	cursor++;

	// 次のリテラルを探す
	nextCursor(str[cursor].depth);
}

void String::replace(const String& str) {
	int depth = this->str[cursor].depth;

	this->str.erase(this->str.begin() + cursor);
	this->str.insert(this->str.begin() + cursor, str.str.begin(), str.str.end());

	// 次のリテラルを探す
	nextCursor(depth);
}

String String::getExpand() const {
	String ret;

	int nest = 0;
	for (int i = cursor; i < str.size(); ++i) {
		if (str[i].name == "[") {
			nest++;
		} else if (str[i].name == "]") {
			nest--;
		}

		if (nest < 0) break;

		ret += str[i];
	}

	return ret;
}

void String::nextCursor(int depth) {
	for (int i = cursor; i < str.size(); ++i) {
		if (str[i].depth != depth) continue;

		if (str[i].type() == Literal::TYPE_NONTERMINAL) {
			cursor = i;
			return;
		} else if (str[i].type() == Literal::TYPE_TERMINAL && !str[i].param_defined) {
			cursor = i;
			return;
		}
	}

	// 同じdepthでリテラルが見つからない場合は、depth+1にする
	depth++;

	for (int i = 0; i < str.size(); ++i) {
		if (str[i].depth != depth) continue;

		if (str[i].type() == Literal::TYPE_NONTERMINAL) {
			cursor = i;
			return;
		} else if (str[i].type() == Literal::TYPE_TERMINAL && !str[i].param_defined) {
			cursor = i;
			return;
		}
	}

	// リテラルが見つからない場合は、-1にする
	cursor = -1;
}

ostream& operator<<(ostream& os, const String& str) {
	os << setprecision(1);
	for (int i = 0; i < str.length(); ++i) {
		os << str[i].name;
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
 *
 * @param model					モデル
 * @return						action適用した後のモデル
 */
String Action::apply(const String& model) {
	String new_model = model;

	if (type == ACTION_RULE) {
		new_model.replace(rule);
	} else {
		new_model.setValue(value);
	}

	return new_model;
}

ostream& operator<<(ostream& os, const Action& a) {
	os << "type(";
	if (a.type == Action::ACTION_RULE) {
		os << "Rule" << "): " << a.rule;
	} else {
		os << "Value" << "): " << a.value;
	}

	return os;
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

ParametricLSystem::ParametricLSystem(int grid_size, float scale, const String& axiom) {
	this->grid_size = grid_size;
	this->scale = scale;
	this->axiom = axiom;
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
	String result_model = derive(axiom, MAX_ITERATIONS, derivation_history);

	computeIndicator(result_model, scale, glm::mat4(), indicator);

	return result_model;
}

/**
 * 指定された開始モデルからスタートし、ランダムにgenerateする。
 *
 * @param start_model			開始モデル
 * @param max_iterations		繰り返し数
 * @param build_tree			木を生成するか？
 * @param indicator [OUT]		生成されたモデルのindicator
 * @return						生成されたモデル
 */
String ParametricLSystem::derive(const String& start_model, int max_iterations, std::vector<int>& derivation_history) {
	String model = start_model;

	for (int iter = 0; iter < max_iterations; ++iter) {
		std::vector<Action> actions = getActions(model);
		if (actions.size() == 0) break;

		int index = ml::genRand(0, actions.size());
		derivation_history.push_back(index);
		model = actions[index].apply(model);
	}

	return model;
}

/**
 * 指定された文字列に基づいてモデルを生成し、indicatorを計算して返却する。
 * 
 * @param rule				モデルを表す文字列
 * @param scale				grid_size * scaleのサイズでindicatorを計算する
 * @param indicator [OUT]	indicator
 */
void ParametricLSystem::computeIndicator(const String& model, float scale, const glm::mat4& baseModelMat, cv::Mat& indicator) {
	int size = grid_size * scale;

	indicator = cv::Mat::zeros(size, size, CV_32F);

	std::list<glm::mat4> stack;

	glm::mat4 modelMat = baseModelMat;

	for (int i = 0; i < model.length(); ++i) {
		if (!model[i].param_defined) break;

		if (model[i].name == "[") {
			stack.push_back(modelMat);
		} else if (model[i].name == "]") {
			if (!stack.empty()) {
				modelMat = stack.back();
				stack.pop_back();
			}
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
		} else if (model[i].name == "&" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_values[0]), glm::vec3(1, 0, 0));
		} else if (model[i].name == "^" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(-model[i].param_values[0]), glm::vec3(1, 0, 0));
		} else if (model[i].name == "f" && model[i].param_defined) {
			double length = model[i].param_values[0] * scale;
			modelMat = glm::translate(modelMat, glm::vec3(0, 0, length));
		} else if (model[i].name == "F" && model[i].param_defined) {
			double length = model[i].param_values[0] * scale;
			double radius1 = model[i].param_values[1] * scale;
			double radius2 = (model[i].param_values[1] -  model[i].param_values[0] * SIZE_ATTENUATION) * scale;
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

			//if (radius > 0.1f) {
				int thickness = max(1.0, radius);
				cv::line(indicator, cv::Point(u1, v1), cv::Point(u2, v2), cv::Scalar(1), thickness);
			//}

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
	// UCTを使って探索木を構築していく
	String model = axiom;

	for (int l = 0; l < MAX_ITERATIONS; ++l) {
		model = UCT(model, target, l);

		cv::Mat indicator;
		computeIndicator(model, scale, glm::mat4(), indicator);
		double sc = score(indicator, target, cv::Mat::ones(target.size(), CV_8U));

		/////// デバッグ ///////
		/*
		char filename[256];
		sprintf(filename, "indicator_%d.png", l);
		computeIndicator(model, 4, glm::mat4(), indicator);

		cv::Mat target2;
		cv::resize(target, target2, cv::Size(400, 400));

		ml::mat_save(filename, indicator + target2 * 0.4);
		*/
		/////// デバッグ ///////

		//cout << l << ": " << "Best score=" << sc << " : " << model << endl;
		cout << l << ": " << "Best score=" << sc << " : " << endl;

		// これ以上、derivationできなら、終了
		if (model.cursor < 0) break;
	}

	// スコア表示
	cv::Mat indicator;
	computeIndicator(model, scale, glm::mat4(), indicator);
	cout << score(indicator, target, cv::Mat::ones(target.size(), CV_8U)) << endl;
	
	return model;
}

/**
 * 指定されたmodelからUCTをスタートし、最善のoptionを返却する。
 *
 * @param model		モデル
 * @param target	ターゲット
 * @return			最善のoption
 */
String ParametricLSystem::UCT(const String& current_model, const cv::Mat& target, int derivation_step) {
	// これ以上、derivationできなら、終了
	int index = current_model.cursor;
	if (index < 0) return current_model;

	// expandするリテラルを取得する
	String model = current_model.getExpand();

	// 現在のカーソルのdepthを取得
	int depth = current_model[current_model.cursor].depth;

	// 現在の座標を計算
	glm::mat4 baseModelMat;
	glm::vec2 curPt = computeCurrentPoint(current_model, scale, baseModelMat);

	// マスク画像を作成
	cv::Mat mask = ml::create_mask(target.rows, target.cols, CV_8U, cv::Point(curPt.x, curPt.y), MASK_RADIUS);
	cv::Mat mask2 = ml::create_mask(target.rows * 4, target.cols * 4, CV_8U, cv::Point(curPt.x * 4, curPt.y * 4), MASK_RADIUS * 4);
	
	// ルートノードを作成
	Node* current_node = new Node(model);
	current_node->setActions(getActions(model));

	// ベースとなるindicatorを計算
	cv::Mat baseIndicator;
	computeIndicator(current_model, scale, glm::mat4(), baseIndicator);

	for (int iter = 0; iter < NUM_MONTE_CARLO_SAMPLING; ++iter) {
		// もしノードがリーフノードなら、終了
		if (current_node->untriedActions.size() == 0 && current_node->children.size() == 0) break;

		// 現在のノードのスコアが確定したら、終了
		if (current_node->fixed) break;

		Node* node = current_node;

		// 探索木のリーフノードを選択
		while (node->untriedActions.size() == 0 && node->children.size() > 0) {
			node = node->UCTSelectChild();
		}

		// 子ノードがまだ全てexpandされていない時は、1つランダムにexpand
		if (node->untriedActions.size() > 0) {// && node->children.size() <= ml::log((double)iter * 0.01 + 1, 1.4)) {
			Action action = node->randomlySelectAction();
			String child_model = action.apply(node->model);
						
			node = node->addChild(child_model, action);
			node->setActions(getActions(child_model));
			num_nodes++;
		}

		// ランダムにderiveする
		std::vector<int> derivation_history;
		String result_model = derive(node->model, MAX_ITERATIONS_FOR_MC, derivation_history);

		// indicatorを計算する
		cv::Mat indicator;
		computeIndicator(result_model, scale, baseModelMat, indicator);
		indicator += baseIndicator;

		// スコアを計算する
		double sc = score(indicator, target, mask);
		node->best_score = sc;


		/////// デバッグ ///////
		/*
		char filename[256];
		sprintf(filename, "images/indicator_%d_%d_%d_%lf.png", derivation_step, iter, action_index, sc);
		cv::Mat img = indicator + target * 0.4;
		img = ml::mat_mask(img, mask, 0.7);

		ml::mat_save(filename, img);

		cout << "   " << filename << " : " << result_model << endl;
		*/
		/////// デバッグ ///////



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


	Action best_action = current_node->bestChild()->action;
	String best_model = best_action.apply(current_model);



	/////// デバッグ ///////
	/*
	char filename[256];
	computeIndicator(best_model, 4, glm::mat4(), bestIndicator);
	sprintf(filename, "images/indicator_%d.png", derivation_step);

	cv::Mat target2;
	cv::resize(target, target2, cv::Size(400, 400));
	cv::Mat img = bestIndicator + target2 * 0.4;
	img = ml::mat_mask(img, mask2, 0.8);

	ml::mat_save(filename, img);
	*/
	/////// デバッグ ///////



	// 探索木のメモリを解放する
	releaseNodeMemory(current_node);
	
	return best_model;
}

/**
 * indicatorのスコアを計算して返却する。
 *
 * @param indicator		indicator
 * @param target		ターゲットindicator
 * @return				距離
 */
double ParametricLSystem::score(const cv::Mat& indicator, const cv::Mat& target, const cv::Mat& mask) {
	cv::Mat result;
	cv::subtract(indicator, target, result, mask);
	double count = ml::mat_squared_sum(result);

	cv::Mat result2;
	cv::subtract(target, cv::Mat::zeros(target.size(), target.type()), result2, mask);
	double base = ml::mat_squared_sum(result2);

	if (base > 0.0) {
		return 1.0 - count / base;
	} else {
		if (count > 0.0) {
			return -1.0;
		} else {
			return 1.0;
		}
	}
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
	int i = model.cursor;

	// 新たなderivationがないなら、終了
	if (i == -1) return actions;

	if (model[i].name == "X") {
		//String rule = Literal("F", model[i].depth + 1, 0, 0);
		String rule;
		actions.push_back(Action(0, i, rule));

		if (model[i].param_values[0] >= 0.01f) {
			String rule = Literal("F", model[i].depth + 1, model[i].param_values[0], model[i].param_values[1])
				+ Literal("#", model[i].depth + 1)
				+ Literal("X", model[i].depth + 1, model[i].param_values[0] * LENGTH_ATTENUATION, model[i].param_values[1] - model[i].param_values[0] * SIZE_ATTENUATION);
			actions.push_back(Action(1, i, rule));

			rule = Literal("F", model[i].depth + 1, model[i].param_values[0] * 0.5f, model[i].param_values[1])
				+ Literal("[", model[i].depth + 1, true)
				+ Literal("+", model[i].depth + 1)
				+ Literal("X", model[i].depth + 1, model[i].param_values[0] * LENGTH_ATTENUATION, model[i].param_values[1] * LENGTH_ATTENUATION)
				+ Literal("]", model[i].depth + 1, true)
				+ Literal("F", model[i].depth + 1, model[i].param_values[0] * 0.5f, model[i].param_values[1] - model[i].param_values[0] * SIZE_ATTENUATION * 0.5f)
				+ Literal("#", model[i].depth + 1)
				+ Literal("X", model[i].depth + 1, model[i].param_values[0] * LENGTH_ATTENUATION, model[i].param_values[1] - model[i].param_values[0] * SIZE_ATTENUATION);
			actions.push_back(Action(2, i, rule));
		}
	} else if (model[i].name == "-" || model[i].name == "+") {
		int count = 0;
		for (int k = -80; k <= 80; k += 20, ++count) {
			if (k == 0) continue;
			actions.push_back(Action(count, i, k));
		}
	} else if (model[i].name == "#") {
		int count = 0;
		for (int k = -20; k <= 20; k += 10, ++count) {
			actions.push_back(Action(count, i, k));
		}
	} else if (model[i].name == "\\") {
		int count = 0;
		//actions.push_back(Action(count, i, 180));
		for (int k = 0; k <= 180; k += 30, ++count) {
			actions.push_back(Action(count, i, k));
		}
	}

	return actions;
}

glm::vec2 ParametricLSystem::computeCurrentPoint(const String& model, float scale, glm::mat4& modelMat) {
	int size = grid_size * scale;

	std::list<glm::mat4> stack;

	for (int i = 0; i < model.cursor; ++i) {
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
		} else if (model[i].name == "&" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(model[i].param_values[0]), glm::vec3(1, 0, 0));
		} else if (model[i].name == "^" && model[i].param_defined) {
			modelMat = glm::rotate(modelMat, deg2rad(-model[i].param_values[0]), glm::vec3(1, 0, 0));
		} else if (model[i].name == "f" && model[i].param_defined) {
			double length = model[i].param_values[0] * scale;
			modelMat = glm::translate(modelMat, glm::vec3(0, 0, length));
		} else if (model[i].name == "F" && model[i].param_defined) {
			double length = model[i].param_values[0] * scale;
			modelMat = glm::translate(modelMat, glm::vec3(0, 0, length));
		} else if (model[i].name == "X") {
		} else {
		}
	}

	glm::vec4 p(0, 0, 0, 1);
	p = modelMat * p;

	return glm::vec2(p.x + grid_size * 0.5, p.z);
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
