#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <map>
#include "Vertex.h"

using namespace std;

namespace parametriclsystem {

class State {
public:
	double diameter;
	glm::mat4 modelMat;
	glm::vec3 color;

public:
	State() : diameter(0.0), color(0, 0.7, 0) {};
};

class String;

class Literal {
public:
	static enum { TYPE_TERMINAL = 0, TYPE_NONTERMINAL};

public:
	string name;
	int depth;
	std::vector<double> param_values;
	bool param_defined;

public:
	Literal() {}
	Literal(const string& name, int depth, bool param_defined = false);
	Literal(const string& name, int depth, double param_value);
	Literal(const string& name, int depth, double param_value1, double param_value2);
	Literal(const string& name, int depth, double param_value1, double param_value2, double param_value3);
	Literal(const string& name, int depth, double param_value1, double param_value2, double param_value3, double param_value4);
	Literal(const string& name, int depth, double param_value1, double param_value2, double param_value3, double param_value4, double param_value5);
	Literal(const string& name, int depth, const std::vector<double>& param_values);
	
	String operator+(const Literal& l) const;
	int type();
};

class String {
public:
	vector<Literal> str;
	int cursor;				// expandするリテラルの位置
	//priority_queue<int, vector<int>, greater<int> > queue;

public:
	String();
	String(const string& str, int depth);
	String(const Literal& l);

	int length() const { return str.size(); }
	Literal operator[](int index) const { return str[index]; }
	Literal& operator[](int index) { return str[index]; }
	void operator+=(const Literal& l);
	void operator+=(const String& str);
	String operator+(const String& str) const;
	void setValue(double value);
	void replace(const String& str);

	void setExpand();
	void resetExpand();
	String getExpand() const;
	void nextCursor(int depth);
};

ostream& operator<<(ostream& os, const String& dt);

/**
 * 1ステップのderiveを表す。
 */
class Action {
public:
	static enum { ACTION_RULE = 0, ACTION_VALUE };

public:
	int type;		// 0 -- rule / 1 -- value
	int index;		// モデルの何文字目の変数に対するactionか？
	int action_index;	// actionsの中の何番目のactionか？
	String rule;
	double value;

public:
	Action() {}
	Action(int action_index, int index, const String& rule);
	Action(int action_index, int index, double value);

	String apply(const String& model);
};

ostream& operator<<(ostream& os, const Action& a);

/**
 * MCTS用のノードクラス
 */
class Node {
public:
	double best_score;	// ベストスコア
	std::vector<double> scores;
	Node* parent;
	std::vector<Node*> children;
	std::vector<Action> actions;
	std::vector<int> untriedActions;
	Action action;
	int visits;			// このノードの訪問回数
	parametriclsystem::String model;
	bool fixed;

public:
	Node(const String& model);
	Node(const String& model, const Action& action);

	Node* addChild(const String& model, const Action& action);
	void setActions(std::vector<Action>& actions);
	Action randomlySelectAction();
	void removeAction(int index);
	Node* getChild(int index);
	Node* UCTSelectChild();
	Node* bestChild();
};


class ParametricLSystem {
public:
	int grid_size;
	int indicator_data_type;
	float scale;
	int num_nodes;

	String axiom;
	map<char, vector<string> > rules;

public:
	ParametricLSystem(int grid_size, float scale, const String& axiom);
	String derive(int random_seed, cv::Mat& indicator);
	String derive(const String& start_model, int max_iterations, std::vector<int>& derivation_history);
	void computeIndicator(const String& model, float scale, const glm::mat4& baseModelMat, cv::Mat& indicator);
	String inverse(const cv::Mat& target);
	String UCT(const String& model, const cv::Mat& target, int derivation_step);
	double score(const cv::Mat& indicator, const cv::Mat& target, const cv::Mat& mask);

private:
	std::vector<Action> getActions(const String& model);
	glm::vec2 computeCurrentPoint(const String& model, float scale, glm::mat4& modelMmat);
	void releaseNodeMemory(Node* node);
};

float deg2rad(float deg);
vector<string> split(const string& str, char delim);

}
