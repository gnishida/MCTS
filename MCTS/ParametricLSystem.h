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

class Literal {
public:
	char c;
	int level;
	double param_value;
	bool param_defined;

public:
	Literal() {}
	Literal(char c, int level);
	Literal(char c, int level, double param_value);
};

class String {
public:
	vector<Literal> str;

public:
	String() {}
	String(string str, int level);
	String(Literal l);

	int length() const { return str.size(); }
	Literal operator[](int index) const { return str[index]; }
	Literal& operator[](int index) { return str[index]; }
	void operator+=(const Literal& l) { str.push_back(l); }
	void operator+=(const String& str);
	String operator+(const String& str) const;
	void replace(int index, const String& str);
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
	string rule;
	double value;

public:
	Action() {}
	Action(int index, const string& rule);
	Action(int index, double value);

	String apply(const String& model);
};

/**
 * MCTS用のノードクラス
 */
class Node {
public:
	double best_score;	// ベストスコア
	Node* parent;
	std::vector<Node*> children;
	std::vector<Action> untriedActions;
	Action action;
	int visits;			// このノードの訪問回数
	parametriclsystem::String model;
	bool fixed;

public:
	Node(const String& model);
	Node(const String& model, const Action& action);

	Node* addChild(const String& model, const Action& action);
	Action randomlySelectAction();
	Node* UCTSelectChild();
};


class ParametricLSystem {
public:
	int grid_size;
	int indicator_data_type;
	float scale;

	string axiom;
	map<char, vector<string> > rules;

public:
	ParametricLSystem(int grid_size, float scale);
	void draw(const String& model, std::vector<Vertex>& vertices);
	String derive(int random_seed, cv::Mat& indicator);
	String derive(const String& start_model, int max_iterations, cv::Mat& indicator);
	void computeIndicator(String str, float scale, cv::Mat& indicator);
	String inverse(const cv::Mat& target, cv::Mat& indicator);
	Node* UCT(Node* current_node, const cv::Mat& target);
	double distance(const cv::Mat& indicator, const cv::Mat& target, double alpha = 1.0, double beta = 1.0);
	double score(const cv::Mat& indicator, const cv::Mat& target);

private:
	void setUntriedActions(Node* node);
	int findNextLiteralToDefineValue(const String& str);
	int chooseRule(const Literal& non_terminal);
	void releaseNodeMemory(Node* node);
};

float deg2rad(float deg);

}
