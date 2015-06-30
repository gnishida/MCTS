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
 * MCTS用のノードクラス
 */
class Node {
public:
	double max_score;	// ベストスコア
	std::vector<Node*> children;
	int num_visited;	// このノードの訪問回数
	int num_playout;
	parametriclsystem::String model;

public:
	Node();
	Node(const String& model);
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
	//void draw(const String& model, std::vector<Vertex>& vertices);
	String derive(int random_seed, cv::Mat& indicator);
	String derive(const String& start_model, int random_seed, int max_iterations, cv::Mat& indicator);
	void computeIndicator(String str, float scale, cv::Mat& indicator);
	String inverse(const cv::Mat& target, cv::Mat& indicator);
	double distance(const cv::Mat& indicator, const cv::Mat& target, double alpha = 1.0, double beta = 1.0);

	// MCTS関係のメソッド
	double uct_traverse(Node* node, const cv::Mat& target);
	double uct(Node* node, int num_playout);
	bool expand(Node* node);

private:
	int findNextLiteralToDefineValue(const String& str);
	int chooseRule(const Literal& non_terminal);
};

float deg2rad(float deg);

}
