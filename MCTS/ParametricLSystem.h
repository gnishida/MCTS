#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <map>

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

class TreeNode {
public:
	static int seq;

public:
	int id;
	Literal l;
	String model;
	TreeNode* parent;
	map<double, TreeNode*> children;
	int num_leaves;
	cv::Mat indicator;

public:
	TreeNode(const Literal& l, TreeNode* parent);
	TreeNode* getChild(char c, int level, double value);
};


class ParametricLSystem {
public:
	int grid_size;
	int indicator_data_type;
	float scale;

	string axiom;
	map<char, vector<string> > rules;
	TreeNode* root;

public:
	ParametricLSystem();
	void draw(const String& model);
	String derive(int random_seed, cv::Mat& indicator);
	String derive(const String& start_model, int random_seed, int max_iterations, bool build_tree, cv::Mat& indicator);
	String derive(const String& start_model, int max_iterations, const cv::Mat& target, cv::Mat& indicator);
	void drawTree(int max_depth);
	void gatherIndicators(int gather_type);
	void saveIndicatorImages(int max_depth, float min_threshold);
	void computeIndicator(String str, cv::Mat& indicator);
	void estimateIndicator(const String start_model, int max_iterations, float scale, const cv::Mat& target, cv::Mat& indicator);
	String inverse(const cv::Mat& target, cv::Mat& indicator);
	double distance(const cv::Mat& indicator, const cv::Mat& target, double alpha = 1.0, double beta = 1.0);

private:
	int findNextLiteralToDefineValue(const String& str);
	void drawSubTree(TreeNode* node, int depth, int max_depth);
	int countLeaves(TreeNode* node);
	cv::Mat gatherSubIndicators(TreeNode* node, int gather_type);
	void saveSubIndicatorImages(TreeNode* node, int depth, int max_depth, float min_threshold);
	TreeNode* traverseTree(TreeNode* node, const cv::Mat& target);
	int chooseRule(const Literal& non_terminal);
};

float deg2rad(float deg);

}
