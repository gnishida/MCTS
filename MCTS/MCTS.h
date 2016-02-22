#pragma once

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <list>
#include <map>
#include "Vertex.h"
#include <QImage>

class GLWidget3D;
class RenderManager;

namespace mcts {

	class Nonterminal {
	public:
		std::string name;
		int level;
		int dist;
		float segmentLength;
		float segmentWidth;
		float angle;
		std::vector<boost::shared_ptr<Nonterminal> > children;
		bool terminal; // trueなら、ruleは適用しない。もう確定ということ。

	public:
		Nonterminal(const std::string& name, int level, int dist, float segmentLength, float angle = 0.0f, bool terminal = false);
		boost::shared_ptr<Nonterminal> clone();
	};

	class DerivationTree {
	public:
		boost::shared_ptr<Nonterminal> root;

	public:
		DerivationTree();
		DerivationTree(const boost::shared_ptr<Nonterminal>& root);
		DerivationTree clone() const;
	};

	class State {
	public:
		DerivationTree derivationTree;
		std::list<boost::shared_ptr<Nonterminal> > queue;

	public:
		State();
		State(const boost::shared_ptr<Nonterminal>& nonterminal);
		State clone() const;
		bool applyAction(int action);
	};

	class MCTSTreeNode {
	public:
		int visits;
		float bestValue;
		float meanValue;
		bool valueFixed;
		std::vector<float> values;
		float varianceValues;
		State state;
		boost::shared_ptr<MCTSTreeNode> parent;
		std::vector<boost::shared_ptr<MCTSTreeNode> > children;
		std::vector<int> unexpandedActions;
		int selectedAction;

	public:
		MCTSTreeNode(const State& state);
		boost::shared_ptr<MCTSTreeNode> UCTSelectChild();
		int randomlySelectAction();
		boost::shared_ptr<MCTSTreeNode> bestChild();
		void addValue(float value);
	};

	class MCTS {
	private:
		cv::Mat target;
		cv::Mat targetDistMap;
		GLWidget3D* glWidget;

	public:
		MCTS(const cv::Mat& target, GLWidget3D* glWidget);

		State inverse(int maxDerivationSteps, int maxMCTSIterations);
		void randomGeneration(RenderManager* renderManager);
		State mcts(const State& state, int maxMCTSIterations);
		boost::shared_ptr<MCTSTreeNode> select(const boost::shared_ptr<MCTSTreeNode>& rootNode);
		boost::shared_ptr<MCTSTreeNode> expand(const boost::shared_ptr<MCTSTreeNode>& leafNode);
		float simulate(const boost::shared_ptr<MCTSTreeNode>& childNode);
		void backpropage(const boost::shared_ptr<MCTSTreeNode>& childNode, float value);
		float evaluate(const DerivationTree& derivationTree);
		void render(const DerivationTree& derivationTree, QImage& image);
		void generateGeometry(RenderManager* renderManager, const glm::mat4& modelMat, const boost::shared_ptr<Nonterminal>& node, std::vector<Vertex>& vertices);
	};

	std::vector<int> actions(const boost::shared_ptr<Nonterminal>& nonterminal);
	void randomDerivation(DerivationTree& derivationTree, std::list<boost::shared_ptr<Nonterminal> >& queue);
	void applyRule(DerivationTree& derivationTree, const boost::shared_ptr<Nonterminal>& node, int action, std::list<boost::shared_ptr<Nonterminal> >& queue);
	float similarity(const cv::Mat& distMap, const cv::Mat& targetDistMap, float alpha, float beta);

}