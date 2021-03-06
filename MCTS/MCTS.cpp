﻿#include "MCTS.h"
#include "GLWidget3D.h"
#include "GLUtils.h"
#include <QDir>
#include <QTextStream>
#include <time.h>

namespace mcts {
	const double PARAM_EXPLORATION = 1.0;
	const double PARAM_EXPLORATION_VARIANCE = 0.1;
	const float M_PI = 3.141592653f;
	const float INITIAL_SEGMENT_LENGTH = 0.5f;
	const float INITIAL_SEGMENT_WIDTH = 0.3f;
	const int MAX_LEVEL = 3;
	const int MAX_DIST = 20;
	const float SIMILARITY_METRICS_ALPHA = 10000.0f;
	const float SIMILARITY_METRICS_BETA = 5000.0f;
	const int BASE_PART = 3;
	const int SIMULATION_DEPTH = 2;

	float time_select = 0.0f;
	float time_expand = 0.0f;
	float time_simulate = 0.0f;
	float time_backpropagate = 0.0f;

	Nonterminal::Nonterminal(const std::string& name, int level, int dist, float segmentLength, float angle, bool terminal) {
		this->name = name;
		this->level = level;
		this->dist = dist;
		this->segmentLength = segmentLength;
		this->segmentWidth = INITIAL_SEGMENT_WIDTH;
		this->angle = angle;
		this->terminal = terminal;
	}

	boost::shared_ptr<Nonterminal> Nonterminal::clone() {
		boost::shared_ptr<Nonterminal> newNonterminal = boost::shared_ptr<Nonterminal>(new Nonterminal(name, level, dist, segmentLength, angle, terminal));
		for (int i = 0; i < children.size(); ++i) {
			newNonterminal->children.push_back(children[i]->clone());
		}
		return newNonterminal;
	}

	DerivationTree::DerivationTree() {
		root = NULL;
	}

	DerivationTree::DerivationTree(const boost::shared_ptr<Nonterminal>& root) {
		this->root = root;
	}

	DerivationTree DerivationTree::clone() const {
		if (root != NULL) {
			return DerivationTree(root->clone());
		}
		else {
			return DerivationTree();
		}
	}

	State::State() {
	}

	State::State(const boost::shared_ptr<Nonterminal>& root) {
		derivationTree = DerivationTree(root);
		this->queue.push_back(root);
	}

	State State::clone() const {
		// derivationTreeは、コピーコンストラクタにより、クローンを生成
		State newState;
		newState.derivationTree = derivationTree.clone();

		std::list<boost::shared_ptr<Nonterminal> > queue;
		queue.push_back(newState.derivationTree.root);
		while (!queue.empty()) {
			boost::shared_ptr<Nonterminal> node = queue.front();
			queue.pop_front();

			if (!node->terminal) {
				newState.queue.push_back(node);
			}

			for (int i = 0; i < node->children.size(); ++i) {
				queue.push_back(node->children[i]);
			}
		}

		return newState;
	}

	bool State::applyAction(int action) {
		if (queue.empty()) return false;

		boost::shared_ptr<Nonterminal> node = queue.front();
		queue.pop_front();

		if (node->terminal) return false;

		applyRule(derivationTree, node, action, queue);

		return true;
	}

	MCTSTreeNode::MCTSTreeNode(const State& state) {
		visits = 0;
		bestValue = 0;
		meanValue = 0;
		valueFixed = false;
		varianceValues = 0;
		this->state = state;
		parent = NULL;

		if (!state.queue.empty()) {
			// queueが空でない場合、先頭のnon-terminalに基づいて、unexpandedActionsを設定する
			this->unexpandedActions = actions(state.queue.front());
			/*
			if (state.queue.front()->name == "X") {
				if (state.queue.front()->dist >= MAX_DIST - 1) { //末端は、ストップ
					this->unexpandedActions.push_back(0);
				}
				else if (state.queue.front()->dist < BASE_PART) { // 根元は、延伸のみ
					this->unexpandedActions.push_back(1);
				}
				else { // 中間部分は、延伸または枝分かれ
					this->unexpandedActions.push_back(1);
					if (state.queue.front()->level < MAX_LEVEL - 1) {
						this->unexpandedActions.push_back(2);
					}
				}
			}
			else if (state.queue.front()->name == "/") {
				for (int i = 0; i < 5; ++i) {
					this->unexpandedActions.push_back(i);
				}
			}
			else if (state.queue.front()->name == "\\") {
				for (int i = 0; i < 8; ++i) {
					this->unexpandedActions.push_back(i);
				}
			}
			*/
		}
	}

	boost::shared_ptr<MCTSTreeNode> MCTSTreeNode::UCTSelectChild() {
		double max_uct = -std::numeric_limits<double>::max();
		boost::shared_ptr<MCTSTreeNode> bestChild = NULL;

		for (int i = 0; i < children.size(); ++i) {
			// スコアが確定済みの子ノードは探索対象外とする
			if (children[i]->valueFixed) continue;

			double uct;
			if (children[i]->visits == 0) {
				uct = 10000 + rand() % 1000;
			}
			else {
				uct = children[i]->bestValue
					+ PARAM_EXPLORATION * sqrt(2 * log((double)visits) / (double)children[i]->visits);
					//+ PARAM_EXPLORATION_VARIANCE * sqrt(children[i]->varianceValues + 0.0 / (double)children[i]->visits);
			}

			if (uct > max_uct) {
				max_uct = uct;
				bestChild = children[i];
			}
		}

		return bestChild;
	}

	boost::shared_ptr<MCTSTreeNode> MCTSTreeNode::bestChild() {
		double bestValue = -std::numeric_limits<double>::max();
		boost::shared_ptr<MCTSTreeNode> bestChild = NULL;

		for (int i = 0; i < children.size(); ++i) {
			if (children[i]->bestValue > bestValue) {
				bestValue = children[i]->bestValue;
				bestChild = children[i];
			}
		}

		return bestChild;
	}

	void MCTSTreeNode::addValue(float value) {
		values.push_back(value);
		if (value > bestValue) {
			bestValue = value;
		}

		// compute the variance
		float total_val = 0.0f;
		float total_val2 = 0.0f;
		for (int i = 0; i < values.size(); ++i) {
			total_val = values[i];
			total_val2 = values[i] * values[i];
		}

		meanValue = total_val / values.size();
		varianceValues = total_val2 / values.size() - meanValue * meanValue;
	}

	int MCTSTreeNode::randomlySelectAction() {
		int index = rand() % unexpandedActions.size();
		int action = unexpandedActions[index];
		unexpandedActions.erase(unexpandedActions.begin() + index);
		return action;
	}

	MCTS::MCTS(const cv::Mat& target, GLWidget3D* glWidget) {
		this->target = target;
		this->glWidget = glWidget;

		// compute a distance map
		cv::Mat grayImage;
		cv::cvtColor(target, grayImage, CV_RGB2GRAY);
		cv::distanceTransform(grayImage, targetDistMap, CV_DIST_L2, 3);

		////////////////////////////////////////////// DEBUG //////////////////////////////////////////////
		//cv::imwrite("targetDistMap.png", targetDistMap);
		////////////////////////////////////////////// DEBUG //////////////////////////////////////////////

		targetDistMap.convertTo(targetDistMap, CV_32F);
	}

	State MCTS::inverse(int maxDerivationSteps, int maxMCTSIterations) {
		// initialize computation time
		time_select = 0.0f;
		time_expand = 0.0f;
		time_simulate = 0.0f;
		time_backpropagate = 0.0f;

		if (QDir("results").exists()) {
			QDir("results").removeRecursively();
		}

		State state(boost::shared_ptr<Nonterminal>(new Nonterminal("X", 0, 0, INITIAL_SEGMENT_LENGTH)));

		for (int iter = 0; iter < maxDerivationSteps; ++iter) {
			state = mcts(state, maxMCTSIterations);

			////////////////////////////////////////////// DEBUG //////////////////////////////////////////////
			if (!QDir("results/").exists())	QDir().mkpath("results/");

			QString filename = QString("results/result_%1.png").arg(iter);
			QImage image;
			render(state.derivationTree, image);
			cv::Mat backMat = target.clone();
			QImage background(backMat.data, backMat.cols, backMat.rows, backMat.step, QImage::Format_RGB888);
			QPainter painter(&background);
			painter.setOpacity(0.5);
			painter.drawImage(0, 0, image);
			background.save(filename);
			////////////////////////////////////////////// DEBUG //////////////////////////////////////////////

			// これ以上derivationできない場合は、終了
			if (state.queue.empty()) break;
		}

		// show compuattion time
		std::cout << "Select: " << time_select << std::endl;
		std::cout << "Expand: " << time_expand << std::endl;
		std::cout << "Simulate: " << time_simulate << std::endl;
		std::cout << "Back: " << time_backpropagate << std::endl;


		return state;
	}

	void MCTS::randomGeneration(RenderManager* renderManager) {
		State state(boost::shared_ptr<Nonterminal>(new Nonterminal("X", 0, 0, INITIAL_SEGMENT_LENGTH)));
		randomDerivation(state.derivationTree, state.queue);

		glWidget->renderManager.removeObjects();
		std::vector<Vertex> vertices;
		generateGeometry(renderManager, glm::mat4(), state.derivationTree.root, vertices);
		glWidget->renderManager.addObject("tree", "", vertices, true);
	}

	State MCTS::mcts(const State& state, int maxMCTSIterations) {
		boost::shared_ptr<MCTSTreeNode> rootNode = boost::shared_ptr<MCTSTreeNode>(new MCTSTreeNode(state));
		for (int iter = 0; iter < maxMCTSIterations; ++iter) {
			// MCTS selection
			time_t start = clock();
			boost::shared_ptr<MCTSTreeNode> LeafNode = select(rootNode);
			time_t end = clock();
			time_select += (double)(end - start) / CLOCKS_PER_SEC;

			// MCTS expansion
			start = clock();
			boost::shared_ptr<MCTSTreeNode> childNode = expand(LeafNode);
			end = clock();
			time_expand += (double)(end - start) / CLOCKS_PER_SEC;

			// MCTS simulation
			start = clock();
			float value = simulate(childNode);
			end = clock();
			time_simulate += (double)(end - start) / CLOCKS_PER_SEC;

			// MCTS backpropagation
			start = clock();
			backpropage(childNode, value);
			end = clock();
			time_backpropagate += (double)(end - start) / CLOCKS_PER_SEC;

			// 子ノードが1個なら、終了
			if (rootNode->unexpandedActions.size() == 0 && rootNode->children.size() <= 1) break;
		}

		////////////////////////////////////////////// DEBUG //////////////////////////////////////////////
		if (!QDir("results/").exists())	QDir().mkpath("results/");
		QFile file("results/visits.txt");
		file.open(QIODevice::Append);
		QTextStream out(&file);
		for (int i = 0; i < rootNode->children.size(); ++i) {
			if (i > 0) out << ",";
			out << rootNode->children[i]->selectedAction << "(#visits: " << rootNode->children[i]->visits << ", #val: " << rootNode->children[i]->bestValue << ")";
		}
		out << "\n";
		file.close();
		////////////////////////////////////////////// DEBUG //////////////////////////////////////////////

		return rootNode->bestChild()->state;
	}

	boost::shared_ptr<MCTSTreeNode> MCTS::select(const boost::shared_ptr<MCTSTreeNode>& rootNode) {
		boost::shared_ptr<MCTSTreeNode> node = rootNode;

		// 探索木のリーフノードまで探索
		while (node->unexpandedActions.size() == 0 && node->children.size() > 0) {
			boost::shared_ptr<MCTSTreeNode> childNode = node->UCTSelectChild();
			if (childNode == NULL) break;
			node = childNode;
		}

		return node;
	}

	boost::shared_ptr<MCTSTreeNode> MCTS::expand(const boost::shared_ptr<MCTSTreeNode>& leafNode) {
		boost::shared_ptr<MCTSTreeNode> node = leafNode;

		// expandできない場合、つまり、本当のleafNodeなら、そのノードをそのまま返す
		if (node->unexpandedActions.size() == 0) {
			return node;
		}
		// 子ノードがまだ全てexpandされていない時は、1つランダムにexpand
		else {
			int action = node->randomlySelectAction();
			
			State child_state = node->state.clone();
			child_state.applyAction(action);

			boost::shared_ptr<MCTSTreeNode> child_node = boost::shared_ptr<MCTSTreeNode>(new MCTSTreeNode(child_state));
			child_node->selectedAction = action;
			node->children.push_back(child_node);
			child_node->parent = node;

			return child_node;
		}
	}
	
	float MCTS::simulate(const boost::shared_ptr<MCTSTreeNode>& childNode) {
		State state = childNode->state.clone();
		randomDerivation(state.derivationTree, state.queue);
		return evaluate(state.derivationTree);
	}

	void MCTS::backpropage(const boost::shared_ptr<MCTSTreeNode>& childNode, float value) {
		boost::shared_ptr<MCTSTreeNode> node = childNode;

		// リーフノードなら、スコアを確定する
		if (node->unexpandedActions.size() == 0 && node->children.size() == 0) {
			node->valueFixed = true;
		}

		while (node != NULL) {
			node->visits++;
			node->addValue(value);

			// 子ノードが全て展開済みで、且つ、スコア確定済みなら、このノードのスコアも確定とする
			if (node->unexpandedActions.size() == 0) {
				bool fixed = true;
				for (int c = 0; c < node->children.size(); ++c) {
					if (!node->children[c]->valueFixed) {
						fixed = false;
						break;
					}
				}
				node->valueFixed = fixed;
			}

			node = node->parent;
		}
	}

	float MCTS::evaluate(const DerivationTree& derivationTree) {
		QImage image;
		render(derivationTree.root, image);
		////////////////////////////////////////////// DEBUG //////////////////////////////////////////////
		//image.save("output.png");
		////////////////////////////////////////////// DEBUG //////////////////////////////////////////////

		cv::Mat sourceImage(image.height(), image.width(), CV_8UC4, image.bits(), image.bytesPerLine());
		cv::Mat grayImage;
		cv::cvtColor(sourceImage, grayImage, CV_RGB2GRAY);


		// compute a distance map
		cv::Mat distMap;
		cv::distanceTransform(grayImage, distMap, CV_DIST_L2, 3);
		////////////////////////////////////////////// DEBUG //////////////////////////////////////////////
		//cv::imwrite("distMap.png", distMap);
		////////////////////////////////////////////// DEBUG //////////////////////////////////////////////
		distMap.convertTo(distMap, CV_32F);

		// compute the squared difference
		return similarity(distMap, targetDistMap, SIMILARITY_METRICS_ALPHA, SIMILARITY_METRICS_BETA);
	}

	void MCTS::render(const DerivationTree& derivationTree, QImage& image) {
		glWidget->renderManager.removeObjects();
		std::vector<Vertex> vertices;
		generateGeometry(&glWidget->renderManager, glm::mat4(), derivationTree.root, vertices);
		glWidget->renderManager.addObject("tree", "", vertices, true);
		glWidget->render();
		
		image = glWidget->grabFrameBuffer();
	}

	void MCTS::generateGeometry(RenderManager* renderManager, const glm::mat4& modelMat, const boost::shared_ptr<Nonterminal>& node, std::vector<Vertex>& vertices) {
		glm::mat4 mat;

		if (node->name == "F") {
			glutils::drawQuad(node->segmentWidth, node->segmentLength, glm::vec4(0, 0, 0, 1), glm::translate(modelMat, glm::vec3(0, node->segmentLength * 0.5, 0)), vertices);
			mat = glm::translate(modelMat, glm::vec3(0, node->segmentLength, 0));
		}
		else if (node->name == "X") {
			glutils::drawQuad(node->segmentWidth, node->segmentLength, glm::vec4(0.5, 0.5, 0.5, 1), glm::translate(modelMat, glm::vec3(0, node->segmentLength * 0.5, 0)), vertices);
			mat = glm::translate(modelMat, glm::vec3(0, node->segmentLength, 0));
		}
		else if (node->name == "/") {
			if (!node->terminal) return;
			mat = glm::rotate(modelMat, node->angle / 180.0f * M_PI, glm::vec3(0, 0, 1));
		}
		else if (node->name == "\\") {
			if (!node->terminal) return;
			mat = glm::rotate(modelMat, node->angle / 180.0f * M_PI, glm::vec3(0, 0, 1));
		}
		
		for (int i = 0; i < node->children.size(); ++i) {
			generateGeometry(renderManager, mat, node->children[i], vertices);
		}
	}

	std::vector<int> actions(const boost::shared_ptr<Nonterminal>& nonterminal) {
		std::vector<int> ret;

		if (nonterminal->terminal) return ret;

		if (nonterminal->name == "X") {
			if (nonterminal->dist >= MAX_DIST - 1) {
				ret.push_back(0);
			}
			else if (nonterminal->dist < BASE_PART) {
				ret.push_back(1);
			}
			else {
				ret.push_back(1);
				if (nonterminal->level < MAX_LEVEL - 1) {
					ret.push_back(2);
				}
			}
		}
		else if (nonterminal->name == "/") {
			for (int i = 0; i < 5; ++i) {
				ret.push_back(i);
			}
		}
		else if (nonterminal->name == "\\") {
			for (int i = 0; i < 8; ++i) {
				ret.push_back(i);
			}
		}

		return ret;
	}

	void randomDerivation(DerivationTree& derivationTree, std::list<boost::shared_ptr<Nonterminal> >& queue) {
		int start_depth = queue.front()->dist;

		while (!queue.empty()) {
			boost::shared_ptr<Nonterminal> node = queue.front();
			queue.pop_front();

			if (node->terminal) continue;

			// simulation depth以上ならシミュレーションを終了
			if (SIMULATION_DEPTH > 0 && node->dist >= start_depth + SIMULATION_DEPTH) continue;

			std::vector<int> act = actions(node);
			if (act.size() > 0) {
				int action = act[rand() % act.size()];
				applyRule(derivationTree, node, action, queue);
			}
			else {
				int z = 0;
			}
		}
	}

	void applyRule(DerivationTree& derivationTree, const boost::shared_ptr<Nonterminal>& node, int action, std::list<boost::shared_ptr<Nonterminal> >& queue) {
		if (node->name == "X") {
			if (action == 0) {
				node->name = "F";
				node->terminal = true;
			}
			else if (action == 1) {
				node->name = "F";
				node->terminal = true;

				boost::shared_ptr<Nonterminal> child = boost::shared_ptr<Nonterminal>(new Nonterminal("/", node->level, node->dist + 1, node->segmentLength));
				node->children.push_back(child);
				queue.push_back(child);

				boost::shared_ptr<Nonterminal> grandchild = boost::shared_ptr<Nonterminal>(new Nonterminal("X", node->level, node->dist + 1, INITIAL_SEGMENT_LENGTH));
				child->children.push_back(grandchild);
				queue.push_back(grandchild);
			}
			else if (action == 2) {
				node->name = "F";
				node->terminal = true;

				boost::shared_ptr<Nonterminal> child1 = boost::shared_ptr<Nonterminal>(new Nonterminal("/", node->level, node->dist + 1, node->segmentLength));
				node->children.push_back(child1);
				queue.push_back(child1);

				boost::shared_ptr<Nonterminal> grandchild1 = boost::shared_ptr<Nonterminal>(new Nonterminal("X", node->level, node->dist + 1, INITIAL_SEGMENT_LENGTH));
				child1->children.push_back(grandchild1);
				queue.push_back(grandchild1);

				boost::shared_ptr<Nonterminal> child2 = boost::shared_ptr<Nonterminal>(new Nonterminal("\\", node->level + 1, node->dist + 1, node->segmentLength));
				node->children.push_back(child2);
				queue.push_back(child2);

				boost::shared_ptr<Nonterminal> grandchild2 = boost::shared_ptr<Nonterminal>(new Nonterminal("X", node->level + 1, node->dist + 1, INITIAL_SEGMENT_LENGTH));
				child2->children.push_back(grandchild2);
				queue.push_back(grandchild2);
			}
		}
		else if (node->name == "/") {
			node->angle = action * 10 - 20;
			node->terminal = true;
		}
		else if (node->name == "\\") {
			node->angle = action < 4 ? action * 20 - 90 : action * 20 - 50;
			node->terminal = true;
		}
	}

	float similarity(const cv::Mat& distMap, const cv::Mat& targetDistMap, float alpha, float beta) {
		float dist1 = 0.0f;
		float dist2 = 0.0f;

		for (int r = 0; r < distMap.rows; ++r) {
			for (int c = 0; c < distMap.cols; ++c) {
				if (targetDistMap.at<float>(r, c) == 0) {
					dist1 += distMap.at<float>(r, c);
				}
				if (distMap.at<float>(r, c) == 0) {
					dist2 += targetDistMap.at<float>(r, c);
				}
			}
		}

		// 画像サイズに基づいて、normalizeする
		float Z = distMap.rows * distMap.cols * (distMap.rows + distMap.cols) * 0.5;
		dist1 /= Z;
		dist2 /= Z;

		float dist = alpha * dist1 + beta * dist2;

		return expf(-dist);

	}

}