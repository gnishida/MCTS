#pragma once

#include <glm/glm.hpp>
#include "boost/shared_ptr.hpp"
#include <string>
#include <vector>
#include "Vertex.h"
#include <opencv2/opencv.hpp>

class RenderManager;
class Camera;

namespace pmtree {
	float shapeRatio(int shape, float ratio);

	class TreeNode {
	public:
		float attenuationFactor;	// 親枝に対する長さの比率
		float curveV;

		int level;
		int index;
		float segmentLength;
		float baseFactor;			// この枝の根元部分の割合
		float curve;				// この枝の全体的な曲率（前半部分）
		float curveBack;			// この枝の全体的な曲率（後半部分）
		boost::shared_ptr<TreeNode> parent;
		std::vector<boost::shared_ptr<TreeNode> > children;

	public:
		TreeNode(boost::shared_ptr<TreeNode> parent, int level, int index, float segmentLength, float attenuationFactor, float baseFactor, float curve, float curveBack);
		void generateRandom();
		std::string to_string();
		void recover(const std::vector<float>& params);
	};

	class PMTree2D {
	public:
		boost::shared_ptr<TreeNode> root;

	public:
		PMTree2D();

		void generateRandom();
		bool generateGeometry(RenderManager* renderManager, bool fixed_width);
		void generateTrainingData(const cv::Mat& image, Camera* camera, int screenWidth, int screenHeight, std::vector<cv::Mat>& localImages, std::vector<std::vector<float> >& parameters);
		void generateTrainingData(const glm::mat4& modelMat, float segment_length, boost::shared_ptr<TreeNode>& node, const cv::Mat& imagePadded, int padding, Camera* camera, int screenWidth, int screenHeight, std::vector<cv::Mat>& localImages, std::vector<std::vector<float> >& parameters);
		std::string to_string();
		std::string to_string(int index);
		void recover(const std::vector<std::vector<float> >& params);

	private:
		bool generateGeometry(RenderManager* renderManager, const glm::mat4& modelMat, float segment_length, float segment_width, bool fixed_width, boost::shared_ptr<TreeNode>& node, glm::vec3& p0, glm::vec3& p1, std::vector<Vertex>& vertices);
	};

}