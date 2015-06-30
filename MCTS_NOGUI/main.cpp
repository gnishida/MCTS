#include <time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "ParametricLSystem.h"
#include "MLUtils.h"

using namespace std;

int main() {
	// ターゲットindicatorを読み込む
	cv::Mat target_indicator = cv::imread("target_indicator.png", 0);
	target_indicator.convertTo(target_indicator, CV_32F, 1.0/255.0);
	cv::flip(target_indicator, target_indicator, 0);

	// 白黒を反転させる
	target_indicator = 1 - target_indicator;

	time_t start = clock();

	// ターゲットに近いモデルを生成する
	cv::Mat indicator;
	parametriclsystem::ParametricLSystem lsystem(300, 0.1);
	parametriclsystem::String model = lsystem.inverse(target_indicator, indicator);

	cout << model << endl;

	time_t end = clock();
	cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC  << " [sec]" << endl;

	// 生成したモデルの画像を保存する
	cv::Mat img;
	lsystem.computeIndicator(model, 1.0f, img);
	ml::mat_save("result.png", img);
}
