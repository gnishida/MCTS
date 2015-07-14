#include <time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "ParametricLSystem.h"
#include "MLUtils.h"

using namespace std;

int main() {
	// ターゲットindicatorを読み込む
	cv::Mat target = cv::imread("target_indicator7.png", 0);
	target.convertTo(target, CV_32F, 1.0/255.0);
	cv::flip(target, target, 0);

	// 白黒を反転させる
	target = 1 - target;

	srand(6234);

	// ターゲットに近いモデルを生成する
	time_t start = clock();
	parametriclsystem::ParametricLSystem lsystem(100, 1.0f, parametriclsystem::Literal("X", 0, 12.0f, 3.0f));
	parametriclsystem::String model = lsystem.inverse(target);
	time_t end = clock();

	cout << model << endl;

	cout << fixed << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC  << " [sec]" << endl;

	// 生成したモデルの画像を保存する
	cv::Mat img;
	lsystem.computeIndicator(model, 4.0f, img);
	cv::resize(target, target, cv::Size(400, 400));
	//target.convertTo(target, CV_32F, 0.4);
	img += target * 0.4;
	ml::mat_save("result.png", img);
}
