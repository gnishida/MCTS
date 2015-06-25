#include "MainWindow.h"
#include "MLUtils.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags) : QMainWindow(parent, flags) {
	ui.setupUi(this);

	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionRandomGeneration, SIGNAL(triggered()), this, SLOT(onRandomGeneration()));
	connect(ui.actionGreedyInverse, SIGNAL(triggered()), this, SLOT(onGreedyInverse()));

	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

void MainWindow::onRandomGeneration() {
	cv::Mat indicator;
	glWidget->model = glWidget->lsystem.derive(0, indicator);

	cout << glWidget->model << endl;
	ml::mat_save("indicator.png", indicator);

	glWidget->updateGL();
}

void MainWindow::onGreedyInverse() {
	// ターゲットindicatorを読み込む
	cv::Mat target_indicator = cv::imread("target_indicator.png", 0);
	target_indicator.convertTo(target_indicator, CV_32F, 1.0/255.0);
	cv::flip(target_indicator, target_indicator, 0);

	// 白黒を反転させる
	target_indicator = 1 - target_indicator;

	// blur
	/*cv::Mat tmp;
	cv::GaussianBlur(target_indicator, tmp, cv::Size(11, 11), 11, 11);
	target_indicator += tmp * 0.8;
	//ml::mat_save("target2.png", target_indicator);
	*/

	// ターゲットに近いモデルを生成する
	cv::Mat indicator;
	glWidget->model = glWidget->lsystem.inverse(target_indicator, indicator);

	cout << glWidget->model << endl;
	ml::mat_save("indicator.png", indicator);

	glWidget->updateGL();
}
