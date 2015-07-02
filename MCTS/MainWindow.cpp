#include "MainWindow.h"
#include "MLUtils.h"
#include <time.h>

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
	glWidget->model = glWidget->lsystem.derive(3, indicator);

	cout << glWidget->model << endl;
	ml::mat_save("indicator.png", indicator);

	glWidget->lsystem.draw(glWidget->model, glWidget->vertices);
	glWidget->createVAO();
	glWidget->updateGL();
}

void MainWindow::onGreedyInverse() {
	// ターゲットindicatorを読み込む
	cv::Mat target_indicator = cv::imread("target_indicator2.png", 0);
	target_indicator.convertTo(target_indicator, CV_32F, 1.0/255.0);
	cv::flip(target_indicator, target_indicator, 0);

	// 白黒を反転させる
	target_indicator = 1 - target_indicator;

	// ターゲットに近いモデルを生成する
	cv::Mat indicator;
	time_t start = clock();
	glWidget->model = glWidget->lsystem.inverse(target_indicator, indicator);
	time_t end = clock();

	cout << glWidget->model << endl;

	cout << fixed << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC  << " [sec]" << endl;

	// 生成したモデルの画像を保存する
	cv::Mat img;
	glWidget->lsystem.computeIndicator(glWidget->model, 1.0f, img);
	/*
	cv::resize(target_indicator, target_indicator, cv::Size(300, 300));
	target_indicator.convertTo(target_indicator, CV_32F, 0.4);
	img += target_indicator;
	*/
	ml::mat_save("result.png", img);

	glWidget->lsystem.draw(glWidget->model, glWidget->vertices);
	glWidget->createVAO();
	glWidget->updateGL();
}
