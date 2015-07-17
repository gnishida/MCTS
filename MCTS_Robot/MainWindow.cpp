﻿#include "MainWindow.h"
#include "MLUtils.h"
#include <QFileDialog>
#include <QDate>
#include <time.h>

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags) : QMainWindow(parent, flags) {
	ui.setupUi(this);

	connect(ui.actionSaveImage, SIGNAL(triggered()), this, SLOT(onSaveImage()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionRandomGeneration, SIGNAL(triggered()), this, SLOT(onRandomGeneration()));
	connect(ui.actionGreedyInverse, SIGNAL(triggered()), this, SLOT(onGreedyInverse()));

	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

void MainWindow::onSaveImage() {
	if (!QDir("screenshots").exists()) QDir().mkdir("screenshots");
	QString fileName = "screenshots/" + QDate::currentDate().toString("yyMMdd") + "_" + QTime::currentTime().toString("HHmmss") + ".png";
	glWidget->grabFrameBuffer().save(fileName);
	printf("Save %s\n",fileName.toAscii().constData());
}

void MainWindow::onRandomGeneration() {
	cv::Mat indicator;
	glWidget->model = glWidget->lsystem.derive(1234);

	cout << glWidget->model << endl;

	glWidget->lsystem.draw(glWidget->model, glWidget->vertices);
	glWidget->createVAO();
	glWidget->updateGL();
}

void MainWindow::onGreedyInverse() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open target file..."), "", tr("Indicator Files (*.png)"));
	if (filename.isEmpty()) return;

	// ターゲットindicatorを読み込む
	cv::Mat target = cv::imread(filename.toUtf8().data(), 0);
	target.convertTo(target, CV_32F, 1.0/255.0);
	cv::flip(target, target, 0);

	// 白黒を反転させる
	target = 1 - target;

	// ターゲットに近いモデルを生成する
	time_t start = clock();
	glWidget->model = glWidget->lsystem.inverse(target);
	time_t end = clock();

	cout << glWidget->model << endl;

	cout << fixed << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC  << " [sec]" << endl;

	// 生成したモデルの画像を保存する
	cv::Mat img;
	glWidget->lsystem.computeIndicator(glWidget->model, 1.0f, glm::mat4(), img);
	/*
	cv::resize(target, target, cv::Size(300, 300));
	target.convertTo(target, CV_32F, 0.4);
	img += target;
	*/
	ml::mat_save("result.png", img + target * 0.4);

	glWidget->lsystem.draw(glWidget->model, glWidget->vertices);
	glWidget->createVAO();
	glWidget->updateGL();
}
