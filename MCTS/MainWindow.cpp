#include "MainWindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionClearImage, SIGNAL(triggered()), this, SLOT(onClearImage()));
	connect(ui.actionOpenImage, SIGNAL(triggered()), this, SLOT(onOpenImage()));
	connect(ui.actionSaveImage, SIGNAL(triggered()), this, SLOT(onSaveImage()));
	connect(ui.actionSave3DMesh, SIGNAL(triggered()), this, SLOT(onSave3DMesh()));
	connect(ui.actionMCTS, SIGNAL(triggered()), this, SLOT(onMCTS()));

	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

void MainWindow::onClearImage() {
	glWidget->clearImage();
	glWidget->update();
}

void MainWindow::onOpenImage() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open image file..."), "", tr("Image Files (*.png)"));
	if (filename.isEmpty()) return;

	glWidget->loadImage(filename);
}

void MainWindow::onSaveImage() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save image file..."), "", tr("Image Files (*.png)"));
	if (filename.isEmpty()) return;

	glWidget->saveImage(filename);
}

void MainWindow::onSave3DMesh() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save obj file..."), "", tr("OBJ Files (*.obj)"));
	if (filename.isEmpty()) return;

	glWidget->save3DMesh(filename);
}

void MainWindow::onMCTS() {
	glWidget->runMCTS();
}