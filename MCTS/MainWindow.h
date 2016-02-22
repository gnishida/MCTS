#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "GLWidget3D.h"

class MainWindow : public QMainWindow {
	Q_OBJECT

private:
	Ui::MainWindowClass ui;
	GLWidget3D* glWidget;

public:
	MainWindow(QWidget *parent = 0);

public slots:
	void onClearImage();
	void onOpenImage();
	void onSaveImage();
	void onSave3DMesh();
	void onMCTS();
	void onRandomGeneration();
};

#endif // MAINWINDOW_H
