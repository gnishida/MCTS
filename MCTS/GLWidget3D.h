#pragma once

#include "glew.h"
#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include "Camera.h"
#include "ShadowMapping.h"
#include "RenderManager.h"
#include <vector>

class MainWindow;

class GLWidget3D : public QGLWidget {
public:
	MainWindow* mainWin;
	QImage sketch;
	QPoint lastPos;
	Camera camera;
	glm::vec3 light_dir;
	glm::mat4 light_mvpMatrix;
	RenderManager renderManager;
	bool ctrlPressed;
	bool shiftPressed;
	bool altPressed;

public:
	GLWidget3D(MainWindow *parent);
	void generateTrainingData();
	void generateTrainingDataTrunk();
	void generateLocalTrainingData();
	void generatePredictedData();
	void generatePredictedDataTrunk();
	void render();
	void drawScene();
	void clearImage();
	void loadImage(const QString& filename);
	void saveImage(const QString& filename);
	void save3DMesh(const QString& filename);
	void drawLine(const QPoint& startPoint, const QPoint& endPoint);
	void runMCTS();

	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);

protected:
	void mousePressEvent(QMouseEvent* e);
	void mouseMoveEvent(QMouseEvent* e);
	void mouseReleaseEvent(QMouseEvent* e);
	void wheelEvent(QWheelEvent* e);
	void initializeGL();
	void resizeGL(int width, int height);
	void paintEvent(QPaintEvent* e);
};

