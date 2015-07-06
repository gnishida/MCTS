#pragma once

#include <glew.h>
#include "Shader.h"
#include "Vertex.h"
#include <QGLWidget>
#include <QMouseEvent>
#include "Camera.h"
#include "ShadowMapping.h"
#include "ParametricLSystem.h"

class MainWindow;

class GLWidget3D : public QGLWidget {
public:
	MainWindow* mainWin;
	Camera camera;
	GLuint vao;
	GLuint program;
	std::vector<Vertex> vertices;
	glm::vec3 light_dir;

	ShadowMapping shadow;

	parametriclsystem::ParametricLSystem lsystem;
	parametriclsystem::String model;

public:
	GLWidget3D(MainWindow *parent);
	void drawScene(int drawMode);
	void createVAO();

protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();    
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
};

