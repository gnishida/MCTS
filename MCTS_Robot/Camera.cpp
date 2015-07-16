﻿#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <fstream>

#ifndef M_PI
#define M_PI	3.1415926535
#endif

Camera::Camera() {
	fovy = 60.0f;

	resetCamera();
}

/**
 * Call this function when the mouse button is pressed.
 */
void Camera::mousePress(int mouse_x, int mouse_y) {
	mouse_pos = glm::vec2(mouse_x, mouse_y);
}

/**
 * Call this function whenever the mouse moves while rotating the model.
 */
void Camera::rotate(int mouse_x, int mouse_y) {
	xrot += mouse_y - mouse_pos.y;
	yrot += mouse_x - mouse_pos.x;
	updateMVPMatrix();

	mouse_pos = glm::vec2(mouse_x, mouse_y);
}

/**
 * Call this function whenever the mouse moves while zooming.
 */
void Camera::zoom(int mouse_x, int mouse_y) {
	pos.z += mouse_pos.y - mouse_y;
	updateMVPMatrix();

	mouse_pos = glm::vec2(mouse_x, mouse_y);
}

/**
 * Call this function whenever the mouse moves while moving the model.
 */
void Camera::move(int mouse_x, int mouse_y) {
	pos.x -= (mouse_x - mouse_pos.x) * 0.1;
	pos.y += (mouse_y - mouse_pos.y) * 0.1;
	updateMVPMatrix();

	mouse_pos = glm::vec2(mouse_x, mouse_y);
}

/**
 * Update perspective projection matrix, and then, update the model view projection matrix.
 */
void Camera::updatePMatrix(int width,int height) {
	float aspect = (float)width / (float)height;
	float zfar = 8000.0f;
	float znear = 0.1f;
	float f = 1.0f / tan(fovy * M_PI / 360.0f);

	// projection行列
	// ただし、mat4はcolumn majorなので、転置した感じで初期構築する。
	// つまり、下記の一行目は、mat4の一列目に格納されるのだ。
	pMatrix = glm::mat4(
		 f/aspect,	0,								0,									0,
				0,	f,								0,						 			0,
			    0,	0,		(zfar+znear)/(znear-zfar),		                           -1,
			    0,	0, (2.0f*zfar*znear)/(znear-zfar),									0);

	updateMVPMatrix();
}

/**
 * Update the model view projection matrix
 */
void Camera::updateMVPMatrix() {
	// create model view matrix
	mvMatrix = glm::mat4();
	mvMatrix = glm::translate(mvMatrix, -pos);
	mvMatrix = glm::rotate(mvMatrix, xrot * (float)M_PI / 180.0f, glm::vec3(1, 0, 0));
	mvMatrix = glm::rotate(mvMatrix, yrot * (float)M_PI / 180.0f, glm::vec3(0, 1, 0));
	mvMatrix = glm::rotate(mvMatrix, zrot * (float)M_PI / 180.0f, glm::vec3(0, 0, 1));

	// create model view projection matrix
	mvpMatrix = pMatrix * mvMatrix;
}

void Camera::resetCamera() {
	xrot = 0.0f;
	yrot = 0.0;
	zrot = 0.0f;
	pos = glm::vec3(0, 0, 300);
	updateMVPMatrix();
}

void Camera::saveCameraPose(char* filename) {
	std::ofstream ofs(filename);

	if (ofs.fail()) {
		std::cout << "Can't open file: " << filename << std::endl;
	} else {
		ofs << xrot << " " << yrot << " " << zrot << " " << pos.x << " " << pos.y << " " << pos.z << std::endl;
	}
}

void Camera::loadCameraPose(char* filename) {
	std::ifstream ifs(filename);

	if (ifs.fail()) {
		std::cout << "Can't open file: " << filename << std::endl;
	} else {
		ifs >> xrot >> yrot >> zrot >> pos.x >> pos.y >> pos.z;
		updateMVPMatrix();
	}
}