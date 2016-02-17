#pragma once

#include "glew.h"
#include <vector>
#include <QMap>
#include "Vertex.h"
#include "ShadowMapping.h"

class GeometryObject {
public:
	GLuint vao;
	GLuint vbo;
	std::vector<Vertex> vertices;
	bool vaoCreated;
	bool vaoOutdated;

public:
	GeometryObject();
	GeometryObject(const std::vector<Vertex>& vertices);
	void addVertices(const std::vector<Vertex>& vertices);
	void createVAO();
};

class RenderManager {
public:
	GLuint program;
	QMap<QString, QMap<GLuint, GeometryObject> > objects;
	QMap<QString, GLuint> textures;

	bool useShadow;
	ShadowMapping shadow;

	int renderingMode;
	float depthSensitivity;
	float normalSensitivity;
	bool useThreshold;
	float threshold;

public:
	RenderManager();

	void init(bool useShadow, int shadowMapSize = 4096);
	void addObject(const QString& object_name, const QString& texture_file, const std::vector<Vertex>& vertices);
	void removeObjects();
	void removeObject(const QString& object_name);
	void centerObjects();
	void renderAll();
	void renderAllExcept(const QString& object_name);
	void render(const QString& object_name);
	void updateShadowMap(GLWidget3D* glWidget3D, const glm::vec3& light_dir, const glm::mat4& light_mvpMatrix);
	void resize(int width, int height);


private:
	GLuint loadTexture(const QString& filename);
};

