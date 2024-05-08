#pragma once
#include <GL/glew.h>
#include "shader.h"
#include <vector>

class CPointCloud {
public:
	CPointCloud(const char* vertexPath, const char* fragmentPath);
	~CPointCloud();

    void Rendering(glm::mat4 model, glm::mat4 view, glm::mat4 projection, std::vector<float> vertices);

private:
	GLuint mVAO;
    GLuint mVBO;
    Shader *mShader;
};
