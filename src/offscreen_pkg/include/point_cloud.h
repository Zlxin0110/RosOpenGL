#pragma once
#include <GL/glew.h>
#include "shader.h"
#include <vector>

class CPointCloud {
public:
	CPointCloud(const char* vertexPath, const char* fragmentPath);
	~CPointCloud();

    void Rendering(std::vector<float> vertices);


private:
	GLuint mVAO;
    GLuint mVBO;
    Shader *mShader;
};
