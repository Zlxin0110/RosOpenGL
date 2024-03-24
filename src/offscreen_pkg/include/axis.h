#pragma once
#include <GL/glew.h>
#include "shader.h"

class CAxis {
public:
	CAxis(const char* vertexPath, const char* fragmentPath);
	~CAxis();

    void Rendering();


private:
	GLuint mVAO;
    GLuint mVBO;
    Shader *mShader;
};
