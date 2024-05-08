#pragma once
#include <GL/glew.h>
#include "shader.h"

class CAxis {
public:
	CAxis(const char* vertexPath, const char* fragmentPath);
	~CAxis();

    void Rendering(glm::mat4 model, glm::mat4 view, glm::mat4 projection);
 

private:
	GLuint mVAO;
    GLuint mVBO;
    Shader *mShader;
};
