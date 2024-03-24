#pragma once
#include <GL/glew.h>

#include "shader.h"

class CTexture {
public:
	CTexture(const char* vertexPath, const char* fragmentPath);
	~CTexture();

    void Rendering();


private:
	GLuint mVAO;
    GLuint mVBO;
    GLuint mEBO;
    GLuint mFBO;
    GLuint mTexture;
    Shader *mShader;
};
