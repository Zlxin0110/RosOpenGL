#pragma once
#include <GL/glew.h>

#include "shader.h"
#include "render.h"

class CTexture : public CRender{
public:
	CTexture();
	~CTexture();
#if BY_SHADER
    void Rendering();
#else
    void Rendering(GLuint fbo);
#endif

private:
	GLuint mVAO;
    GLuint mVBO;
    GLuint mEBO;
    GLuint mTexture;
    Shader *mShader;
};
