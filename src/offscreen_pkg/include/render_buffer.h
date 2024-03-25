#pragma once
#include <GL/glew.h>
#include "shader.h"
#include "render.h"

class CRanderBuffer : public CRender{
public:
	CRanderBuffer();
	~CRanderBuffer();
#if BY_SHADER
    void Rendering();
#else
    void Rendering(GLuint fbo);
#endif
    

private:
	GLuint mVAO;
    GLuint mVBO;
    GLuint mEBO;
    GLuint mRBO;
    Shader *mShader;
};
