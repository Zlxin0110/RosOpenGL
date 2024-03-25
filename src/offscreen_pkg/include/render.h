#pragma once

#define BY_SHADER   (0)

class CRender {
public:
#if BY_SHADER
    virtual void Rendering() = 0; // 纯虚函数，作为接口
#else
    virtual void Rendering(GLuint fbo) = 0; // 纯虚函数，作为接口
#endif
};