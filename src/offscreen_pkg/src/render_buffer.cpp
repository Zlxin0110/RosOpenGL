#include "render_buffer.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "def.h"

#if BY_SHADER
// // 在初始化时创建矩形的顶点数据
// float screenVertices[] = {
//     // 位置           // 纹理坐标
//     -1.0f, -1.0f,    0.0f, 0.0f,
//      1.0f, -1.0f,    1.0f, 0.0f,
//      1.0f,  1.0f,    1.0f, 1.0f,
//     -1.0f,  1.0f,    0.0f, 1.0f
// };

// unsigned int indices[] = {
//     0, 1, 2,
//     2, 3, 0
// };
CRanderBuffer::CRanderBuffer()
{
    //mShader = new Shader(vertexPath, fragmentPath);

    // 创建渲染缓冲对象并附加到帧缓冲对象
    GLuint mRbo;
    glGenRenderbuffers(1, &mRbo);
    //glActiveTexture(GL_TEXTURE0);
    glBindRenderbuffer(GL_RENDERBUFFER, mRbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, WIDTH, HEIGHT);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, mRbo);

    // 检查帧缓冲完整性
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Framebuffer is not complete" << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        //glDeleteFramebuffers(1, &fbo);
        glDeleteRenderbuffers(1, &mRbo);
        glfwTerminate();
        return;
    }
}

CRanderBuffer::~CRanderBuffer()
{
    glDeleteVertexArrays(1, &mVAO);
    glDeleteBuffers(1, &mVBO);

    if (mShader != nullptr)
    {
        delete mShader;
        mShader = nullptr;
    }
}

void CRanderBuffer::Rendering()
{

}
#else
CRanderBuffer::CRanderBuffer()
{
    //mShader = new Shader(vertexPath, fragmentPath);

    // 创建渲染缓冲对象并附加到帧缓冲对象
    GLuint mRbo;
    glGenRenderbuffers(1, &mRbo);
    //glActiveTexture(GL_TEXTURE0);
    glBindRenderbuffer(GL_RENDERBUFFER, mRbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, WIDTH, HEIGHT);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, mRbo);

    // 检查帧缓冲完整性
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Framebuffer is not complete" << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        //glDeleteFramebuffers(1, &fbo);
        glDeleteRenderbuffers(1, &mRbo);
        glfwTerminate();
        return;
    }
}

CRanderBuffer::~CRanderBuffer()
{
    glDeleteVertexArrays(1, &mVAO);
    glDeleteBuffers(1, &mVBO);

    if (mShader != nullptr)
    {
        delete mShader;
        mShader = nullptr;
    }
}

void CRanderBuffer::Rendering(GLuint fbo)
{
    // framebuffer是您的帧缓冲对象的id
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
    // 绑定默认帧缓冲，即屏幕
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    // glBlitFramebuffer函数用于将一个帧缓冲的内容复制到另一个帧缓冲（这里是默认帧缓冲，即屏幕）。
    // GL_NEAREST表示使用最近邻插值方法进行复制，可以根据需要选择其他插值方法。
    glBlitFramebuffer(0, 0, WIDTH, HEIGHT, 0, 0, WIDTH, HEIGHT, GL_COLOR_BUFFER_BIT, GL_NEAREST);
}
#endif