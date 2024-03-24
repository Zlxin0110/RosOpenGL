#include "texture.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include "def.h"

// 在初始化时创建矩形的顶点数据
float screenVertices[] = {
    // 位置           // 纹理坐标
    -1.0f, -1.0f,    0.0f, 0.0f,
     1.0f, -1.0f,    1.0f, 0.0f,
     1.0f,  1.0f,    1.0f, 1.0f,
    -1.0f,  1.0f,    0.0f, 1.0f
};

unsigned int indices[] = {
    0, 1, 2,
    2, 3, 0
};

CTexture::CTexture(const char *vertexPath, const char *fragmentPath)
{
    mShader = new Shader(vertexPath, fragmentPath);

    glGenTextures(1, &mTexture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WIDTH, HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mTexture, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "Error: Framebuffer is not complete!" << std::endl;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
// ==============================================================================
    glGenVertexArrays(1, &mVAO);
    glGenBuffers(1, &mVBO);
    glGenBuffers(1, &mEBO);

    glBindVertexArray(mVAO);
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(screenVertices), screenVertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
}

CTexture::~CTexture()
{
    glDeleteVertexArrays(1, &mVAO);
    glDeleteBuffers(1, &mVBO);
    glDeleteTextures(1, &mTexture);

    if (mShader != nullptr)
    {
        delete mShader;
        mShader = nullptr;
    }
}

void CTexture::Rendering()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    mShader->begin();
    {
        // 绑定纹理
        //glBindTexture(GL_TEXTURE_2D, mTexture);

        mShader->setMatrix("texture1",0);
        // 设置纹理单元
        // 渲染矩形
        glBindVertexArray(mVAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }
    mShader->end();
}