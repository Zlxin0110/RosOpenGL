#include "axis.h"
#include "def.h"

const float axisVertices[] = {
    // X轴
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    // Y轴
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    // Z轴
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};

CAxis::CAxis(const char *vertexPath, const char *fragmentPath)
{
    mShader = new Shader(vertexPath, fragmentPath);

    glGenBuffers(1, &mVBO);
    glGenVertexArrays(1, &mVAO);

    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBindVertexArray(mVAO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axisVertices), axisVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
}

CAxis::~CAxis()
{
    glDeleteVertexArrays(1, &mVAO);
    glDeleteBuffers(1, &mVBO);

    if (mShader != nullptr)
    {
        delete mShader;
        mShader = nullptr;
    }
}

void CAxis::Rendering(glm::mat4 model, glm::mat4 view, glm::mat4 projection)
{
    mShader->begin();
    {
        // glm::mat4 model = glm::mat4(1.0f);
        // glm::mat4 view = glm::lookAt(VIEW_EYE_POSITION, VIEW_LOOK_AT_POSITION, VIEW_CAMERA_UP_POSITION);
        // glm::mat4 projection = glm::perspective(glm::radians(PROJECTION_FOVY), PROJECTION_ASPEC, PROJECTION_ZNEAR, PROJECTION_ZFAR);
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "projection"), 1, GL_FALSE, glm::value_ptr(projection));

        glBindVertexArray(mVAO);
        glDrawArrays(GL_LINES, 0, 6);
        glBindVertexArray(0);
    }
    mShader->end();
}