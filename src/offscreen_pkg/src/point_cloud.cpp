#include "point_cloud.h"
#include "def.h"


CPointCloud::CPointCloud(const char *vertexPath, const char *fragmentPath)
{
    mShader = new Shader(vertexPath, fragmentPath);

    glGenBuffers(1, &mVBO);
    glGenVertexArrays(1, &mVAO);
 
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBindVertexArray(mVAO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

CPointCloud::~CPointCloud()
{
    glDeleteVertexArrays(1, &mVAO);
    glDeleteBuffers(1, &mVBO);

    if (mShader != nullptr)
    {
        delete mShader;
        mShader = nullptr;
    }
}


void CPointCloud::Rendering(glm::mat4 model, glm::mat4 view, glm::mat4 projection, std::vector<float> vertices)
{
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    mShader->begin();
    {
        // 绘制ball
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "projection"), 1, GL_FALSE, glm::value_ptr(projection));
  
        glBindVertexArray(mVAO);
        glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
        glBindVertexArray(0);
    }
    mShader->end();
}