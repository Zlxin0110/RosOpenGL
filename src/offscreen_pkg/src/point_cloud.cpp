#include "point_cloud.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "def.h"
#include <iostream>


CPointCloud::CPointCloud(const char *vertexPath, const char *fragmentPath)
{
    mShader = new Shader(vertexPath, fragmentPath);

    glGenBuffers(1, &mVBO);
    glGenVertexArrays(1, &mVAO);
 
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBindVertexArray(mVAO);
    // glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
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

void CPointCloud::Rendering(std::vector<float> vertices)
{
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    mShader->begin();
    {
        // 绘制ball
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::lookAt(glm::vec3(5.0f, 5.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(mShader->Handle(), "projection"), 1, GL_FALSE, glm::value_ptr(projection));
  
        glBindVertexArray(mVAO);
        glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
        glBindVertexArray(0);
    }
    mShader->end();
}