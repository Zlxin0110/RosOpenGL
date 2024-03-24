#pragma once

#include <GL/glew.h>
#include <string>
#include "def.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Shader {
public:
	Shader(const char* vertexPath, const char* fragmentPath);
	~Shader();
	
	void begin();//开始使用当前Shader

	void end();//结束使用当前Shader

	void setFloat(const std::string& name, float value);

	void setVector3(const std::string& name, float x, float y, float z);
	void setVector3(const std::string& name, const float* values);

	void setInt(const std::string& name, int value);
	void setMatrix(const std::string& name, glm::mat4 value);

	std::string readShaderFile(const std::string& filePath);

    GLuint Handle();
private:

	void checkShaderErrors(GLuint target,std::string type);

private:
	GLuint mProgram{ 0 };
};
