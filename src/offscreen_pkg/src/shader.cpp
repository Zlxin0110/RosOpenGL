#include <string>
#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>
#include "shader.h"
#include <ros/ros.h>

std::string Shader::readShaderFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

Shader::Shader(const char* vertexPath, const char* fragmentPath) {

    boost::filesystem::path exePath = boost::filesystem::canonical("/proc/self/exe");
    boost::filesystem::path exeDir = exePath.parent_path();

	std::string vSlgl = (exeDir / "../../../src/offscreen_pkg").string() + vertexPath;
	std::string fSlgl = (exeDir / "../../../src/offscreen_pkg").string() + fragmentPath;
    // std::cout << "Executable directory: " << dirSlgl << std::endl;

	// //声明装入shader代码字符串的两个string
	std::string vertexCode = readShaderFile(vSlgl);
	std::string fragmentCode = readShaderFile(fSlgl);

	const char* vertexShaderSource = vertexCode.c_str();
	const char* fragmentShaderSource = fragmentCode.c_str();

	//1 创建Shader程序（vs、fs）
	GLuint vertex, fragment;
	vertex = glCreateShader(GL_VERTEX_SHADER);
	fragment = glCreateShader(GL_FRAGMENT_SHADER);

	//2 为shader程序输入shader代码
	glShaderSource(vertex, 1, &vertexShaderSource, NULL);
	glShaderSource(fragment, 1, &fragmentShaderSource, NULL);

	//3 执行shader代码编译 
	glCompileShader(vertex);
	//检查vertex编译结果
	checkShaderErrors(vertex, "COMPILE");
	
	glCompileShader(fragment);
	//检查fragment编译结果
	checkShaderErrors(fragment, "COMPILE");
	
	//4 创建一个Program壳子
	mProgram = glCreateProgram();

	//6 将vs与fs编译好的结果放到program这个壳子里
	glAttachShader(mProgram, vertex);
	glAttachShader(mProgram, fragment);

	//7 执行program的链接操作，形成最终可执行shader程序
	glLinkProgram(mProgram);

	//检查链接错误
	checkShaderErrors(mProgram, "LINK");

	//清理
	glDeleteShader(vertex);
	glDeleteShader(fragment);
}
Shader::~Shader() {
    glDeleteProgram(mProgram);
}

void Shader::begin() {
	glUseProgram(mProgram);
}

void Shader::end() {
	glUseProgram(0);
}

void Shader::setFloat(const std::string& name, float value) {
	//1 通过名称拿到Uniform变量的位置Location
	GLint location = glGetUniformLocation(mProgram, name.c_str());

	//2 通过Location更新Uniform变量的值
	glUniform1f(location, value);
}

void Shader::setVector3(const std::string& name, float x, float y, float z) {
	//1 通过名称拿到Uniform变量的位置Location
	GLint location = glGetUniformLocation(mProgram, name.c_str());
	
	//2 通过Location更新Uniform变量的值
	glUniform3f(location, x, y, z);
}

//重载 overload
void Shader::setVector3(const std::string& name, const float* values) {
	//1 通过名称拿到Uniform变量的位置Location
	GLint location = glGetUniformLocation(mProgram, name.c_str());

	//2 通过Location更新Uniform变量的值
	//第二个参数：你当前要更新的uniform变量如果是数组，数组里面包括多少个向量vec3
	glUniform3fv(location, 1, values);
}

void Shader::setInt(const std::string& name, int value) {
	//1 通过名称拿到Uniform变量的位置Location
	GLint location = glGetUniformLocation(mProgram, name.c_str());

	//2 通过Location更新Uniform变量的值
	glUniform1i(location, value);
}

void Shader::setMatrix(const std::string& name, glm::mat4 value)
{
	//1 通过名称拿到Uniform变量的位置Location
	GLint location = glGetUniformLocation(mProgram, name.c_str());

	//2 通过Location更新Uniform变量的值
	glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value));
}

void Shader::checkShaderErrors(GLuint target, std::string type) {
	int success = 0;
	char infoLog[1024];

	if (type == "COMPILE") {
		glGetShaderiv(target, GL_COMPILE_STATUS, &success);
		if (!success) {
			glGetShaderInfoLog(target, 1024, NULL, infoLog);
			std::cout << "Error: SHADER COMPILE ERROR" << "\n" << infoLog << std::endl;
		}
	}
	else if (type == "LINK") {
		glGetProgramiv(target, GL_LINK_STATUS, &success);
		if (!success) {
			glGetProgramInfoLog(target, 1024, NULL, infoLog);
			std::cout << "Error: SHADER LINK ERROR " << "\n" << infoLog << std::endl;
		}
	}
	else {
		std::cout << "Error: Check shader errors Type is wrong" << std::endl;
	}
}

GLuint Shader::Handle()
{
    return mProgram;
}