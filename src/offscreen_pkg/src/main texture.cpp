#include <thread>
#include "def.h"
#include <chrono>
#include <vector>
#include <png++/png.hpp>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "axis.h"
#include "point_cloud.h"
#include "texture.h"

// ======================================================================
// 生成3D点云数据
void saveRenderbufferToPNG(const std::string& filename, int width, int height) {
    std::vector<unsigned char> pixels(width * height * 4); // RGBA format
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    png::image<png::rgba_pixel> image(width, height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = (y * width + x) * 4;
            image[y][x] = png::rgba_pixel(pixels[index], pixels[index + 1], pixels[index + 2], pixels[index + 3]);
        }
    }
    image.write(filename);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

GLFWwindow* createWindow()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "3D Sphere", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cout << "Failed to initialize GLEW" << std::endl;
        return nullptr;
    }

    return window;
}

GLuint createShaderProgram(const char* vertexs,const char* fragments)
{
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexs, NULL);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragments, NULL);
    glCompileShader(fragmentShader);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return shaderProgram;
}

std::vector<float> getBallData(const int sectorCount, const int stackCount)
{
    std::vector<float> vertices;
    for (int i = 0; i <= stackCount; ++i) {
        float stackAngle = PI / 2 - i * PI / stackCount;
        float xy = 2.0f * cosf(stackAngle);
        float z = 2.0f * sinf(stackAngle);
        for (int j = 0; j <= sectorCount; ++j) {
            float sectorAngle = j * 2 * PI / sectorCount;
            float x = xy * cosf(sectorAngle);
            float y = xy * sinf(sectorAngle);
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
        }
    }

    return vertices;
}

int main()
{
    GLFWwindow* window = createWindow();
    if (window == NULL) {
        return -1;
    }

    glViewport(0, 0, WIDTH, HEIGHT);

// =================================================================================
    // 创建和编译坐标轴着色器
    CAxis axis("/res/shaders/axis_vertex.glsl","/res/shaders/axis_fragment.glsl");
    // 创建和编译ball着色器
    CPointCloud pointCloud("/res/shaders/sphere_vertex.glsl","/res/shaders/sphere_fragment.glsl");

    // 创建FBO
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    // 创建和编译坐标轴着色器
    CTexture texture("/res/shaders/texture_vertex.glsl","/res/shaders/texture_fragment.glsl");

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

// #########################################################################
    const bool _draw_screen = true;
    {
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer); // 绑定FBO，之后就能渲染到Texture中了（因为前面已经将Texture和绑定进行了绑定）
        
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // 绘制坐标轴
        axis.Rendering();
        // 绘制ball
        std::vector<float> sphereVertices = getBallData(sectorCount,stackCount);  // Create sphere vertices
        pointCloud.Rendering(sphereVertices);

        saveRenderbufferToPNG("texture_render_to_texture.png", 800, 600);
    }
    
    // 使用纹理渲染到屏幕上
    if(_draw_screen){
        texture.Rendering();
        saveRenderbufferToPNG("texture_render_to_screen.png", 800, 600);
    }

    glfwSwapBuffers(window);

    while (!glfwWindowShouldClose(window)) {
        processInput(window);
        glfwPollEvents();
    }
    glDeleteFramebuffers(1, &framebuffer);

    glfwTerminate();
    return 0;
}
