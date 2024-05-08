#include "ros/ros.h"
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
#include "render_buffer.h"
#include "render.h"
#include "camera.h"

//相机
Camera* camera = nullptr;
OrthographicCamera* orthographicCamera = nullptr;
PerspectiveCamera* perspectiveCamera = nullptr;
//相机控制
CameraControl* cameraControl = nullptr;
GameCameraControl* gameCameraControl = nullptr;
TrackBallCameraControl* trackBallCameraControl = nullptr;

// ======================================================================
float mFar = PROJECTION_ZFAR;
float mFovy = PROJECTION_FOVY;
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

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    cameraControl->onCursor(xpos, ypos);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    double start_x = 0.0f;
    double start_y = 0.0f;
    glfwGetCursorPos(window, &start_x, &start_y);
    cameraControl->onMouse(button, action, start_x, start_y);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    cameraControl->onScroll(yoffset);
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

    // 注册鼠标滚轮事件回调函数
    glfwSetScrollCallback(window, scroll_callback);
    // 注册鼠标移动事件回调函数
    glfwSetCursorPosCallback(window, cursor_position_callback);
    // 注册鼠标按键事件回调函数
    glfwSetMouseButtonCallback(window, mouse_button_callback);

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

void prepareCamera() {
    //相机公用数据
    glm::vec3 CPosition = VIEW_EYE_POSITION;
    glm::vec3 CUp = VIEW_CAMERA_UP_POSITION;
    glm::vec3 CRight = glm::vec3(1.0f, 0.0f, 0.0f);
    float size = 10.0f;

    std::string paraValue;
    //创建两个相机    
    if (ros::param::get("camera_type", paraValue) && paraValue == "perspective" ) {
        camera= new PerspectiveCamera(60.0f,PROJECTION_ASPEC,0.1f,1000.0f,glm::vec3(0.0f, 0.0f, 5.0f),glm::vec3(0.0f, 1.0f, 0.0f),glm::vec3(1.0f, 0.0f, 0.0f));
    }else{
        camera = new OrthographicCamera(-size, size, size, -size, size, -size, CPosition, CUp, CRight);
    }
    
    //创建两个相机控制
    if (ros::param::get("camera_control", paraValue) && paraValue == "track_ball" ) {
        cameraControl = new TrackBallCameraControl;
    }else{
        cameraControl = new GameCameraControl;
    }
    
    //切换相机
    cameraControl->setCamera(camera);
    cameraControl->setSensitivity(0.05f);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "offscreen_node");

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
    CRender *render = nullptr;
    std::string paraValue;
    if (ros::param::get("render_type", paraValue) && paraValue == "render_buffer" ) {
        std::cout << "CRanderBuffer()" << std::endl;
        render = new CRanderBuffer();
    }else{
        render = new CTexture();
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    prepareCamera();

    std::vector<float> sphereVertices = getBallData(sectorCount,stackCount);  // Create sphere vertices
    while (!glfwWindowShouldClose(window)) {
        // =================================================================================
        {
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer); // 绑定FBO，之后就能渲染到Texture中了（因为前面已经将Texture和绑定进行了绑定）
            glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            glm::mat4 model = glm::mat4(1.0f);
            glm::mat4 view = camera->getViewMatrix();
            glm::mat4 projection = camera->getProjectionMatrix();

            // 绘制坐标轴
            axis.Rendering(model, view, projection);
            // 绘制ball
            pointCloud.Rendering(model, view, projection, sphereVertices);
        }
        // ================================================================================= 
// 使用纹理渲染到屏幕上
#if BY_SHADER
        render->Rendering();
#else
        render->Rendering(framebuffer);
#endif

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glDeleteFramebuffers(1, &framebuffer);

    glfwTerminate();
    return 0;
}
