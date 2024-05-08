#include <ros/ros.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdlib>
#include <vector>
#include <math.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <queue>
#include <mutex>
#include <condition_variable>

#define TOPIC_NOT_SUB                       (0) // Topicを購読しない、点群描画
void drawAxes(void)
{
    glBegin(GL_LINES);
    // x軸（red）
    glColor3f(1.0, 0.0, 0.0); 
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(10.0, 0.0, 0.0);
    // y軸（green）
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 10.0, 0.0);
    // z軸（blue）
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 10.0);
    glEnd();
}
#if TOPIC_NOT_SUB
const int numPoints = 1000;
const float radius = 2.0f;
GLuint vbo = 0; // 初始化为0，表示未创建

void createSpherePointCloud() {
    std::vector<float> vertices;
    vertices.reserve(numPoints * 3);

    for (int i = 0; i < numPoints; ++i) {
        float u = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
        float v = static_cast<float>(rand()) / RAND_MAX * M_PI;

        float x = radius * cos(u) * sin(v);
        float y = radius * sin(u) * sin(v);
        float z = radius * cos(v);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(z);
    }

    // 创建VBO并传输数据
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
}

void drawSpherePointCloud() {
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(3, GL_FLOAT, 0, nullptr);
    glDrawArrays(GL_POINTS, 0, numPoints);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    drawSpherePointCloud();

    glutSwapBuffers();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, static_cast<float>(w) / static_cast<float>(h), 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void cleanup() {
    if (vbo != 0) {
        glDeleteBuffers(1, &vbo);
    }
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Sphere Point Cloud");

    glewInit(); // 初始化 GLEW 库

    glEnable(GL_DEPTH_TEST);

    createSpherePointCloud(); // 创建球体点云

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    atexit(cleanup);

    glutMainLoop();

    return 0;
}
#endif


GLuint vbo; // VBO对象


// FIFO队列和相关的互斥量和条件变量
std::queue<sensor_msgs::PointCloud2::ConstPtr> point_cloud_queue;
std::mutex mtx;
std::condition_variable cv;

void drawSpherePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::cout << "+++++drawSpherePointCloud" << std::endl;
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(3, GL_FLOAT, 0, cloud.points.data());
    glDrawArrays(GL_POINTS, 0, cloud.points.size());
    glDisableClientState(GL_VERTEX_ARRAY);
}


void display() {
    std::cout << "+++++display" << std::endl;
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    //gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    while (true) {
        mtx.lock(); // ロック
        if (!point_cloud_queue.empty()) {
            std::cout << "point_cloud_queue.size() :" << point_cloud_queue.size() << std::endl;
            // キューから点群データを取得する。
            sensor_msgs::PointCloud2::ConstPtr msg = point_cloud_queue.front();
            point_cloud_queue.pop();
            mtx.unlock(); // ロック解除

            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*msg, cloud);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glLoadIdentity();

            // カメラの位置（視点）
            gluLookAt(5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            drawAxes();

            drawSpherePointCloud(cloud);

            // 画面を更新する。
            glutSwapBuffers();
            //glutPostRedisplay(); // OpenGLへ再描画を通知する。
        }else{
            mtx.unlock(); // ロック解除
        }
        sleep(5);
    }
}

void reshape(int w, int h) {
    std::cout << "+++++reshape" << std::endl;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, static_cast<float>(w) / static_cast<float>(h), 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

// 订阅点云 topic 的回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::cout << "+++++pointCloudCallback" << std::endl;
    mtx.lock();                     // ロック
    point_cloud_queue.push(msg);    // 受信したポイントクラウドデータをキューに格納します
    mtx.unlock();                   // ロック解除
}
void cleanup() {
    if (vbo != 0) {
        glDeleteBuffers(1, &vbo);
    }
}
// 线程函数，用于处理 FIFO 中的数据并绘制
void processPointCloud(int argc, char** argv) {
#if 0
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Sphere Point Cloud");

    glewInit();
    glEnable(GL_DEPTH_TEST);

    // 创建 VBO
    glGenBuffers(1, &vbo);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    
    glutMainLoop() ;
 #else
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Sphere Point Cloud");

    glewInit(); // 初始化 GLEW 库

    glEnable(GL_DEPTH_TEST);
//----
/*
    glClearColor(0.0, 0.0, 0.0, 0.0);

    // プロジェクション行列を設定します
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 800.0 / 600.0, 1.0, 100.0); // 視野角60度

    // モデルビュー行列を設定します
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    */
///-----


    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    atexit(cleanup);

    glutMainLoop();
#endif
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_visualization_node");
    ros::NodeHandle nh;

    // 启动处理点云的线程
    std::thread pointCloudThread(processPointCloud, argc, argv);

    // 订阅点云 topic，并将数据传递给回调函数
    ros::Subscriber sub = nh.subscribe("/lidar_points", 1, pointCloudCallback);

    ros::spin(); // 处理 ROS 消息队列

    // 清理资源
    glDeleteBuffers(1, &vbo);
    pointCloudThread.join();

    return 0;
}