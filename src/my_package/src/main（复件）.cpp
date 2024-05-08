#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <GL/glut.h>
#include <iostream>
#include <cmath>
#include <thread>
#include <queue>

#define TOPIC_NOT_SUB                       (0) // Topicを購読しない、点群描画
#define TOPIC_SUB_DRAWIN_CALLBACK           (0) // Topicを購読し、Callback関数の中身に描画する。
#define TOPIC_IN_MAIN_GLUTLOOP_IN_SUB       (0) // ros::spin()をMainThreadで実行し、glutMainLoop()をSubThreadで実行する。
#define TOPIC_IN_SUB_GLUTLOOP_IN_MAIN       (0) // ros::spin()をSubThreadで実行し、glutMainLoop()をMainThreadで実行する。
// 座標軸描画
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

//=======================================================================
// 試験１：Topicをサブすクライマしない、点群描画
// 結論：glBegin、glVertex3d、glEndで点群を描画できる
//=======================================================================
#if TOPIC_NOT_SUB
// 球体描画
void generateSphere(float radius, int slices, int stacks) {
    glBegin(GL_POINTS);
    for (int i = 0; i <= slices; ++i) {
        double lat0 = M_PI * (-0.5 + (double)(i - 1) / slices);
        double z0  = radius * sin(lat0);
        double zr0 =  radius * cos(lat0);

        double lat1 = M_PI * (-0.5 + (double)i / slices);
        double z1 = radius * sin(lat1);
        double zr1 = radius * cos(lat1);

        for (int j = 0; j <= stacks; ++j) {
            double lng = 2 * M_PI * (double)(j - 1) / stacks;
            double x = cos(lng);
            double y = sin(lng);

            glVertex3d(x * zr0, y * zr0, z0);
            glVertex3d(x * zr1, y * zr1, z1);
        }
    }
    glEnd();
}

// 描画処理
void display() {
    std::cout << "++++++++display" << std::endl;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // カメラの位置（視点）
    gluLookAt(5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    // 座標軸描画
    drawAxes();

    // 球体描画
    glColor3f(1.0, 1.0, 1.0);
    generateSphere(3.0, 30, 30);

    glutSwapBuffers();
}

// メイン処理
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Sphere with Axes");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    // プロジェクション行列を設定します
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 800.0 / 600.0, 1.0, 100.0); // 視野角60度

    // モデルビュー行列を設定します
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // コールバック関数を登録します
    glutDisplayFunc(display);

    // GLUTのメインループに入ります
    glutMainLoop();

    return 0;
}
#endif

//=======================================================================
// 試験２：Topicを購読し、Callback関数の中身に描画する。
// 結論：glBegin、glVertex3d、glEndを実行されるが、点群が描画できない。
//      （glutMainLoop()必須）
//=======================================================================
#if TOPIC_SUB_DRAWIN_CALLBACK
// ROSのコールバック関数
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // 座標軸描画
    drawAxes();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 点群描画
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_POINTS);
    for (const auto& point : cloud.points) {
        //std::cout << "x: " << point.x << ",y: " << point.y << ",z: " << point.z << std::endl; 
        glVertex3d(point.x, point.y, point.z);
    }
    glEnd();
    GLenum error;
    while ((error = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL Error: " << gluErrorString(error) << std::endl;
    }

    glFlush();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_visualization_node");
    ros::NodeHandle nh;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Lidar Visualization");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glPointSize(2.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-10, 10, -10, 10, -1, 1);

    ros::Subscriber lidar_sub = nh.subscribe("/lidar_points", 1, lidarCallback);

    ros::spin();

    return 0;
}
#endif

//=======================================================================
// 試験３：ros::spin()をMainThreadで実行し、glutMainLoop()をSubThreadで実行する。
// 結論：①点群が描画できます。
//      ②pcl::fromROSMsg(*msg, cloud)で点群データの解析処理は便利になる。
//      ③glutPostRedisplay()を呼び出さないと、OpenGLに再描画しない。
//=======================================================================
#if TOPIC_IN_MAIN_GLUTLOOP_IN_SUB

std::mutex mtx; // ミューテックス、共有リソースを保護するために使用されます
std::queue<sensor_msgs::PointCloud2::ConstPtr> point_cloud_queue; // FIFOキュー、ポイントクラウドデータを格納するために使用されます

// ROS コールバック関数、受信したポイントクラウドデータを処理します
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    mtx.lock();                  // ロックする
    point_cloud_queue.push(msg); // 受信したポイントクラウドデータをキューに格納します
    mtx.unlock();                // ロックを解除します
}

// 描画処理
void display() {

    while (true) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        mtx.lock(); // ロック
        if (!point_cloud_queue.empty()) {
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

            // 点群描画
            glColor3f(1.0, 1.0, 1.0);
            glBegin(GL_POINTS);
            for (const auto& point : cloud.points) {
                //std::cout << "x: " << point.x << ",y: " << point.y << ",z: " << point.z << std::endl; 
                glVertex3d(point.x, point.y, point.z);
            }
            glEnd();
            
            // 画面を更新する。
            glutSwapBuffers();
        } else {
            mtx.unlock(); // ロック解除
        }
    }

    glutPostRedisplay(); // OpenGLへ再描画を通知する。
}

// OpenGL スレッド関数
void OpenGLThread(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Sphere with Axes");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    // プロジェクション行列を設定します
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 800.0 / 600.0, 1.0, 100.0); // 視野角60度

    // モデルビュー行列を設定します
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

     // コールバック関数を登録します
    glutDisplayFunc(display);

    // GLUTのメインループに入ります
    glutMainLoop();
}

// メイン関数
int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_visualization_node");
    ros::NodeHandle nh;

    // 点群トピックを購読し、コールバック関数を設定します
    ros::Subscriber lidar_sub = nh.subscribe("/lidar_points", 1, lidarCallback);

    // OpenGLスレッドを作成します
    std::thread gl_thread(OpenGLThread, argc, argv);

    // ROSメインループ
    ros::spin();

    // OpenGLスレッドの終了を待ちます
    gl_thread.join();

    return 0;
}

#endif

//=======================================================================
// 試験４：ros::spin()をSubThreadで実行し、glutMainLoop()をMainThreadで実行する。
// 結論：点群が描画できますが、Sleepしないと、lidarCallbackを呼び出せない。
//=======================================================================
#if TOPIC_IN_SUB_GLUTLOOP_IN_MAIN
std::queue<sensor_msgs::PointCloud2::ConstPtr> point_cloud_queue; // FIFOキュー、ポイントクラウドデータを格納するために使用されます
std::mutex mtx;                                                   // ミューテックス、共有リソースを保護するために使用されます

// 描画処理
void display() {
    std::cout << "++++++++display" << std::endl;

    mtx.lock(); // ロック
    if (!point_cloud_queue.empty()) {
        // キューから点群データを取得する。
        sensor_msgs::PointCloud2::ConstPtr msg = point_cloud_queue.front();
        point_cloud_queue.pop();
        mtx.unlock(); // ロック解除

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        //  カメラの位置（視点）
        gluLookAt(5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        
        // 座標軸描画
        drawAxes();

        // 点群描画
        glColor3f(1.0, 1.0, 1.0);
        glBegin(GL_POINTS);
        for (const auto& point : cloud.points) {
            //std::cout << "x: " << point.x << ",y: " << point.y << ",z: " << point.z << std::endl; 
            glVertex3d(point.x, point.y, point.z);
        }
        glEnd();

        // 画面を更新する。
        glutSwapBuffers();

        sleep(1);
    } else {
        mtx.unlock(); // ロック解除
    }
    glutPostRedisplay(); // OpenGLへ再描画を通知する。
}


void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::cout << "+++++lidarCallback" << std::endl;
    mtx.lock();                     // ロック
    point_cloud_queue.push(msg);    // 受信したポイントクラウドデータをキューに格納します
    mtx.unlock();                   // ロック解除
}

// ROSスレッド
void rosThread(int argc, char** argv) {
    ros::init(argc, argv, "lidar_visualization_node");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/lidar_points", 1, lidarCallback);
    ros::spin();
}

// メイン関数
int main(int argc, char** argv) {
  
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Sphere with Axes");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    // プロジェクション行列を設定します
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 800.0 / 600.0, 1.0, 100.0); // 視野角60度

    // モデルビュー行列を設定します
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // コールバック関数を登録します
    glutDisplayFunc(display);

    // ros thread start
    std::thread ros_thread(rosThread, argc, argv);

    // GLUTのメインループに入ります
    glutMainLoop();

    //  ROSスレッドの終了を待ちます
    ros_thread.join();

    return 0;
}
#endif



#include <zlib.h>
std::vector<unsigned char> decompressData(const std::vector<unsigned char>& compressedData) {
    // 分配足够的空间来存储解压缩后的数据
    uLongf uncompressedSize = /* 您需要知道解压缩后的数据大小 */;
    std::vector<unsigned char> uncompressedData(uncompressedSize);

    // 解压缩数据
    if (uncompress(uncompressedData.data(), &uncompressedSize, compressedData.data(), compressedData.size()) != Z_OK) {
        ROS_ERROR("Failed to decompress data.");
        return std::vector<unsigned char>();
    }

    // 调整解压缩后的数据大小
    uncompressedData.resize(uncompressedSize);

    return uncompressedData;
}


// ROS コールバック関数、受信したポイントクラウドデータを処理します
void lidarCallback(const sensor_msgs::UInt8MultiArray::ConstPtr& msg) {
    size_t dataSize = msg->data.size();
    std::cout << "dataSize : " << dataSize << std::endl;

    // 解压缩消息数据
    std::vector<unsigned char> uncompressedData = decompressData(msg->data);
    std::cout << "uncompressedData : " << uncompressedData << std::endl;
}
// メイン関数
int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_visualization_node");
    ros::NodeHandle nh;

    // 点群トピックを購読し、コールバック関数を設定します
    ros::Subscriber lidar_sub = nh.subscribe("/compressed_texture_pixels", 1, lidarCallback);

    // ROSメインループ
    ros::spin();

    // OpenGLスレッドの終了を待ちます
    gl_thread.join();

    return 0;
}