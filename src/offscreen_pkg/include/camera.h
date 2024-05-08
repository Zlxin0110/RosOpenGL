#pragma once
#include "def.h"
#include <map>

// ====================================================================
// Class: Camera
// ====================================================================
class Camera
{
public:
	//每帧实时计算并存储到Camera中：
	glm::vec3 mPosition{ 0.0f,0.0f,1.0f };//eye参数
	glm::vec3 mUp{ 0.0f,1.0f,0.0f };//up向量
	glm::vec3 mRight{ 1.0f,0.0f,0.0f };//

public:
	Camera(glm::vec3 Position, glm::vec3 Up, glm::vec3 Right);
	~Camera();

	glm::mat4 getViewMatrix();

	virtual glm::mat4 getProjectionMatrix();
	virtual void scale(float deltaScale);//获取鼠标滚轮
};

// ====================================================================
// Class: PerspectiveCamera
// ====================================================================
class PerspectiveCamera :public Camera {
public:
	PerspectiveCamera(
		float fovy,
		float aspect,
		float near,
		float far,
		glm::vec3 Position,
		glm::vec3 Up,
		glm::vec3 Right
	);
	~PerspectiveCamera();

	glm::mat4 getProjectionMatrix()override;
	void scale(float deltaScale)override;//��д�����ֶ�ȡ����

private:
	float mFovy = 0.0f;
	float mAspect = 0.0f;
	float mNear = 0.0f;
	float mFar = 0.0f;
};

// ====================================================================
// Class: OrthographicCamera
// ====================================================================
class OrthographicCamera :public Camera
{
public:
	OrthographicCamera(float l,
		float r,
		float t,
		float b,
		float n,
		float f,
		glm::vec3 Position,
		glm::vec3 Up,
		glm::vec3 Right
	);
	~OrthographicCamera();

	glm::mat4 getProjectionMatrix()override;
	void scale(float deltaScale)override;

private://一次赋值后无需更改
	float mLeft = 0.0f;
	float mRight = 0.0f;
	float mTop = 0.0f;
	float mBottom = 0.0f;
	float mNear = 0.0f;
	float mFar = 0.0f;

	float mScale{ 0.0f };//记录正交盒的大小变化
};




// ====================================================================
// Class: CameraControl
// ====================================================================
class CameraControl
{
public:
	CameraControl();
	~CameraControl();

	//函数接口
	virtual void onMouse(int button, int action, double xpos, double ypos);
	virtual void onCursor(double xpos, double ypos);
	virtual void onKey(int key, int action, int mods);
	virtual void onScroll(float offset);

	//每帧更新
	virtual void update();

	void setCamera(Camera* camera) { mCamera = camera; }
	auto getCamera() { return mCamera; }
	void setSensitivity(float s) { mSensitivity = s; }
	void setScaleSpeed(float s) { mScaleSpeed = s; }

protected://使用受保护；方便派生类使用
	//01鼠标按键
	bool mLeftMouseDown = false;
	bool mRightMouseDown = false;
	bool mMiddleMouseDown = false;

	//02鼠标位置
	float mCurrentX = 0.0f, mCurrentY = 0.0f;

	//03敏感度
	float mSensitivity = 0.05f;

	//04记录键盘相关按键的按下状态
	std::map<int, bool> mKeyMap;

	//05存储当前控制的摄像机
	Camera* mCamera = nullptr;

	//06记录相机缩放速度
	float mScaleSpeed = 0.2f;
};

// ====================================================================
// Class: TrackBallCameraControl
// ====================================================================
class TrackBallCameraControl :public CameraControl
{
public:
	TrackBallCameraControl();
	~TrackBallCameraControl();

	//父类中，需要重写的函数
	void onCursor(double xpos, double ypos)override;
	void onScroll(float offset)override;

private:
	float moveSpeed = 0.001f;
	//供自己调用的工具函数
		////02、分开 pitch 和 yaw 各自计算
	void pitch(float angle);
	void yaw(float angle);

};

// ====================================================================
// Class: GameCameraControl
// ====================================================================
class GameCameraControl : public CameraControl
{
public:
	GameCameraControl();
	~GameCameraControl();

	//函数接口
	void onCursor(double xpos, double ypos)override;//覆写鼠标移动函数，控制视角
	void update()override;//覆写更新函数，监控键盘状态
	void setSpeed(float speed) { mSpeed = speed; }
private:
	//工具函数
	void pitch(float angle);
	void yaw(float angle);
	//变量存储
	float mPitch{ 0.0f };//记录角度，用来限制角度
	float mSpeed{ 0.1f };//移动速度
};