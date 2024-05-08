#include "camera.h"

// ====================================================================
// Class: Camera
// ====================================================================
Camera::Camera(glm::vec3 Position, glm::vec3 Up, glm::vec3 Right) {
	mPosition = Position;
	mUp = Up;
	mRight = Right;
}

Camera::~Camera() {

}

glm::mat4 Camera::getViewMatrix() {
	//创建lookAt函数需要的参数
	glm::vec3 front = glm::cross(mUp, mRight);
	glm::vec3 center = mPosition + front;

	//返回lookAt函数值
	return glm::lookAt(mPosition, center, mUp);
}

glm::mat4 Camera::getProjectionMatrix() {
	return glm::identity<glm::mat4>();
}

void Camera::scale(float deltaScale) {
	//等待被覆写
}

// ====================================================================
// Class: PerspectiveCamera
// ====================================================================
PerspectiveCamera::PerspectiveCamera(
	float fovy,
	float aspect,
	float near,
	float far,
	glm::vec3 Position,
	glm::vec3 Up,
	glm::vec3 Right
):Camera(Position,Up,Right) {
	mFovy = fovy;
	mAspect = aspect;
	mNear = near;
	mFar = far;
}

PerspectiveCamera::~PerspectiveCamera() {

}

glm::mat4 PerspectiveCamera::getProjectionMatrix() {
	//std::cout << "投影变换" << std::endl;
	//注意：传入的fovy是角度，需要转化为弧度
	return glm::perspective(glm::radians(mFovy), mAspect, mNear, mFar);
}

void PerspectiveCamera::scale(float deltaScale) {
	auto frount = glm::cross(mUp, mRight);
	mPosition += (frount * deltaScale);
}

// ====================================================================
// Class: OrthographicCamera
// ====================================================================
OrthographicCamera::OrthographicCamera(float l,
	float r,
	float t,
	float b,
	float n,
	float f,
	glm::vec3 Position,
	glm::vec3 Up,
	glm::vec3 Right
	):Camera(Position, Up, Right)
{
	mLeft = l;
	mRight = r;
	mTop = t;
	mBottom = b;
	mNear = n;
	mFar = f;
}

OrthographicCamera::~OrthographicCamera() {

}

glm::mat4 OrthographicCamera::getProjectionMatrix() {
	float scale = std::pow(2.0f, mScale);
	return glm::ortho(mLeft * scale, mRight * scale, mBottom * scale, mTop * scale, mNear, mFar);
}

void OrthographicCamera::scale(float deltaScale) {
	mScale += deltaScale;
}

// ====================================================================
// Class: CameraControl
// ====================================================================

CameraControl::CameraControl() {

}

CameraControl::~CameraControl() {

}

//函数接口
void CameraControl::onMouse(int button, int action, double xpos, double ypos)
 {
	std::cout << "onMouse触发" << std::endl;
	//01、判断当前按键是否按下
	bool pressed = action == GLFW_PRESS ? true : false;

	//02、按下时记录按键位置
	if (pressed) {
		mCurrentX = xpos;
		mCurrentY = ypos;
	}

	//03、根据按下的鼠标按键，更改按键记录的状态
	switch (button) {
	case GLFW_MOUSE_BUTTON_LEFT:
		mLeftMouseDown = pressed;
		break;
	case GLFW_MOUSE_BUTTON_RIGHT:
		mRightMouseDown = pressed;
		break;
	case GLFW_MOUSE_BUTTON_MIDDLE:
		mMiddleMouseDown = pressed;
		break;
	}
}

void CameraControl::onCursor(double xpos, double ypos) {
	std::cout << "onCursor(鼠标移动)触发" << std::endl;
	//没有公共部分，由各自实现
}

void CameraControl::onKey(int key, int action, int mods) {
	std::cout << "onKey触发" << std::endl;
	//过滤repeat（按住）的情况
	if (action == GLFW_REPEAT) {
		return;
	}
	
	//01、判断当前按键是否按下
	bool pressed = action == GLFW_PRESS ? true : false;
	
	//02、记录在keyMap中
	mKeyMap[key] = pressed;
}

void CameraControl::onScroll(float offset) {

}

//每帧更新
void CameraControl::update() {
	//没有公共部分，由各自实现
}


// ====================================================================
// Class: TrackBallCameraControl
// ====================================================================
TrackBallCameraControl::TrackBallCameraControl()
{
}

TrackBallCameraControl::~TrackBallCameraControl()
{
}

void TrackBallCameraControl::onCursor(double xpos, double ypos) {

	//if (mMiddleMouseDown && !mKeyMap[GLFW_KEY_LEFT_SHIFT]) {
	if (mLeftMouseDown) {
	//当左键按下时（父类中已经实现）//调整摄像机参数
		//01、计算鼠标位移得出旋转的增量角度(sen敏感度）正负皆可
		float deltaX = (xpos - mCurrentX) * mSensitivity;
		float deltaY = (ypos - mCurrentY) * mSensitivity;

	//02、分开 pitch 和 yaw 各自计算//逻辑封装在函数中
		pitch(deltaY);//负号调整鼠标方向
		yaw(deltaX);
	}

	//实现按住shift同时按住鼠标中间平移
	if (mMiddleMouseDown && mKeyMap[GLFW_KEY_LEFT_SHIFT]) {
		float deltaX = (xpos - mCurrentX) * moveSpeed;
		float deltaY = (ypos - mCurrentY) * moveSpeed;

		mCamera->mPosition += mCamera->mUp * deltaY;
		mCamera->mPosition -= mCamera->mRight * deltaX;
	}

	//完成后更新x,y的值***别忘了****
	mCurrentX = xpos;
	mCurrentY = ypos;
}

void TrackBallCameraControl::onScroll(float offset) {
	mCamera->scale(mScaleSpeed * offset);
}

//封装pitch角旋转函数
void TrackBallCameraControl::pitch(float angle) {
	//绕right向量旋转
	auto mat = glm::rotate(glm::mat4(1.0f), glm::radians(angle), mCamera->mRight);

	//影响当前相机的up向量和位置（增量变换）
			//将up向量提升为四维(1代表点0代表向量)
	mCamera->mUp = mat * glm::vec4(mCamera->mUp, 0.0f);//四维向量可以給3维（给xyz）
	mCamera->mPosition = mat * glm::vec4(mCamera->mPosition, 1.0f);
}

void TrackBallCameraControl::yaw(float angle) {
	auto mat = glm::rotate(glm::mat4(1.0f), glm::radians(angle), glm::vec3(0.0f,1.0f,0.0f));

	mCamera->mUp = mat * glm::vec4(mCamera->mUp, 0.0f);
	mCamera->mRight = mat * glm::vec4(mCamera->mRight, 0.0f);
	mCamera->mPosition = mat * glm::vec4(mCamera->mPosition, 1.0f);
}


// ====================================================================
// Class: GameCameraControl
// ====================================================================

GameCameraControl::GameCameraControl()
{
}

GameCameraControl::~GameCameraControl()
{
}

void GameCameraControl::onCursor(double xpos, double ypos) {
	float deltaX = (xpos - mCurrentX) * mSensitivity;
	float deltaY = (ypos - mCurrentY) * mSensitivity;

	if (mRightMouseDown) {
		pitch(-deltaY);
		yaw(-deltaX);
	}

	mCurrentX = xpos;
	mCurrentY = ypos;
}

void GameCameraControl::update() {
	glm::vec3 direction(0.0f);//最终移动方向
	glm::vec3 upDown(0.0f);//上下方向

	auto front = glm::cross(mCamera->mUp, mCamera->mRight);//计算前向量
	auto right = mCamera->mRight;
	auto up = mCamera->mUp;

	//移动逻辑实现
	if (mKeyMap[GLFW_KEY_W]) {
		direction += front;
	}

	if (mKeyMap[GLFW_KEY_S]) {
		direction -= front;
	}
	
	if (mKeyMap[GLFW_KEY_A]) {
		direction -= right;
	}

	if (mKeyMap[GLFW_KEY_D]) {
		direction += right;
	}

	if (mKeyMap[GLFW_KEY_E]) {
		upDown += up;
	}

	if (mKeyMap[GLFW_KEY_Q]) {
		upDown -= up;
	}

	//此时direction有可能不为1，所以要进行归一化
		//但是要排除为0的情况
	if (glm::length(direction) != 0) {
		direction = glm::normalize(direction);
		mCamera->mPosition += direction * mSpeed;
	}

	//upDown = glm::normalize(upDown);
	mCamera->mPosition += upDown * mScaleSpeed;

	//固定鼠标位置
	if (mKeyMap[GLFW_KEY_L]) {
		// glfwSetCursorPos(application->getWindow(), application->getWidth() / 2, application->getHidth() / 2);
	}

}






//***********工具函数*************
void GameCameraControl::pitch(float angle) {
	//抬头角度限制
	mPitch += angle;
	if (mPitch > 89.9f || mPitch < -89.9f) {
		mPitch -= angle;
		return;
	}

	//在gameCameraControl下，pitch不会影响位置
	auto mat = glm::rotate(glm::mat4(1.0f), glm::radians(angle), mCamera->mRight);
	mCamera->mUp = mat * glm::vec4(mCamera->mUp, 0.0f);
}

void GameCameraControl::yaw(float angle) {
	auto mat = glm::rotate(glm::mat4(1.0f), glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
	mCamera->mUp = mat * glm::vec4(mCamera->mUp, 0.0f);
	mCamera->mRight = mat * glm::vec4(mCamera->mRight, 0.0f);
}
