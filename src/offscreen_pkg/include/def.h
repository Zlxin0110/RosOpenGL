#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

const GLuint WIDTH = 800, HEIGHT = 600;
const float PI = 3.14159265359f;
const int sectorCount = 100;
const int stackCount = 100;

#define VIEW_EYE_POSITION           (glm::vec3(0.0f, 0.0f, 20.0f))
#define VIEW_LOOK_AT_POSITION       (glm::vec3(0.0f, 0.0f, 0.0f))
#define VIEW_CAMERA_UP_POSITION     (glm::vec3(0.0f, 1.0f, 0.0f))

#define PROJECTION_FOVY             (45.0f)
#define PROJECTION_ASPEC            ((float)WIDTH / (float)HEIGHT)
#define PROJECTION_ZNEAR            (10.0f)
#define PROJECTION_ZFAR             (100.0f)
