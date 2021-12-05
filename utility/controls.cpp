#include <GLFW/glfw3.h>

extern GLFWwindow* window;

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>
using namespace glm;

#include "controls.h"

glm::mat4 ViewMatrix;
glm::mat4 ProjectionMatrix;
glm::vec3 cameraPosition = glm::vec3( 0, 0, 10 );
glm::mat4 getViewMatrix()
{
    return ViewMatrix;
}
glm::mat4 getProjectionMatrix()
{
    return ProjectionMatrix;
}
glm::vec3 getCameraPosition()
{
    return cameraPosition;
}

glm::vec3 up = glm::vec3(0, 1, 0);

float rotSpeed = 4.0f;
float zoomSpeed = 5.0f;

void computeVPMatricesFromInputs()
{
    static double lastTime = glfwGetTime();
    double currentTime = glfwGetTime();
    float deltaTime = float(currentTime - lastTime);
    lastTime = currentTime;

    glm::vec3 right = glm::cross(cameraPosition, up);

    if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS)
    {
        glm::mat4 rotationMat(1);
        rotationMat = glm::rotate(rotationMat, rotSpeed*deltaTime, right);
        if (glm::angle(glm::normalize(cameraPosition), up) > rotSpeed*deltaTime)
        {
            cameraPosition = glm::vec3(rotationMat * glm::vec4(cameraPosition, 1.0));
        }

    }

    if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS)
    {
        glm::mat4 rotationMat(1);
        rotationMat = glm::rotate(rotationMat, -rotSpeed*deltaTime, right);
        if (glm::angle(glm::normalize(cameraPosition), up) < 3.14f-rotSpeed*deltaTime)
        {
            cameraPosition = glm::vec3(rotationMat * glm::vec4(cameraPosition, 1.0));
        }
    }

    if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS)
    {
        glm::mat4 rotationMat(1);
        rotationMat = glm::rotate(rotationMat, rotSpeed*deltaTime, up);
        cameraPosition = glm::vec3(rotationMat * glm::vec4(cameraPosition, 1.0));
    }

    if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS)
    {
        glm::mat4 rotationMat(1);
        rotationMat = glm::rotate(rotationMat, -rotSpeed*deltaTime, up);
        cameraPosition = glm::vec3(rotationMat * glm::vec4(cameraPosition, 1.0));
    }

    if (glfwGetKey( window, GLFW_KEY_KP_ADD ) == GLFW_PRESS and glm::length(cameraPosition) > 3)
    {
        glm::mat4 scaleMat(1);
        scaleMat = glm::scale(scaleMat, glm::vec3(glm::length(cameraPosition) - 0.1));
        cameraPosition = glm::vec3(scaleMat * glm::vec4(glm::normalize(cameraPosition), 1.0));
    }

    if (glfwGetKey( window, GLFW_KEY_KP_SUBTRACT ) == GLFW_PRESS and glm::length(cameraPosition) < 20)
    {
        glm::mat4 scaleMat(1);
        scaleMat = glm::scale(scaleMat, glm::vec3(glm::length(cameraPosition) + 0.1));
        cameraPosition = glm::vec3(scaleMat * glm::vec4(glm::normalize(cameraPosition), 1.0));
    }

    ProjectionMatrix = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
    ViewMatrix = glm::lookAt(
            cameraPosition,
            glm::vec3(0,0,0),
            up
    );
}