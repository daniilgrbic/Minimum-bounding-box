#include <iostream>
#include <vector>
#include <algorithm>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "utility/loadobj.h"
#include "utility/loadshaders.h"
#include "utility/controls.h"

GLFWwindow* window;

int main() {
    if(not glfwInit()) {
        return std::cout << "Failed to initialize GLFW" << std::endl, -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(1024, 768, "MBB", nullptr, nullptr);
    if(window == nullptr) {
        return glfwTerminate(), std::cout << "Failed to open GLFW window" << std::endl, -1;
    }
    glfwMakeContextCurrent(window);

    glewExperimental = true;
    if(glewInit() != GLEW_OK) {
        return glfwTerminate(), std::cout << "Failed to initialize GLEW" << std::endl, -1;
    }

    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);
    //glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    // glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec3> tempPoints;

    if(not loadObj("teapot_normals.obj", vertices, normals, tempPoints)) {
        return glfwTerminate(), -1;
    }

    unsigned int shaderProgramID = loadShaders("shaders/Vertex.shader", "shaders/Fragment.shader");

    unsigned int vertexArrayID;
    glGenVertexArrays(1, &vertexArrayID);
    glBindVertexArray(vertexArrayID);

    unsigned int vertexBufferID;
    glGenBuffers(1, &vertexBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

    unsigned int normalBufferID;
    glGenBuffers(1, &normalBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, normalBufferID);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);

    unsigned int matrixUniform = glGetUniformLocation(shaderProgramID, "u_MVP");
    unsigned int viewMatrixUniform = glGetUniformLocation(shaderProgramID, "u_V");
    unsigned int modelMatrixUniform = glGetUniformLocation(shaderProgramID, "u_M");
    unsigned int lightPositionUniform = glGetUniformLocation(shaderProgramID, "u_LightPosition_world");
    unsigned int transparencyUniform = glGetUniformLocation(shaderProgramID, "u_Transparency");
    unsigned int lightColorUniform = glGetUniformLocation(shaderProgramID, "u_LightColor");

    glUseProgram(shaderProgramID);

    glm::vec3 lightPos = glm::vec3(10,10,10);

    do {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        computeVPMatricesFromInputs();
        glm::mat4 ProjectionMatrix = getProjectionMatrix();
        glm::mat4 ViewMatrix = getViewMatrix();
        glm::mat4 ModelMatrix = glm::translate(glm::mat4(1.0), glm::vec3(0, 0, 0));
        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

        glUniformMatrix4fv(matrixUniform, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(modelMatrixUniform, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(viewMatrixUniform, 1, GL_FALSE, &ViewMatrix[0][0]);
        glUniform3f(lightPositionUniform, lightPos.x, lightPos.y, lightPos.z);
        glUniform1f(transparencyUniform, 0.5);
        glUniform3f(lightColorUniform, 1, 0, 0);

        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, normalBufferID);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);

        glDrawArrays(GL_TRIANGLES, 0, vertices.size());
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        glfwSwapBuffers(window);
        glfwPollEvents();

    } while(not glfwWindowShouldClose(window));

    glDeleteBuffers(1, &vertexBufferID);
    glDeleteBuffers(1, &normalBufferID);
    glDeleteProgram(shaderProgramID);
    glDeleteVertexArrays(1, &vertexArrayID);

    return glfwTerminate(), 0;
}
