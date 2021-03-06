#include <iostream>
#include <vector>
#include <algorithm>

#define GLFW_INCLUDE_GLCOREARB
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "utility/loadobj.h"
#include "utility/loadshaders.h"
#include "utility/controls.h"
#include "mbb.h"

GLFWwindow* window;

glm::vec3 ptToVec3(struct pt3 point) {
    glm::vec3 result = glm::vec3(point.x, point.y, point.z);
    return result;
}

void calculateNormals(std::vector<glm::vec3>&vertices, std::vector<glm::vec3>&normals) {
    normals.clear();
    for(int i = 0; i < vertices.size(); i+= 3) {
        glm::vec3 BMinusA = vertices[i+1] - vertices[i+0];
        glm::vec3 CMinusA = vertices[i+2] - vertices[i+0];
        glm::vec3 dir = glm::cross(BMinusA, CMinusA);
        glm::vec3 normal = glm::normalize(dir);
        normals.push_back(normal); normals.push_back(normal); normals.push_back(normal);
    }
}

int main(int argc, char ** argv) {
    if(not glfwInit()) {
        return std::cout << "Failed to initialize GLFW" << std::endl, -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    #ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

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
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    // glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec3> tempPoints;

    if(not loadObj(argc > 1 ? argv[1] : "teapot.obj", vertices, normals, tempPoints)) {
        return glfwTerminate(), -1;
    }

    if(normals.empty()) {
        calculateNormals(vertices, normals);
    }

    glm::vec3 lowerBoundary = tempPoints[0];
    glm::vec3 upperBoundary = tempPoints[0];
    std::vector<pt3> points;
    for(auto v : tempPoints) {
        pt3 P(v.x, v.y, v.z);
        points.push_back(P);
        lowerBoundary.x = std::min(lowerBoundary.x, v.x);
        lowerBoundary.y = std::min(lowerBoundary.y, v.y);
        lowerBoundary.z = std::min(lowerBoundary.z, v.z);
        upperBoundary.x = std::max(upperBoundary.x, v.x);
        upperBoundary.y = std::max(upperBoundary.y, v.y);
        upperBoundary.z = std::max(upperBoundary.z, v.z);
    }
    glm::vec3 modelCenter = lowerBoundary + upperBoundary;
    modelCenter /= 2;

    std::vector<glm::vec3> lower, upper;

    {
        std::vector<pt3> _lower(4), _upper(4);
        mbbApproximation(points, _lower, _upper);
        glm::vec3 boxCenter = glm::vec3(0, 0, 0);

        std::cout << "Lower base:" << std::endl;
        for(auto point : _lower) {
            point.show();
            boxCenter += ptToVec3(point);
            lower.push_back(ptToVec3(point));
        }
        std::cout << "Upper base:" << std::endl;
        for(auto point : _upper) {
            point.show();
            boxCenter += ptToVec3(point);
            upper.push_back(ptToVec3(point));
        }

        boxCenter /= 8;
        for(int i = 0; i < 4; i++) {
            glm::vec3 centerToPoint;
            centerToPoint = glm::normalize(lower[i] - boxCenter);
            centerToPoint /= 1000;
            lower[i] += centerToPoint;
            centerToPoint = glm::normalize(upper[i] - boxCenter);
            centerToPoint /= 1000;
            upper[i] += centerToPoint;
        }
    }

    std::vector<glm::vec3> boxVertices;
    std::vector<glm::vec3> boxNormals;

    boxVertices.push_back(lower[0]); boxVertices.push_back(lower[1]); boxVertices.push_back(lower[2]);
    boxVertices.push_back(lower[0]); boxVertices.push_back(lower[2]); boxVertices.push_back(lower[3]);
    boxVertices.push_back(upper[0]); boxVertices.push_back(upper[2]); boxVertices.push_back(upper[1]);
    boxVertices.push_back(upper[0]); boxVertices.push_back(upper[3]); boxVertices.push_back(upper[2]);

    boxVertices.push_back(lower[2]); boxVertices.push_back(lower[1]); boxVertices.push_back(upper[2]);
    boxVertices.push_back(lower[1]); boxVertices.push_back(upper[1]); boxVertices.push_back(upper[2]);
    boxVertices.push_back(lower[3]); boxVertices.push_back(upper[3]); boxVertices.push_back(lower[0]);
    boxVertices.push_back(lower[0]); boxVertices.push_back(upper[3]); boxVertices.push_back(upper[0]);

    boxVertices.push_back(lower[1]); boxVertices.push_back(lower[0]); boxVertices.push_back(upper[1]);
    boxVertices.push_back(lower[0]); boxVertices.push_back(upper[0]); boxVertices.push_back(upper[1]);
    boxVertices.push_back(lower[2]); boxVertices.push_back(upper[2]); boxVertices.push_back(lower[3]);
    boxVertices.push_back(lower[3]); boxVertices.push_back(upper[2]); boxVertices.push_back(upper[3]);

    calculateNormals(boxVertices, boxNormals);

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

    unsigned int boxVertexBufferID;
    glGenBuffers(1, &boxVertexBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, boxVertexBufferID);
    glBufferData(GL_ARRAY_BUFFER, boxVertices.size() * sizeof(glm::vec3), &boxVertices[0], GL_STATIC_DRAW);

    unsigned int boxNormalBufferID;
    glGenBuffers(1, &boxNormalBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, boxNormalBufferID);
    glBufferData(GL_ARRAY_BUFFER, boxNormals.size() * sizeof(glm::vec3), &boxNormals[0], GL_STATIC_DRAW);

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
        glm::mat4 ModelMatrix = glm::translate(glm::mat4(1.0), -modelCenter);
        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

        glUniformMatrix4fv(matrixUniform, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(modelMatrixUniform, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(viewMatrixUniform, 1, GL_FALSE, &ViewMatrix[0][0]);
        glUniform3f(lightPositionUniform, lightPos.x, lightPos.y, lightPos.z);

        glUniform1f(transparencyUniform, 1);
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

        glUniform1f(transparencyUniform, 0.5);
        glUniform3f(lightColorUniform, 1, 1, 1);

        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, boxVertexBufferID);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, boxNormalBufferID);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
        glDrawArrays(GL_TRIANGLES, 0, boxVertices.size());
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        glfwSwapBuffers(window);
        glfwPollEvents();

    } while(not glfwWindowShouldClose(window));

    glDeleteBuffers(1, &vertexBufferID);
    glDeleteBuffers(1, &normalBufferID);
    glDeleteBuffers(1, &boxVertexBufferID);
    glDeleteBuffers(1, &boxNormalBufferID);
    glDeleteProgram(shaderProgramID);
    glDeleteVertexArrays(1, &vertexArrayID);

    return glfwTerminate(), 0;
}
