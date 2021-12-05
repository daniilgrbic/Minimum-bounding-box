#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <glm/glm.hpp>

#include "loadobj.h"

bool loadObj(
        const std::string& path,
        std::vector<glm::vec3> & vertices,
        std::vector<glm::vec3> & normals,
        std::vector<glm::vec3> & uniqueVertices) {

    std::cout << "Loading " << path << "..." << std::endl;

    std::vector<unsigned long> vertexIndices, normalIndices;
    std::vector<glm::vec3> tempNormals;

    bool bHasNormals = false;
    bool bHasTexture = false;

    std::ifstream in(path, std::ios::in);
    if(not in) {
        std::cerr << "Cannot open " << path << std::endl;
        return false;
    }

    try {
        std::string line;
        while(std::getline(in, line)) {
            if(line.substr(0,2) == "v ") {
                std::istringstream s(line.substr(2));
                glm::vec3 vertex;
                double x,y,z;
                s >> x; s >> y; s >> z;
                vertex = glm::vec3(x, y, z);
                uniqueVertices.push_back(vertex);
            }
            else if(line.substr(0,2) == "vt") {
                bHasTexture = true;
            }
            else if(line.substr(0,2) == "vn") {
                std::istringstream s(line.substr(2));
                glm::vec3 normal;
                double x,y,z;
                s >> x; s >> y; s >> z;
                normal = glm::vec3(x, y, z);
                tempNormals.push_back(normal);
                bHasNormals = true;
            }
            else if(line.substr(0,2) == "f ") {
                line.erase(0,2);
                if(bHasNormals and !bHasTexture) {
                    //std::cout << line << " : " << std::endl;
                    vertexIndices.push_back(std::stoul(line.substr(0, line.find('/'))));
                    line.erase(0, line.find('/') + 2);
                    normalIndices.push_back(std::stoul(line.substr(0, line.find(' '))));
                    line.erase(0, line.find(' ') + 1);
                    //std::cout << "(" << vertexIndices[vertexIndices.size()-1] << "," << normalIndices[normalIndices.size()-1] << ")" << std::endl;

                    vertexIndices.push_back(std::stoul(line.substr(0, line.find('/'))));
                    line.erase(0, line.find('/') + 2);
                    normalIndices.push_back(std::stoul(line.substr(0, line.find(' '))));
                    line.erase(0, line.find(' ') + 1);

                    vertexIndices.push_back(std::stoul(line.substr(0, line.find('/'))));
                    line.erase(0, line.find('/') + 2);
                    normalIndices.push_back(std::stoul(line));
                }
                else {
                    std::cerr << "This file is not supported" << std::endl;
                    return false;
                }
            }
        }
    }
    catch (...) {
        std::cerr << "An error occured while parsing " << path << std::endl;
        return false;
    }

    for(unsigned int i = 0; i < vertexIndices.size(); i++) {
        unsigned int vertexIndex = vertexIndices[i];
        unsigned int normalIndex = normalIndices[i];

        glm::vec3 vertex = uniqueVertices[vertexIndex-1];
        glm::vec3 normal = tempNormals[normalIndex-1];

        vertices.push_back(vertex);
        normals.push_back(normal);
    }

    in.close();
    return true;
}