#ifndef MINIMUM_BOUNDING_BOX_LOADOBJ_H
#define MINIMUM_BOUNDING_BOX_LOADOBJ_H

bool loadObj(
        const std::string & path,
        std::vector<glm::vec3> & out_vertices,
        std::vector<glm::vec3> & out_normals,
        std::vector<glm::vec3> & unique_vertices
);

#endif //MINIMUM_BOUNDING_BOX_LOADOBJ_H
