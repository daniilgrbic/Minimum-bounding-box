#ifndef MINIMUM_BOUNDING_BOX_CONTROLS_H
#define MINIMUM_BOUNDING_BOX_CONTROLS_H

void computeVPMatricesFromInputs();
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();
glm::vec3 getCameraPosition();

#endif //MINIMUM_BOUNDING_BOX_CONTROLS_H
