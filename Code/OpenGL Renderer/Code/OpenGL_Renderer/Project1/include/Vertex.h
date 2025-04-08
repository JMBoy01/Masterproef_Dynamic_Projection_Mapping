// ModelLoader.h
#ifndef VERTEX_H
#define VERTEX_H

#include <glm/glm.hpp>

struct Vertex {
    glm::fvec3 Position;
    glm::fvec3 Normal;
    glm::fvec2 TexCoord;
};

#endif // VERTEX_H