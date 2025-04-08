// Mesh.h
#ifndef MESH_H
#define MESH_H

#include <glm/glm.hpp>

#include <vector>
#include <string>

#include "Vertex.h"

class Mesh {
public:
    std::vector<Vertex> vertices;

    Mesh(const std::vector<Vertex>& vertices, const std::string& texturePath = "");

    void LoadTexture(const std::string& texturePath);

    void Draw();
    void DrawWithoutTexture();
    void DrawWireframe();

private:
    unsigned int VAO, VBO;
    unsigned int textureID;

    void SetupMesh();
};

#endif // MESH_H