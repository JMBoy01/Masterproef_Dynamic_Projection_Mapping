// ModelLoader.h
#ifndef MODELLOADER_H
#define MODELLOADER_H

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "Vertex.h"

static std::vector<Vertex> LoadOBJ(const char* path)
{
    std::vector<glm::fvec3> v;
    std::vector<glm::fvec3> vn;
    std::vector<glm::fvec2> vt;

    std::vector<GLint> v_i;
    std::vector<GLint> vn_i;
    std::vector<GLint> vt_i;

    std::vector<Vertex> vertices;

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Fout bij openen van bestand: " << path << std::endl;
        return vertices;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string type;
        ss >> type;

        // Lees de vertices (v)
        if (type == "v") {
            glm::fvec3 vertex;
            ss >> vertex.x >> vertex.y >> vertex.z;
            v.push_back(vertex);
        }
        // Lees de normale vectoren (vn)
        else if (type == "vn") {
            glm::fvec3 normal;
            ss >> normal.x >> normal.y >> normal.z;
            vn.push_back(normal);
        }
        // Lees de textuurcoördinaten (vt)
        else if (type == "vt") {
            glm::fvec2 texCoord;
            ss >> texCoord.x >> texCoord.y;
            vt.push_back(texCoord);
        }
        // Lees de faces (f)
        else if (type == "f") {
            /* NEW - face parser code */
            GLint glint;

            int counter = 0;
            while (ss >> glint) {
                if (counter == 0)
                    v_i.push_back(glint);
                else if (counter == 1)
                    vt_i.push_back(glint);
                else if (counter == 2)
                    vn_i.push_back(glint);

                if (ss.peek() == '/') {
                    ++counter;
                    ss.ignore(1, '/');
                }
                else if (ss.peek() == ' ') {
                    ++counter;
                    ss.ignore(1, ' ');
                }

                if (counter > 2) {
                    counter = 0;
                }
            }

            /* OLD - face parser code */
            /*std::vector<glm::fvec3> faceVertices;
            std::vector<glm::fvec3> faceNormals;
            std::vector<glm::fvec2> faceTextures;

            // Lees de face gegevens in een dynamische lijst, afhankelijk van het aantal vertices in de face.
            unsigned int faceElementAmount = 0;
            std::string faceElement;
            while (ss >> faceElement) {
                unsigned int position, texture, normal;
                char slash;

                // Parse de face in de vorm van vertex/texcoord/normal (bijvoorbeeld 1/1/1)
                std::istringstream fs(faceElement);
                fs >> position >> slash >> texture >> slash >> normal;

                // Converteer naar 0-gebaseerde indexen
                faceVertices.push_back(v[position - 1]); // Converteer naar 0-gebaseerde index
                faceNormals.push_back(vn[normal - 1]);  // Converteer naar 0-gebaseerde index
                faceTextures.push_back(vt[texture - 1]);    // Converteer naar 0-gebaseerde index

                faceElementAmount++;
            }

            unsigned int trianglesInFace = faceElementAmount - 2;
            std::vector<unsigned int> vertexOrder;
            for (size_t i = 0; i < trianglesInFace; ++i) {
                vertexOrder.push_back(0);
                vertexOrder.push_back(i + 1);
                vertexOrder.push_back(i + 2);
            }

            // Voeg de vertices van de face toe aan de lijst van vertices
            for (unsigned int i : vertexOrder) {
                Vertex vertex;

                vertex.Position = faceVertices[i];
                vertex.Normal = faceNormals[i];
                vertex.TexCoord = faceTextures[i];

                vertices.push_back(vertex);
            }*/
        }
    }

    file.close();

    for (size_t i = 0; i < v_i.size(); ++i) {
        Vertex vertex;
        vertex.Position = v[v_i[i] - 1];
        vertex.TexCoord = vt[vt_i[i] - 1];
        vertex.Normal = vn[vn_i[i] - 1];

        vertex.TexCoord.y = 1.0f - vertex.TexCoord.y;

        vertices.push_back(vertex);
    }

    return vertices;
}


#endif // MODELLOADER_H