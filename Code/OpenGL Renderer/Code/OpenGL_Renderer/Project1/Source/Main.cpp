#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

#include <iostream>
#include <thread>

#include "ModelLoader.h"
#include "Mesh.h"
#include "UDPReceiver.h"
#include "UDPMessage.h"
#include "UpdateOpenGLParameters.h"

// simple shaders with default color (white)
/*const char* defaultVertexShaderSource = R"(
    #version 330 core
    layout(location = 0) in vec3 aPos;  // De positie van de vertex
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0f);  // Bereken de uiteindelijke positie
    }
)";


const char* defaultFragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;
    void main() {
        FragColor = vec4(1.0, 1.0, 1.0, 1.0);  // Witte kleur voor de draadframe
    }
)";*/


// simple texture shader
const char* textureVertexShaderSource = R"(
    #version 330 core
    layout(location = 0) in vec3 aPos;  
    layout(location = 1) in vec2 aTexCoord;  
    out vec2 TexCoord; 
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0f);
        TexCoord = aTexCoord;
    }
)";

const char* textureFragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;
    in vec2 TexCoord;
    uniform sampler2D texture1;
    void main() {
        FragColor = texture(texture1, TexCoord);
    }
)";

int main(void) {
    // Socket communication
    UDPReceiver receiver(5005);
    receiver.start();
    UDPMessage msg;

    std::thread receiverThread(&UDPReceiver::receiveData, &receiver, std::ref(msg));

    // OpenGL
    GLFWwindow* window;

    if (!glfwInit()) return -1;

    // Create borderless fullscreen window for the projector
    int monitorCount;
    GLFWmonitor** monitors = glfwGetMonitors(&monitorCount);

    if (monitorCount < 2) {
        std::cerr << "Less than two monitors connected." << std::endl;
        glfwTerminate();
        return -1;
    }

    GLFWmonitor* targetMonitor = monitors[1]; // ID 1 is typically the projector

    const GLFWvidmode* mode = glfwGetVideoMode(targetMonitor);

    // Set window hints for borderless fullscreen
    glfwWindowHint(GLFW_RED_BITS, mode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
    glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE); // This removes the border
    glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE); // Prevents minimization when focus is lost

    // Create the window at the projector's position
    window = glfwCreateWindow(mode->width, mode->height, "Projector View", NULL, NULL);

    // Set the window's position to the projector's screen
    int monitorX, monitorY;
    glfwGetMonitorPos(targetMonitor, &monitorX, &monitorY);
    glfwSetWindowPos(window, monitorX, monitorY);

    //window = glfwCreateWindow(1280, 800, "Camera view", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    //glViewport(0, 0, 1920, 1080);

    glEnable(GL_DEPTH_TEST); // Zorg ervoor dat de diepte goed wordt weergegeven

    // Shader setup
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &textureVertexShaderSource, nullptr);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &textureFragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glUseProgram(shaderProgram);

    Mesh myMesh = Mesh(LoadOBJ("Debug/Simple_House_Assembly.obj"), "Debug/Simple_House_Assembly_Texture_Marker_Black.png");

    // Print vertex amount
    //std::cout << "Aantal vertices geladen: " << myMesh.vertices.size() << std::endl;

    // Model pose
    glm::mat4 model = glm::mat4(1.0f);

    // Camera extrinsics, DON'T CHANGE THIS MATRIX
    glm::mat4 view = glm::mat4(1.0f);

    // Camera intrinsics
    glm::mat4 projection = glm::mat4(1.0f);


    GLuint modelLoc = glGetUniformLocation(shaderProgram, "model");
    GLuint viewLoc = glGetUniformLocation(shaderProgram, "view");
    GLuint projectionLoc = glGetUniformLocation(shaderProgram, "projection");
    GLuint textureLoc = glGetUniformLocation(shaderProgram, "texture1");

    glUniform1i(textureLoc, 0); // Set texture unit 0 for our texture sampler

    while (!glfwWindowShouldClose(window)) {
        // Handle new data
        switch(msg.flag) {
            case 'i':
                UpdateIntrinsics(projection, msg.data);
                break;
            case 'p':
                UpdatePose(model, msg.data); // 
                break;
        }

        // OpenGL
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

        myMesh.Draw();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // UDP receiver stop, 1) stop thread, 2) stop receiver
    if (receiverThread.joinable()) {
        receiverThread.join();
    }
    receiver.stop();

    glfwTerminate();
    return 0;
}