#include <iostream>
#include <fstream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen>

#include "glad/glad.h"
#include "GLFW/glfw3.h"

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

int main() {
    // Initialize GLFW context
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    GLFWwindow* window = glfwCreateWindow(800, 600, "Galaxy Garden v2", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD." << std::endl;
        return -1;
    }

    // OpenGL config
    glViewport(0, 0, 800, 600);

    // Set window callbacks
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // Render loop
    while (!glfwWindowShouldClose(window)) {
        // Render
        glClear(GL_COLOR_BUFFER_BIT);

        // Process events and swap buffers
        glfwPollEvents();
        glfwSwapBuffers(window);
    } 

    glfwTerminate();
    return 0;
}
