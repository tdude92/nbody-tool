#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen>
#include "nbodytool.hpp"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

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

    // Initialize simulator
    Integrator* integrator;
    ForceComputer* forceComputer;

    // Filename
    std::string datafileName = TOSTRING(DATA_FILE);
    std::ifstream datafile("data/2d/" + datafileName);
    std::cout << "Simulation Data: " << datafileName << std::endl;

    unit_t unitL, unitM, unitT;
    datafile >> unitL >> unitM >> unitT;

    // Integrator
    #ifdef EULER
        integrator = new EulerIntegrator();
    #else
        #error Integrator not defined
    #endif

    // Force Computer
    #ifdef GRAVITATIONAL_DIRECT
        forceComputer = new Gravitational_Direct(0.01, unitL, unitM, unitT);
    #else
        #error Force computer not defined
    #endif

    Simulator2d sim(1, 1000000, integrator, forceComputer);
    std::vector<Rigidbody> rigidbodies;
    double m, r, x, y, v_x, v_y;
    while (datafile >> m >> r >> x >> y >> v_x >> v_y) {
        Eigen::Vector2d p0{x, y}, v0{v_x, v_y};
        rigidbodies.push_back(sim.addObject(m, r, p0, v0));
    }

    // Render loop
    uint64_t frame = 0;
    while (!glfwWindowShouldClose(window)) {
        // Render
        glClear(GL_COLOR_BUFFER_BIT);

        sim.computeForces();
        sim.step();
        std::cout << "\rstep " << ++frame;

        // Process events and swap buffers
        glfwPollEvents();
        glfwSwapBuffers(window);
    } 

    glfwTerminate();
    return 0;
}
