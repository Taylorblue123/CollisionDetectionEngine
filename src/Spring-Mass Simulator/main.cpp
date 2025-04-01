#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/glm.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include "SimulatorWorld.h"
#include "SphereMesh.h"
#include "Utils.h"
#include "CollisionDetection.h"
#include "SphereBV.h"

// Global variables
SimulatorWorld* worldSimulator = nullptr;
GLuint shaderProgram; // Add missing declaration
float deltaTime = 0.016f; // Add missing declaration for timing
GLFWwindow* window; // Make window global

int minComplexity = 8;      // Initialize with default value
int maxComplexity = 50;     // Initialize with default value
int numSpheres = 15;        
float minRadius = 0.2f;     
float maxRadius = 3.0f;     
float minVelocity = -5.0f; 
float maxVelocity = 5.0f;  
float minMass = 0.5f;       
float maxMass = 10.0f;      
float worldSize = 20.0f;    
float step = 0.016f;       

// Camera towards the world center origin
glm::vec3 cameraPos(0.0f, 1.0f, 70.0f);
glm::vec3 cameraFront(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);
float yaw = -90.0f, pitch = 0.0f;
float lastX = 400, lastY = 300;
bool firstMouse = true;
float fov = 60.0f;
bool cursorEnabled = false;

void init_shaders(){

    // Vertex shader source code
    const char* vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;
        out vec3 ourColor;
        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        void main() {
            gl_Position = projection * view * model * vec4(aPos, 1.0);
            ourColor = aColor;
        }
    )";

    // Fragment shader source code
    const char* fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        in vec3 ourColor;
        void main() {
            FragColor = vec4(ourColor, 1.0f);
        }
    )";

    // Compile and link shaders (omitted for brevity)
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glUseProgram(shaderProgram);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

// ---------------------
// Mouse Callback
// ---------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    // if cursorEnabled, then no control for camera view
    if (cursorEnabled)
        return;

    if (firstMouse) {
        lastX = (float)xpos;
        lastY = (float)ypos;
        firstMouse = false;
    }
    float xoffset = (float)xpos - lastX;
    float yoffset = lastY - (float)ypos; // y inverse
    lastX = (float)xpos;
    lastY = (float)ypos;
    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;
    yaw += xoffset;
    pitch += yoffset;
    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;
    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}

// ---------------------
// Keyboard Callbacks
// ---------------------
void processInput(GLFWwindow* window) {
    float speed = 10.0f * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += speed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= speed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * speed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * speed;

    //  ESC Keyboard
    static bool EscKeyPressed = false;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        if (!EscKeyPressed) {
            cursorEnabled = !cursorEnabled;
            if (cursorEnabled) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
            else {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                firstMouse = true; 
            }
            EscKeyPressed = true;
        }
    }
    else {
        EscKeyPressed = false;
    }
}

//Render world boundary (wireframe cube)
void renderWorld(){
    if (!worldSimulator) return;

    GLuint VBO, VAO, EBO; // Fix type from Guint to GLuint
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, worldSimulator->cubicWorldVertices.size() * sizeof(vertice), &worldSimulator->cubicWorldVertices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, worldSimulator->indices.size() * sizeof(int), &worldSimulator->indices[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertice), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertice), (void*)offsetof(vertice, color));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
    glBindVertexArray(0); // Unbind VAO

    // Set the shader program and uniforms
    glUseProgram(shaderProgram);
    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
    glm::mat4 projection = glm::perspective(glm::radians(fov), (float)800 / (float)600, 0.1f, 100.0f);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    // Render the world boundary
    glBindVertexArray(VAO); //Use previous VAO configeruation
    glLineWidth(2.0f); 
    glDrawElements(GL_LINES, worldSimulator->indices.size(), GL_UNSIGNED_INT, 0); // Draw the cube as wireframe
    glBindVertexArray(0); // Unbind VAO
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glUseProgram(0); // Unbind shader program
    glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); // Unbind EBO
    glBindVertexArray(0); // Unbind VAO
    
}

// Render the spheres with their mesh using OpenGL non-traditional pipeline (triangle)
void renderSpheres() {
    if (!worldSimulator) return;
    
    for (int i = 0; i < worldSimulator->numSpheres; i++) {
        // Check if the mesh has vertices and indices before rendering
        const auto& verts = worldSimulator->spheres[i].mesh->getVertices();
        const auto& inds = worldSimulator->spheres[i].mesh->getIndices();
        if(verts.empty() || inds.empty())
            continue; // Skip rendering if mesh data is missing

        //Unique VAO, VBO, EBO for each sphere
        GLuint VBO, VAO, EBO;
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(vertice), &verts[0], GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, inds.size() * sizeof(int), &inds[0], GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertice), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertice), (void*)offsetof(vertice, color));
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
        glBindVertexArray(0); // Unbind VAO

        // Set the shader program and uniforms
        glUseProgram(shaderProgram);
        glm::mat4 model = worldSimulator->spheres[i].transform; // Use the sphere's transform matrix
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        glm::mat4 projection = glm::perspective(glm::radians(fov), (float)800 / (float)600, 0.1f, 100.0f);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        
        // First render the solid sphere
        glBindVertexArray(VAO);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Ensure we're in fill mode for solid rendering
        glDrawElements(GL_TRIANGLES, inds.size(), GL_UNSIGNED_INT, 0);
        
        // Now render the wireframe overlay
        // Enable polygon offset to prevent z-fighting between solid and wireframe
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        
        // Set wireframe mode and render
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.0f); // Set the line width for the wireframe
        
        // Optional: Use a different color for wireframe by setting a uniform
        // You'll need to add this uniform to your shader if you want to use it
        // glUniform3f(glGetUniformLocation(shaderProgram, "wireframeColor"), 0.0f, 0.0f, 0.0f); // Black wireframe
        
        // Draw the wireframe
        glDrawElements(GL_TRIANGLES, inds.size(), GL_UNSIGNED_INT, 0);
        
        // Reset polygon mode and disable polygon offset
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDisable(GL_POLYGON_OFFSET_FILL);
        
        glBindVertexArray(0); // Unbind VAO
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glUseProgram(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
}

void drawImGuiControls(int& numSpheres, float& minRadius, float& maxRadius, float& minVelocity, float& maxVelocity, float& minMass, float& maxMass, float& worldSize, int minComplexity, int maxComplexity) {
    // ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    
    ImGui::Begin("Simulator Settings");
    ImGui::SliderInt("Number of Spheres", &numSpheres, 1, 500);               // 范围1~50
    ImGui::SliderInt("Min Complexity", &minComplexity, 1, 100);                 // 范围1~50
    ImGui::SliderInt("Max Complexity", &maxComplexity, 1, 100);                 // 范围1~50
    ImGui::SliderFloat("Min Radius", &minRadius, 0.1f, 5.0f);                  // 范围0.1~5.0
    ImGui::SliderFloat("Max Radius", &maxRadius, 1.0f, 10.0f);                  // 范围0.1~5.0
    ImGui::SliderFloat("Min Velocity", &minVelocity, -10.0f, 10.0f);             // 范围-0.1~0.0
    ImGui::SliderFloat("Max Velocity", &maxVelocity, -10.0f, 10.0f);              // 范围0.0~0.1
    ImGui::SliderFloat("Min Mass", &minMass, 0.1f, 10.0f);                      // 范围0.1~5.0
    ImGui::SliderFloat("Max Mass", &maxMass, 0.1f, 10.0f);                      // 范围0.1~5.0
    ImGui::SliderFloat("World Size", &worldSize, 5.0f, 50.0f);                 // 范围5.0~50.0
    ImGui::Text("Simulation Method: ");
    const char* methods[] = { "Sweep and Prune", "Brute Force", "Grid" };
    static int method = 0;
    ImGui::Combo("Method", &method, methods, IM_ARRAYSIZE(methods));
    ImGui::Text("Simulation Step: ");
    ImGui::SliderFloat("Step Time", &step, 0.001f, 0.05f);                    // 范围0.001~0.05
    ImGui::Text("Simulation Control: ");
    if (ImGui::Button("Start Simulation")) {
        // Initialize the simulator world with new parameters
        delete worldSimulator; // Clean up the previous simulator
        worldSimulator = new SimulatorWorld(minComplexity, maxComplexity, numSpheres,
            minRadius, maxRadius, minVelocity, maxVelocity, minMass, maxMass, worldSize);           
        worldSimulator->initializeWorld(); // Reinitialize the world with new spheres
        worldSimulator->stepSimulation(step);
    }
    if (ImGui::Button("Stop Simulation")) {
        // Stop the simulation and clean up resources
        if (worldSimulator) {
            worldSimulator->stopSimulation();
            delete worldSimulator;
            worldSimulator = nullptr;
        }
    }
    if (ImGui::Button("Reset Simulation")) {
        worldSimulator->resetSimulation();
    }
    if (ImGui::Button("Exit")) {
        glfwSetWindowShouldClose(window, true);
    }
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Camera Position: (%.1f, %.1f, %.1f)", cameraPos.x, cameraPos.y, cameraPos.z);
    //Use Wasd keys to control camera view
    ImGui::Text("Use WASD to control camera view.");
    ImGui::Text("Press ESC to toggle cursor mode.");
    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}


int main() {
    // Initialize GLFW
    glfwInit();
    window = glfwCreateWindow(800, 600, "World System", NULL, NULL); // Assign to global window
    glfwMakeContextCurrent(window);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide cursor
    glfwSetCursorPosCallback(window, mouse_callback); // Set mouse callback

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);


     // Initialize GLAD
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

     // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Initialize Spring System from global variable input
    worldSimulator = new SimulatorWorld(minComplexity, maxComplexity, numSpheres,
        minRadius, maxRadius, minVelocity, maxVelocity, minMass, maxMass, worldSize);

    //Init shaders
    init_shaders();

    float lastFrameTime = 0.0f;

    while (!glfwWindowShouldClose(window)) {
        float currentFrameTime = (float)glfwGetTime();
        deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST); // Enable depth testing for 3D rendering

        // glEnable(GL_CULL_FACE); // Enable backface culling
        // glCullFace(GL_BACK); 
        // glFrontFace(GL_CCW); // Set counter-clockwise as the default winding order

        //UI controls
        drawImGuiControls(numSpheres, minRadius, maxRadius, minVelocity, maxVelocity, minMass, maxMass, worldSize, minComplexity, maxComplexity);

        // Process input
        processInput(window);
        // Draw ImGui controls
        // Render the world boundary
        renderWorld();
        // Render the spheres
        renderSpheres();

        // Step the simulation
        worldSimulator->stepSimulation(step);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // clean up
    if (worldSimulator) {
        delete worldSimulator;
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;
}