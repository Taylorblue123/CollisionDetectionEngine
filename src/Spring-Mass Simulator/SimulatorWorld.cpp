#include "SimulatorWorld.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <ctime>   // For time()
#include <algorithm> // For std::min and std::max
#include <iostream> // For std::cout and std::endl
#include <stdexcept> // For std::runtime_error
#include <cmath> // For std::sqrt and std::pow
#include <limits> // For std::numeric_limits
#include "Utils.h"
#include "CollisionDetection.h"

SimulatorWorld::SimulatorWorld(
    const int minComplexity,
    const int maxComplexity,
    const int numSpheres,
    const float minRadius,
    const float maxRadius,
    const float minVelocity,
    const float maxVelocity,
    const float minMass,
    const float maxMass,
    const float worldSize
) : minComplexity(minComplexity),
maxComplexity(maxComplexity),
numSpheres(numSpheres),
minRadius(minRadius),
maxRadius(maxRadius),
minVelocity(minVelocity),
maxVelocity(maxVelocity),
minMass(minMass),
maxMass(maxMass),
worldSize(worldSize) {

    // Initialize random seed
    srand(static_cast<unsigned int>(time(0)));

    // Allocate memory for spheres
    spheres = new SphereBV[numSpheres]; // Allocate memory for the spheres array
    CubeWorldPosition = nullptr; // Initialize CubeWorldPosition to nullptr

    // Initialize the simulation world
    initializeWorld();
}

SimulatorWorld::~SimulatorWorld() {
    delete[] spheres;
    delete[] CubeWorldPosition;
}

void SimulatorWorld::initializeWorld() {
    // Initialize the world boundary
    initializeWorldBoundary();

    // Create a Utils instance for random number generation
    Utils utils;

    //Inmitialize the simulation world with spheres
    for (int i = 0; i < numSpheres; i++) {
        // Random center within cubic worldsize (default world centered at origin)
        float minBoundary = -worldSize;
        float maxBoundary = worldSize;
        glm::vec3 center = glm::vec3(
            utils.randomFloat(minBoundary, maxBoundary),
            utils.randomFloat(minBoundary, maxBoundary),
            utils.randomFloat(minBoundary, maxBoundary)
        );

        // Generate random radius
        float radius = utils.randomFloat(minRadius, maxRadius);

        // Generate random velocity in the given range
        glm::vec3 velocity = glm::vec3(
            utils.randomFloat(minVelocity, maxVelocity),
            utils.randomFloat(minVelocity, maxVelocity),
            utils.randomFloat(minVelocity, maxVelocity)
        );

        // Generate random mass
        float mass = utils.randomFloat(minMass, maxMass);

        // Generate random complexity for lathing
        int complexity = utils.randomInt(minComplexity, maxComplexity);

        // Generate color based on radius, mass, and velocity
        glm::vec3 color;
        float red = std::min(1.0f, radius / maxRadius);        // Higher radius, more red
        float blue = std::min(1.0f, mass / maxMass);             // Higher mass, more blue
        float velocityMagnitude = glm::length(velocity);
        float yellow = std::min(1.0f, velocityMagnitude / maxVelocity); // Higher velocity, more yellow
        color.r = red;
        color.g = yellow;
        color.b = blue;

        // Create the sphere
        spheres[i] = SphereBV(center, radius, velocity, mass, complexity, color, i);
    }
}

void SimulatorWorld::initializeWorldBoundary() {

    // Initialize the world boundary as a cube with vertices at the corners of the cube
    cubicWorldVertices.clear();
    CubeWorldPosition = new glm::vec3[8]; // Allocate memory for the cube vertices
    CubeWorldPosition[0] = glm::vec3(-worldSize, -worldSize, -worldSize); // Bottom-left-back
    CubeWorldPosition[1] = glm::vec3(worldSize, -worldSize, -worldSize); // Bottom-right-back
    CubeWorldPosition[2] = glm::vec3(worldSize, worldSize, -worldSize); // Top-right-back
    CubeWorldPosition[3] = glm::vec3(-worldSize, worldSize, -worldSize); // Top-left-back
    CubeWorldPosition[4] = glm::vec3(-worldSize, -worldSize, worldSize); // Bottom-left-front
    CubeWorldPosition[5] = glm::vec3(worldSize, -worldSize, worldSize); // Bottom-right-front
    CubeWorldPosition[6] = glm::vec3(worldSize, worldSize, worldSize); // Top-right-front
    CubeWorldPosition[7] = glm::vec3(-worldSize, worldSize, worldSize); // Top-left-front
    
    //Unify the cube vertices to the same color
    glm::vec3 color(1.0f, 1.0f, 1.0f); // White color
    for (int i = 0; i < 8; i++) {
        vertice v;
        v.pos = CubeWorldPosition[i];
        v.color = color;
        cubicWorldVertices.push_back(v); // Add vertex to the mesh
    }

    // Use explicit edge indices for a wireframe cube
    indices.clear();
    indices = {
        0,1, 1,2, 2,3, 3,0,   // back face
        4,5, 5,6, 6,7, 7,4,   // front face
        0,4, 1,5, 2,6, 3,7    // connecting edges
    };
}

void SimulatorWorld::stepSimulation(float deltaTime) {
    //Collision detection and response
    //Queue structure to store pairs of spheres that are colliding
    std::vector<std::pair<SphereBV*, SphereBV*>> collisionPairs;

    // Check for collisions between spheres and handle them
    CollisionDetection collisionDetection(spheres, numSpheres, worldSize, &collisionPairs);
    collisionDetection.broadCollisionDetection(); // Perform broad phase collision detection
    collisionDetection.narrowCollisionDetection(); // Perform narrow phase collision detection
    collisionDetection.handleCollision(); // Handle collisions by reversing velocities

    // Update the position of each sphere based on its velocity and delta time
    for (int i = 0; i < numSpheres; i++) {
        spheres[i].center += spheres[i].velocity * deltaTime;
        // Update transformation: include both translation and scaling
        spheres[i].transform = glm::translate(glm::mat4(1.0f), spheres[i].center) *
                                glm::scale(glm::mat4(1.0f), glm::vec3(spheres[i].radius));
    }
}

void SimulatorWorld::stopSimulation() {
    // Stop the simulation and clean up resources
    delete[] spheres; // Free the memory allocated for spheres
    spheres = nullptr; // Set pointer to nullptr to avoid dangling pointer
    delete[] CubeWorldPosition; // Free the memory allocated for CubeWorldPosition
    CubeWorldPosition = nullptr; // Set pointer to nullptr to avoid dangling pointer
    cubicWorldVertices.clear(); // Clear the vertices of the cubic world
}

// Reset the simulation to its initial state
void SimulatorWorld::resetSimulation() {
    // Clean up existing spheres to prevent memory leaks
    for (int i = 0; i < numSpheres; i++) {
        if (spheres[i].mesh) {
            delete spheres[i].mesh;
            spheres[i].mesh = nullptr;
        }
    }
    
    // Reset the simulation by reinitializing the world
    initializeWorld(); // Reinitialize the world with new spheres
}

//Render the simulation world in the current time step
void SimulatorWorld::render() {
    // Render the simulation world and spheres
    // Simple debug output
    for (int i = 0; i < numSpheres; i++) {
        std::cout << "Sphere " << i << ": Position = (" << spheres[i].center.x << ", " << spheres[i].center.y << ", " << spheres[i].center.z << ")" << std::endl;
    }
}


