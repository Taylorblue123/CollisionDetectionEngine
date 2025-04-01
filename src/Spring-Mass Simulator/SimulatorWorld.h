#pragma once
#include <glm/glm.hpp>
#include "SphereBV.h"
#include "SphereMesh.h"
#include <vector>
#include "CollisionDetection.h"

class SimulatorWorld
{
public:
    //INitializes the simulator world from UI input
    //complexityLevel: 0 = simple, 1 = medium, 2 = complex
    SimulatorWorld(
        const int minComplexity,
        const int maxComplexity,
        const int numSpheres,
        const float minRadius,
        const float maxRadius,
        const float minVelocity,
        const float maxVelocity,
        const float minMass,
        const float maxMass,  // Added missing comma here
        const float worldSize // The max boundary in positive + axis
    );
    
    // Add destructor declaration
    ~SimulatorWorld();
    
    // Make these members public so they can be accessed from main
    SphereBV* spheres;  // Array of spheres in the simulation
    glm::vec3* CubeWorldPosition;  // Add missing declaration for CubeWorldPosition
    
    // Bounding box of the simulation world
    std::vector<vertice> cubicWorldVertices; 
    std::vector<int> indices;

    int numSpheres;

    void initializeWorld(); // Initialize the simulation world with spheres and their properties
    void stepSimulation(float deltaTime);
    void stopSimulation(); // Stop the simulation and clean up resources
    void resetSimulation(); // Reset the simulation to its initial state
    void render(); // Render the simulation world
    void initializeWorldBoundary(); // Made public to access from main

private:
    int minComplexity;
    int maxComplexity;     
    float minRadius;    
    float maxRadius;    
    float minVelocity;  
    float maxVelocity; 
    float minMass;     
    float maxMass;
    float worldSize;    
};

