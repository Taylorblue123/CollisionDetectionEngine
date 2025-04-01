#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <iomanip>
#include "SphereBV.h"
#include "CollisionDetection.h"
#include "Utils.h"

// Function to create spheres with specified parameters
void createSpheres(SphereBV* spheres, int numSpheres, int complexity, float radius, float velocity, float mass, float worldSize) {
    Utils utils;
    for (int i = 0; i < numSpheres; i++) {
        // Random center within cubic worldsize (default world centered at origin)
        float minBoundary = -worldSize + radius;
        float maxBoundary = worldSize - radius;
        glm::vec3 center = glm::vec3(
            utils.randomFloat(minBoundary, maxBoundary),
            utils.randomFloat(minBoundary, maxBoundary),
            utils.randomFloat(minBoundary, maxBoundary)
        );

        // Generate random velocity in the given range
        glm::vec3 vel = glm::vec3(
            utils.randomFloat(-velocity, velocity),
            utils.randomFloat(-velocity, velocity),
            utils.randomFloat(-velocity, velocity)
        );

        // Generate color based on radius, mass, and velocity
        glm::vec3 color;
        color.r = utils.randomFloat(0.0f, 1.0f);
        color.g = utils.randomFloat(0.0f, 1.0f);
        color.b = utils.randomFloat(0.0f, 1.0f);

        // Create the sphere
        spheres[i] = SphereBV(center, radius, vel, mass, complexity, color, i);
    }
}

// Function to measure collision detection performance
void measurePerformance(int numSpheres, int complexity, float radius, float velocity, 
                        float mass, float worldSize, int method, std::ofstream& outputFile) {
    
    // Create spheres with specified parameters
    SphereBV* spheres = new SphereBV[numSpheres];
    createSpheres(spheres, numSpheres, complexity, radius, velocity, mass, worldSize);

    // Vector to store collision pairs
    std::vector<std::pair<SphereBV*, SphereBV*>> collisionPairs;
    
    // Create collision detection object
    CollisionDetection collisionDetection(spheres, numSpheres, worldSize, &collisionPairs, method);
    
    // Timing variables
    auto startBroad = std::chrono::high_resolution_clock::now();
    auto endBroad = std::chrono::high_resolution_clock::now();
    auto startNarrow = std::chrono::high_resolution_clock::now();
    auto endNarrow = std::chrono::high_resolution_clock::now();
    auto startHandle = std::chrono::high_resolution_clock::now();
    auto endHandle = std::chrono::high_resolution_clock::now();

    // Timing for broad phase
    startBroad = std::chrono::high_resolution_clock::now();
    collisionDetection.broadCollisionDetection();
    endBroad = std::chrono::high_resolution_clock::now();

    // Count potential collisions after broad phase
    int potentialCollisions = collisionPairs.size();

    // Timing for narrow phase
    startNarrow = std::chrono::high_resolution_clock::now();
    collisionDetection.narrowCollisionDetection();
    endNarrow = std::chrono::high_resolution_clock::now();

    // Count actual collisions after narrow phase
    int actualCollisions = collisionPairs.size();

    // Timing for collision handling
    startHandle = std::chrono::high_resolution_clock::now();
    collisionDetection.handleCollision();
    endHandle = std::chrono::high_resolution_clock::now();

    // Calculate durations in microseconds
    auto broadDuration = std::chrono::duration_cast<std::chrono::microseconds>(endBroad - startBroad).count();
    auto narrowDuration = std::chrono::duration_cast<std::chrono::microseconds>(endNarrow - startNarrow).count();
    auto handleDuration = std::chrono::duration_cast<std::chrono::microseconds>(endHandle - startHandle).count();
    auto totalDuration = broadDuration + narrowDuration + handleDuration;

    // Convert microseconds to milliseconds for output
    double broadMs = broadDuration / 1000.0;
    double narrowMs = narrowDuration / 1000.0;
    double handleMs = handleDuration / 1000.0;
    double totalMs = totalDuration / 1000.0;

    // Get method name
    std::string methodName;
    if (method == 0) methodName = "Sweep_and_Prune";
    else if (method == 1) methodName = "Brute_Force";
    else methodName = "Grid";

    // Output to file: numSpheres,complexity,radius,velocity,mass,worldSize,method,
    // broadTime(ms),narrowTime(ms),handleTime(ms),totalTime(ms),potentialCollisions,actualCollisions
    outputFile << numSpheres << ","
               << complexity << ","
               << radius << ","
               << velocity << ","
               << mass << ","
               << worldSize << ","
               << methodName << ","
               << std::fixed << std::setprecision(3) << broadMs << ","
               << std::fixed << std::setprecision(3) << narrowMs << ","
               << std::fixed << std::setprecision(3) << handleMs << ","
               << std::fixed << std::setprecision(3) << totalMs << ","
               << potentialCollisions << ","
               << actualCollisions << std::endl;

    // Clean up
    for (int i = 0; i < numSpheres; i++) {
        if (spheres[i].mesh) {
            delete spheres[i].mesh;
            spheres[i].mesh = nullptr;
        }
    }
    delete[] spheres;

    std::cout << "Completed test: " << numSpheres << " spheres, complexity " << complexity 
              << ", radius " << radius << ", method " << methodName 
                << ", velocity " << velocity << ", mass " << mass
              << ", broad Collisions Detection Time " << broadMs << " ms"
              << ", narrow Collisions Detection Time " << narrowMs << " ms"
                << ", handle Collisions Time " << handleMs << " ms"
                
              << ", time " << totalMs << " ms" 
                << ", potential collisions " << potentialCollisions 
                    << ", actual collisions " << actualCollisions << std::endl;

}



// Main function to run experiments
//change main1 to main to run the test
int main1() {
    // Create output file
    std::ofstream outputFile("collision_performance_results.csv");
    
    // Write header
    outputFile << "NumSpheres,Complexity,Radius,Velocity,Mass,WorldSize,Method,"
               << "BroadTime_ms,NarrowTime_ms,HandleTime_ms,TotalTime_ms,"
               << "PotentialCollisions,ActualCollisions" << std::endl;
    
    // Experiment parameters
    float worldSize = 20.0f;
    const float defaultMass = 1.0f;
    const float defaultVelocity = 0.5f;
    const float defaultRadius = 1.0f;
    const int defaultComplexity = 20;
    
    std::cout << "Starting performance analysis..." << std::endl;
    
    std::cout << "\n=== Experiment 1: Varying Number of Objects ===" << std::endl;
    // Experiment 1: Varying number of objects (1-50)
    for (int numSpheres : {10, 30, 50, 100, 200, 500, 1000}) {
        // Test Sweep and Prune method (method = 0)
        measurePerformance(numSpheres, defaultComplexity, defaultRadius, 
                          defaultVelocity, defaultMass, worldSize, 0, outputFile);
    
    }

    for (int numSpheres : {10, 30, 50, 100, 200, 500, 1000}) {
        // Test Brute Force method (method = 1)
        measurePerformance(numSpheres, defaultComplexity, defaultRadius, 
                          defaultVelocity, defaultMass, worldSize, 1, outputFile);
    }
    
    std::cout << "\n=== Experiment 2: Varying Object Complexity ===" << std::endl;
    // Experiment 2: Varying object complexity
    for (int complexity : {20, 50, 100, 200, 300, 400, 800}) {
        // Use 20 objects for these tests
        const int testSpheres = 20;
        
        // Test Sweep and Prune method (method = 0)
        measurePerformance(testSpheres, complexity, defaultRadius, 
                          defaultVelocity, defaultMass, worldSize, 0, outputFile);
        
    }
    for (int complexity : {20, 50, 100, 200, 300, 400, 800}) {
        // Use 20 objects for these tests
        const int testSpheres = 20;
        // Test Brute Force method (method = 1)
        measurePerformance(testSpheres, complexity, defaultRadius, 
                          defaultVelocity, defaultMass, worldSize, 1, outputFile);
    }
    
    std::cout << "\n=== Experiment 3: Varying Object Size ===" << std::endl;
    // Experiment 3: Varying object size (relative to world)
    for (float radius : {0.5f, 2.0f, 5.0f, 7.0f, 10.0f, 15.0f}) {
        // Use 20 objects for these tests
        const int testSpheres = 100;
        
        worldSize = 50.0f; // Reset world size for this test
        
        // Test Sweep and Prune method (method = 0)
        measurePerformance(testSpheres, defaultComplexity, radius, 
                          defaultVelocity, defaultMass, worldSize, 0, outputFile);
    
    }

    for (float radius : {0.5f, 2.0f, 5.0f, 7.0f, 10.0f, 15.0f}) {
        // Use 20 objects for these tests
        const int testSpheres = 100;
        // Test Brute Force method (method = 1)
        worldSize = 50.0f; // Reset world size for this test
        measurePerformance(testSpheres, defaultComplexity, radius, 
                          defaultVelocity, defaultMass, worldSize, 1, outputFile);
    }
    
    std::cout << "\n=== Experiment 4: Varying Velocity ===" << std::endl;
    // Experiment 4: Varying object velocities
    for (float velocity : {0.5f, 1.0f, 2.0f, 5.0f, 10.0f}) {
        worldSize = 20.0f; // Reset world size for this test
        // Use 20 objects for these tests
        const int testSpheres = 20;
        
        // Test Sweep and Prune method (method = 0)
        measurePerformance(testSpheres, defaultComplexity, defaultRadius, 
                          velocity, defaultMass, worldSize, 0, outputFile);
    
    }
    for (float velocity : {0.5f, 1.0f, 2.0f, 5.0f, 10.0f}) {
        worldSize = 20.0f; // Reset world size for this test
        // Use 20 objects for these tests
        const int testSpheres = 20;
        // Test Brute Force method (method = 1)
        measurePerformance(testSpheres, defaultComplexity, defaultRadius, 
                          velocity, defaultMass, worldSize, 1, outputFile);
    }
    // Close the output file
    outputFile.close();
    
    std::cout << "\nPerformance analysis completed. Results saved to collision_performance_results.csv" << std::endl;
    
    return 0;
}