#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> // Required for glm::translate and glm::scale
#include "SphereMesh.h"
#include <cmath>
#include <vector>


//This class represents a bounding volume in the shape of a sphere.
//In default, the sphere is centered at the origin, but then it can be transformed to any position in the world using a transformation matrix.
//In default, the sphere has no accerleration, but it can be given a velocity vector to move in the world.
//We now just ignore rigid body physics that containing rotation and angular velocity.

class SphereBV
{
public:
    glm::vec3 center; // Center of the sphere
    float radius;     // Radius of the sphere
    SphereMesh* mesh; // Mesh representation of the sphere
    glm::mat4 transform; // Transformation matrix for the sphere, decide where to place the sphere in the cubic world
    glm::vec3 velocity; // Velocity of the sphere
    float mass;       // Mass of the sphere
    int complexityLevel; // Complexity level of the sphere, use to generate the sphere mesh by lathing and longhitude

    glm::vec3 color;  // Color of the sphere

    int id;

    // Default constructor
    SphereBV() : center(0.0f), radius(0.0f), velocity(0.0f), mass(0.0f), complexityLevel(0), color(0.0f), id(-1), mesh(nullptr) {}
    
    // Constructor
    SphereBV(const glm::vec3& center, float radius, const glm::vec3& velocity, float mass, int complexityLevel, const glm::vec3& color, int id)
        : center(center), radius(radius), velocity(velocity), mass(mass), color(color), id(id), mesh(nullptr) {
        // Ensure minimum complexity of 1 to avoid empty mesh data.
        int effectiveComplexity = (complexityLevel < 8 ? 8 : complexityLevel);
        this->complexityLevel = effectiveComplexity;
        mesh = new SphereMesh(effectiveComplexity, color); // Create a new sphere mesh with the given complexity level
        transform = glm::mat4(1.0f); // Initialize the transformation matrix to identity
        transform = glm::translate(transform, center); // Translate the sphere to its center position
        transform = glm::scale(transform, glm::vec3(radius)); // Scale the sphere to its radius
    }
    
    // Destructor
    ~SphereBV() {
        // We don't delete mesh here as ownership is managed elsewhere
    }

    // Check if a point is inside the sphere
    bool contains(const glm::vec3& point) const {
        return glm::length(point - center) <= radius;
    }

    // Check if another sphere intersects with this one
    bool intersects(const SphereBV& other) const {
        float distance = glm::length(other.center - center);
        return distance <= (radius + other.radius);
    }
};


inline bool operator==(const SphereBV &lhs, const SphereBV &rhs) {
    return lhs.id == rhs.id;
}