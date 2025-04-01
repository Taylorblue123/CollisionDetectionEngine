#pragma once
#include <glm/glm.hpp>
#include "SphereBV.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <unordered_set>
#include "utils.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Represent the endpoint of beginning and end
struct Point {
    float value;
    bool isBeginning;
    int id;

    bool operator<(const Point &other) const {
        return value < other.value || (value == other.value && isBeginning && !other.isBeginning);
    }
}; 

class CollisionDetection
{
public:
    // Constructor
    CollisionDetection(SphereBV* spheres, int numSpheres, float WorldSize, std::vector<std::pair<SphereBV*, SphereBV*>>* collisionPairs, int method = 0)
        : spheres(spheres), numSpheres(numSpheres), worldSize(WorldSize), collisionPairs(collisionPairs), method(method) {
        collisionPairs->clear();
    }

    // Destructor - Modified to not delete spheres, since it doesn't own them
    ~CollisionDetection() {
        // Remove the delete[] spheres line since this class doesn't own the memory
    }

    // Broad Collision Detection
    void broadCollisionDetection(){
        collisionPairs->clear();

        //Check if the spheres are within the world boundary
        for(int i = 0; i < numSpheres; i++){
            if(spheres[i].center.x + spheres[i].radius > worldSize || spheres[i].center.x - spheres[i].radius < -worldSize ||
               spheres[i].center.y + spheres[i].radius > worldSize || spheres[i].center.y - spheres[i].radius < -worldSize ||
               spheres[i].center.z + spheres[i].radius > worldSize || spheres[i].center.z - spheres[i].radius < -worldSize){
                // If the sphere is out of bounds, simply reverse its velocity in certain direction component
                if(spheres[i].center.x + spheres[i].radius > worldSize){
                    spheres[i].velocity.x = -spheres[i].velocity.x;
                    spheres[i].center.x = worldSize - spheres[i].radius;
                } else if(spheres[i].center.x - spheres[i].radius < -worldSize){
                    spheres[i].velocity.x = -spheres[i].velocity.x;
                    spheres[i].center.x = -worldSize + spheres[i].radius;
                }
                if(spheres[i].center.y + spheres[i].radius > worldSize){
                    spheres[i].velocity.y = -spheres[i].velocity.y;
                    spheres[i].center.y = worldSize - spheres[i].radius;
                } else if(spheres[i].center.y - spheres[i].radius < -worldSize){
                    spheres[i].velocity.y = -spheres[i].velocity.y;
                    spheres[i].center.y = -worldSize + spheres[i].radius;
                }
                if(spheres[i].center.z + spheres[i].radius > worldSize){
                    spheres[i].velocity.z = -spheres[i].velocity.z;
                    spheres[i].center.z = worldSize - spheres[i].radius;
                } else if(spheres[i].center.z - spheres[i].radius < -worldSize){
                    spheres[i].velocity.z = -spheres[i].velocity.z;
                    spheres[i].center.z = -worldSize + spheres[i].radius;
                }

            }
        }

        // Sweep and Prune method
        if(method == 0){
            std::vector<Point> PointX;
            std::vector<Point> PointY;
            std::vector<Point> PointZ;

            for (int i = 0; i < numSpheres; i++){
                PointX.push_back({spheres[i].center.x - spheres[i].radius, true, i});
                PointX.push_back({spheres[i].center.x + spheres[i].radius, false, i});

                PointY.push_back({spheres[i].center.y - spheres[i].radius, true, i});
                PointY.push_back({spheres[i].center.y + spheres[i].radius, false, i});

                PointZ.push_back({spheres[i].center.z - spheres[i].radius, true, i});
                PointZ.push_back({spheres[i].center.z + spheres[i].radius, false, i});
            }

            // Create a Utils instance for sorting and intersection
            Utils utils;

            // Sort the points in each dimension
            utils.insertionSort(PointX);
            utils.insertionSort(PointY);
            utils.insertionSort(PointZ);

            std::unordered_set<int> activeSet;
            std::vector<std::pair<SphereBV*, SphereBV*>> potentialCollisionPairsX;
            std::vector<std::pair<SphereBV*, SphereBV*>> potentialCollisionPairsY;
            std::vector<std::pair<SphereBV*, SphereBV*>> potentialCollisionPairsZ;

            // Iterate through the sorted points and find potential collision pairs
            for(auto &point: PointX) {
                if(point.isBeginning) {
                    for(int activeId : activeSet){
                        potentialCollisionPairsX.push_back({&spheres[point.id], &spheres[activeId]});
                    }
                    activeSet.insert(point.id);
                } else {
                    activeSet.erase(point.id);
                }
            }
            activeSet.clear();

            for(auto &point: PointY) {
                if(point.isBeginning) {
                    for(int activeId : activeSet){
                        potentialCollisionPairsY.push_back({&spheres[point.id], &spheres[activeId]});
                    }
                    activeSet.insert(point.id);
                } else {
                    activeSet.erase(point.id);
                }
            }
            activeSet.clear();

            for(auto &point: PointZ) {
                if(point.isBeginning) {
                    for(int activeId : activeSet){
                        potentialCollisionPairsZ.push_back({&spheres[point.id], &spheres[activeId]});
                    }
                    activeSet.insert(point.id);
                } else {
                    activeSet.erase(point.id);
                }
            }
            activeSet.clear();

            // Find the intersection of the three sets of potential collision pairs
            std::vector<std::pair<SphereBV*, SphereBV*>> potentialCollisionPairs;
            potentialCollisionPairs = utils.threeSetIntersectionUnordered(potentialCollisionPairsX, potentialCollisionPairsY, potentialCollisionPairsZ);
            *collisionPairs = potentialCollisionPairs; 

        } else if(method == 1) {
            // Handle by brute force method
            for(int i = 0; i < numSpheres; i++){
                for(int j = i + 1; j < numSpheres; j++){
                    if(spheres[i].intersects(spheres[j])){
                        collisionPairs->push_back({&spheres[i], &spheres[j]});
                    }
                }
            }
        } else if(method == 2){
            // TODO: Handle by grid method
        }
    }

    //Narrow Collision Detection using the standard GJK algorithm
    void narrowCollisionDetection() {
        for(auto &pair : *collisionPairs){
            SphereBV* sphereA = pair.first;
            SphereBV* sphereB = pair.second;

            // Check if the spheres are colliding using GJK algorithm
            if(!GJK(sphereA, sphereB)){
                // If not colliding, remove the pair from the collision pairs
                collisionPairs->erase(std::remove(collisionPairs->begin(), collisionPairs->end(), pair), collisionPairs->end());
            }
        }

    }

    //Simply reverse the velocity between two possible spheres
    void handleCollision() {
        for(auto &pair : *collisionPairs){
            SphereBV* sphereA = pair.first;
            SphereBV* sphereB = pair.second;

            // Simply reverse their velocity direction using conservation of momentum and energy
            glm::vec3 velocityBefore_A = sphereA->velocity;
            glm::vec3 velocityBefore_B = sphereB->velocity;

            float mass_A = sphereA->mass;
            float mass_B = sphereB->mass;

            // updates:
            // v′ = ((m - M) / (m + M)) · v + (2M / (m + M)) · V
            // V′ = (2m / (m + M)) · v + ((M - m) / (m + M)) · V
            glm::vec3 velocityAfter_A = ((mass_A - mass_B) / (mass_A + mass_B)) * velocityBefore_A + (2 * mass_B / (mass_A + mass_B)) * velocityBefore_B;
            glm::vec3 velocityAfter_B = (2 * mass_A / (mass_A + mass_B)) * velocityBefore_A + ((mass_B - mass_A) / (mass_A + mass_B)) * velocityBefore_B;
            
            sphereA->velocity = velocityAfter_A;
            sphereB->velocity = velocityAfter_B;


        }
    }

        // GJK main function
        // Check if two spheres are colliding using the GJK algorithm
        bool GJK(SphereBV* A, SphereBV* B) {
            // 初始搜索方向
            glm::vec3 d = A->center - B->center;
            if(glm::length(d) < 1e-6)
                d = glm::vec3(1.0f, 0.0f, 0.0f);
            
            std::vector<glm::vec3> simplex;
            glm::vec3 supportPoint = support(A, B, d);
            simplex.push_back(supportPoint);
            d = -supportPoint;
            
            // Add maximum iterations to prevent infinite loops
            const int MAX_ITERATIONS = 20;
            int iterations = 0;
            
            while (iterations < MAX_ITERATIONS) {
                iterations++;
                glm::vec3 newPoint = support(A, B, d);
                
                // Check if we've made progress
                if (glm::dot(newPoint, d) < 0)
                    return false;  // 新点未能超过原点，说明无碰撞
                    
                // Check if new point is not significantly different from existing points
                bool duplicate = false;
                for (const auto& p : simplex) {
                    if (glm::length(p - newPoint) < 1e-6) {
                        duplicate = true;
                        break;
                    }
                }
                
                if (duplicate)
                    return false; // No progress being made, exit
                    
                simplex.push_back(newPoint);
                if (handleSimplex(simplex, d))
                    return true;   // 当 simplex 包含原点，则发生碰撞
            }
            // If we reach the maximum iterations, assume no collision for safety
            return false;
        }



private:
    SphereBV* spheres;
    int numSpheres;
    float worldSize;
    std::vector<std::pair<SphereBV*, SphereBV*>>* collisionPairs;
    int method;

    // Support function for GJK algorithm
    glm::vec3 support(SphereBV* A, SphereBV* B, const glm::vec3 &d) {
        // Safety check for zero direction vector
        if (glm::length(d) < 1e-6) {
            return A->center - B->center;
        }
        
        // Normalize the direction to improve numerical stability
        glm::vec3 dir = glm::normalize(d);
        
        // For spheres, we can optimize by using the analytical solution
        // instead of checking all mesh vertices
        glm::vec3 furthest_A = A->center + A->radius * dir;
        glm::vec3 furthest_B = B->center - B->radius * dir;
        
        return furthest_A - furthest_B;
    }

    // 三重叉运算：计算 (a × b) × c
    glm::vec3 tripleCross(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c) {
        return glm::cross(glm::cross(a, b), c);
    }

    // Simplex 处理：针对线段（2点）情况
    bool handleLine(std::vector<glm::vec3>& simplex, glm::vec3 &d) {
        glm::vec3 A = simplex.back();      // 最新点
        glm::vec3 B = simplex.front();       // 最早的点
        glm::vec3 AB = B - A;
        glm::vec3 AO = -A;                   // 从 A 指向原点
        d = tripleCross(AB, AO, AB);
        if(glm::length(d) < 1e-6) {
            d = glm::vec3(-AB.y, AB.x, 0.0f);
        }
        return false;
    }

    // Simplex 处理：针对三角形（3点）情况
    bool handleTriangle(std::vector<glm::vec3>& simplex, glm::vec3 &d) {
        // 假定 simplex 中点的顺序为 C, B, A，其中 A 为最新添加
        glm::vec3 A = simplex[2];
        glm::vec3 B = simplex[1];
        glm::vec3 C = simplex[0];
        glm::vec3 AO = -A;
        glm::vec3 AB = B - A;
        glm::vec3 AC = C - A;
        glm::vec3 ABC = glm::cross(AB, AC);
        
        // 判断原点是否在 AB 边的区域
        glm::vec3 ABPerp = glm::cross(AB, ABC);
        if (glm::dot(ABPerp, AO) > 0) {
            simplex.erase(simplex.begin()); // 移除 C
            d = tripleCross(AB, AO, AB);
            return false;
        }
        // 判断原点是否在 AC 边的区域
        glm::vec3 ACPerp = glm::cross(ABC, AC);
        if (glm::dot(ACPerp, AO) > 0) {
            simplex.erase(simplex.begin() + 1); // 移除 B
            d = tripleCross(AC, AO, AC);
            return false;
        }
        // 原点位于三角形面上方
        if (glm::dot(ABC, AO) > 0)
            d = ABC;
        else {
            std::swap(simplex[0], simplex[1]);
            d = -ABC;
        }
        return false;
    }

    // Simplex 处理：针对四面体（4点）情况
    bool handleTetrahedron(std::vector<glm::vec3>& simplex, glm::vec3 &d) {
        // 假定 simplex 点的顺序为 D, C, B, A，其中 A 为最新
        glm::vec3 A = simplex[3];
        glm::vec3 B = simplex[2];
        glm::vec3 C = simplex[1];
        glm::vec3 D = simplex[0];
        glm::vec3 AO = -A;
        glm::vec3 ABC = glm::cross(B - A, C - A);
        glm::vec3 ACD = glm::cross(C - A, D - A);
        glm::vec3 ADB = glm::cross(D - A, B - A);
        if(glm::dot(ABC, AO) > 0) {
            simplex.erase(simplex.begin()); // 移除 D
            d = ABC;
            return false;
        }
        if(glm::dot(ACD, AO) > 0) {
            simplex.erase(simplex.begin() + 2); // 移除 B
            d = ACD;
            return false;
        }
        if(glm::dot(ADB, AO) > 0) {
            simplex.erase(simplex.begin() + 1); // 移除 C
            d = ADB;
            return false;
        }
        // 原点位于四面体内部
        return true;
    }

    // 根据 simplex 当前点数量选择对应的处理函数
    bool handleSimplex(std::vector<glm::vec3>& simplex, glm::vec3 &d) {
        if (simplex.size() == 2)
            return handleLine(simplex, d);
        else if (simplex.size() == 3)
            return handleTriangle(simplex, d);
        else if (simplex.size() == 4)
            return handleTetrahedron(simplex, d);
        return false;
    }

};