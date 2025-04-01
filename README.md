1. Abstract
The program simulates a 3D system with spheres moving within a cubic world. It has features of collision detection, response handling, and rendering of the simulation.  Coded in C++, OpenGL, and glm libray, we implemented this collision detection simulator from scratch including data structure, algrithem, and rendering pipeline. Also, we provide a nice input UI panel to adjust different parameters including numberOfSpheres, there complexity (achieved by lathing),  velocity, mass, radius, world size and (broad) collision detection methed. User can freely navigate within the whole 3D scene during simulation.

https://youtu.be/ltwDijyG6_Q
2. Implementations
SimulatorWorld
Purpose: Manages the simulation environment, including initialization, stepping through simulation frames, and rendering.
Responsibilities:
Initializes the world with spheres and boundaries.
Updates sphere positions based on velocity and handles collisions.
Provides methods to reset, stop, and render the simulation.
Key Attributes:
spheres: Array of SphereBV objects.
cubicWorldVertices and indices: Used for rendering the world boundary.
Key Methods:
initializeWorld(): Sets up spheres with random properties.
stepSimulation(deltaTime): Advances the simulation by detecting and resolving collisions.
SphereBV
Purpose: Represents a bounding volume for a sphere, including spheres properties and SphereMesh.
Responsibilities:
Stores sphere attributes like position, radius, velocity, and mass.
Key Attributes:
center, radius, velocity, mass: Define the sphere's physical state.
mesh: A SphereMesh object for rendering.
transform: Model Transformation matrix for positioning and scaling.
SphereMesh
Purpose: Generates and stores the mesh data for a sphere using lathing.
Responsibilities:
Creates vertices and indices for rendering a sphere based on its complexity level.
Key Attributes:
vertices: List of vertices defining the sphere's geometry.
indices: Indices for rendering the sphere as a triangular mesh.
CollisionDetection
Purpose: Handles collision detection and response between spheres and world boundary.
Responsibilities:
Implements broad-phase and narrow-phase collision detection.
Resolves collisions using conservation of momentum and energy.
Key Methods:
broadCollisionDetection(): Uses Sweep and Prune or brute force to find potential collisions.
narrowCollisionDetection(): Uses the GJK algorithm to confirm collisions.
handleCollision(): Updates sphere velocities based on collision response.
GJK(): Implements the Gilbert-Johnson-Keerthi algorithm for collision detection.
Utils
Purpose: Provides utility functions for sorting, random number generation, and set operations.
Responsibilities:
Implements insertion sort for sorting endpoints in Sweep and Prune.
Provides a method for three-set intersection to refine collision pairs.
3. Physics and Rendering
Physics:
Sphere velocities are updated based on collisions and boundary interactions.
Collision response uses conservation of momentum and energy equations.
Rendering:
OpenGL Pipeline is used to visualize the simulation result.
The cubic world boundary is represented as a wireframe
4. Experiment of different Collision Detection Design
Broad Phase:
Uses the Sweep and Prune algorithm (method 0) to reduce the number of potential collision pairs by sorting sphere endpoints along each axis.
Supports brute force (method 1) and  grid-based detection (method 2).
Narrow Phase:
Implemented the 3D GJK algorithm to confirm collisions between pairs identified in the broad phase.


In order to measure the performance of our algrithem in different senario, we did experiments based on different number of objects, complexity of objects, and sizes of objects, the velocity. Here are their raw data and analysis:



The experiments above shows the total time for the whole collision detection (broad and narrow) and response process within one frame/timestep. 






For our current implementation, the sweep  and prune method outperforms brute force method in our current experiments, further check for implementation need to be done to optimize its performance of sweep and prune. It may caused by the fact that, in 3-dimensional sweep and prune, we need to check the potential collision in all three axis, and conducting a three set union is computational expensive, which exceeds the computational cost of simple brute force.

Number of Objects Experiment





For both sweep_and_prune and brute force algrithoms, the calculation times increases when number of spheres increases. However, the time cost for sweep and prune, is more expensive than brute_force.



Varying Object Complexity





For both algrithems, they are not sensitive to the change of complexity. maybe because of the fact that the complexity will only influence the narrow detection phase (GJK), and in our implementation, in order to avoid infinite simplex finding caused by entering local optimal point (which happens in real test), we have set the max iteration times of 20. So in general, the narrow detection time also remains stable.



Varing Object Size





With the increaing radius of spheres, the time cost of both algrithems increases. Since larger spheres are more likely to have collisions with others, resulting the time cost of narrow detection increase.



Varying Velocity



In our experiment, the total time cost within one frame remains stable for both algrithems.  It may because of in unit timestep, the displacement of spheres is so small that no new collisions happen.



5. Problem C: When Objects Have Same Size

In this case, we just simply implement brute force algrithem for broad phase collision detection. And the performance comparision shown in the above chart graph. In general, the brute force algrithem in our implementation outperforms the sweep and prune. It may be cause our 3D sweep and prune algrithem itset is expensive (our implementation). 



For future exploration, we believe the unit grid partioning algrithem for collision detection is the optimal choice when objects have same size, and the width of the unit cubic partition should be twice of the radius of spheres.



6. Potential Improvements
Collision Detection: 

Tunneling Issue exists
broad collision detection should be implemented in a more optimal way
Should implement “Hill Climbing” technique for GJK algrithm to avoid stucking at local optimal point.
Collision Response: 

Rigid body feature need to be added (angular velocity)
Gravity or other forces can be added.
Memory Management: Use smart pointers to manage dynamic memory more safely.

