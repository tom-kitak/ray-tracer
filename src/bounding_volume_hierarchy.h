#pragma once
#include "common.h"
#include <array>
#include <framework/ray.h>
#include <vector>

// Forward declaration.
struct Scene;

struct Node {
    bool typeLeaf; // 1 = leaf, 0 = interior
    glm::vec3 lowerBound;
    glm::vec3 upperBound;
    std::vector<Tuple> indices; // if leaf -> index mesh, index triangle, if parent -> index left child, index right child
};

//A node can be of two types : interior or leaf node.
//Use a boolean to identify the Node type. Every node contains its bounds 
//and its own std::vector that is used to store the indices of the two child nodes in the node vector(in case of a parent node) 
//or store the indices of the contained triangles(in case of a leaf node)

class BoundingVolumeHierarchy {
public:
    // Constructor. Receives the scene and builds the bounding volume hierarchy.
    BoundingVolumeHierarchy(Scene* pScene);

    // Return how many levels there are in the tree that you have constructed.
    [[nodiscard]] int numLevels() const;

    // Return how many leaf nodes there are in the tree that you have constructed.
    [[nodiscard]] int numLeaves() const;

    // Visual Debug 1: Draw the bounding boxes of the nodes at the selected level.
    void debugDrawLevel(int level);

    // Visual Debug 2: Draw the triangles of the i-th leaf
    void debugDrawLeaf(int leafIdx);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const;

    // Creates tuples with the mesh index and triangle index, for all the triangles
    std::vector<Tuple> createTuples();

    // Finds the upper- and lowerbound for the triangles in a node
    std::tuple<glm::vec3, glm::vec3> findBounds(std::vector<Tuple> tuples);

    // Splits one node into two new nodes, using the median that alters between x-y-z
    std::tuple<Node, Node> split(Node node, char axis);

    // Recursively splits the nodes until the stop condition
    int recursiveSplit(int nodeIndex, int currentLevel, int maxLevel);

    // Recursive function for visual debug 1, checks for what level to draw
    void recursionDrawLevel(int currentLevel, int level, int nodeIndex); 

private:
    int m_numLevels;
    int m_numLeaves;
    Scene* m_pScene;
    std::vector<Node> m_nodes;
};