#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include "interpolate.h"
#include <glm/glm.hpp>
#include "common.h"


BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    // make root node
    Node root;
    root.typeLeaf = 1;
    std::vector<Tuple> indices = createTuples();
    root.indices = indices;
    std::tuple<glm::vec3, glm::vec3> bounds = findBounds(indices);
    root.lowerBound = std::get<0>(bounds);
    root.upperBound = std::get<1>(bounds);

    m_nodes.push_back(root);

    m_numLevels = recursiveSplit(0, 0, 20);
    m_numLeaves = 0;

    for (Node n : m_nodes) {
        if (n.typeLeaf == 1) {
            m_numLeaves = m_numLeaves + 1;
        }
    } 
}

std::vector<Tuple> BoundingVolumeHierarchy::createTuples()
{
    // create all of the tuples, for each triangle
    std::vector<Tuple> indices;

    int indexMesh = -1;
    for (const auto& mesh : m_pScene->meshes) {
        indexMesh++;
        int indexTriangle = -1;
        for (const auto& tri : mesh.triangles) {
            indexTriangle++;
            indices.push_back(Tuple (indexMesh, indexTriangle));
        }
    }

    return indices;
}

std::tuple<glm::vec3, glm::vec3> BoundingVolumeHierarchy::findBounds(std::vector<Tuple> tuples)
{
    // go through all tuples, min is lowerbound, max is upperbound
    glm::vec3 max = { -INT_MAX, -INT_MAX, -INT_MAX };
    glm::vec3 min = { INT_MAX, INT_MAX, INT_MAX };

    for (Tuple i : tuples) {
        glm::vec3 currentTriangle = m_pScene->meshes[i.first].triangles[i.second];
        for (int p = 0; p < 3; p++) {
            glm::vec3 currentPosition = m_pScene->meshes[i.first].vertices[currentTriangle[p]].position;

            if (currentPosition.x < min.x) {
                min.x = currentPosition.x;
            }
            if (currentPosition.y < min.y) {
                min.y = currentPosition.y;
            }
            if (currentPosition.z < min.z) {
                min.z = currentPosition.z;
            }
            if (currentPosition.x > max.x) {
                max.x = currentPosition.x;
            }
            if (currentPosition.y > max.y) {
                max.y = currentPosition.y;
            }
            if (currentPosition.z > max.z) {
                max.z = currentPosition.z;
            }
        }
    }

    std::tuple<glm::vec3, glm::vec3> result = { min, max };
    return result;
}

float computeC(Vertex v1, Vertex v2, Vertex v3, char axis)
{
    float c;
    if (axis == 'x') {
        c = (v1.position.x + v2.position.x + v3.position.x) / 3;
    } else if (axis == 'y') {
        c = (v1.position.y + v2.position.y + v3.position.y) / 3;
    } else { // axis == 'z'
        c = (v1.position.z + v2.position.z + v3.position.z) / 3;
    }
    return c;
}

std::tuple<Node, Node> BoundingVolumeHierarchy::split(Node node, char axis)
{
    // finds median, then splits one node into two new ones with that median    

    // find the centroids of all the triangle
    std::vector<Tuple> triangles = node.indices;
    std::vector<float> centroids;
    std::vector<std::tuple<Tuple, float>> matchTriangleToC;

    for (int i = 0; i < node.indices.size(); i++) {
        // find vertices for the triangle we're currently looking at
        int indexMesh = node.indices[i].first;
        glm::vec3 currentTriangle = m_pScene->meshes[indexMesh].triangles[node.indices[i].second];
        Vertex v1 = m_pScene->meshes[indexMesh].vertices[currentTriangle.x];
        Vertex v2 = m_pScene->meshes[indexMesh].vertices[currentTriangle.y];
        Vertex v3 = m_pScene->meshes[indexMesh].vertices[currentTriangle.z];

        // compute the centroid value for each triangle
        // only compute the c value for the axis we're interested in
        float c = computeC(v1, v2, v3, axis);

        std::tuple<Tuple, float> tuple = { node.indices[i], c };
        matchTriangleToC.push_back(tuple);

        // add c of each triangle to list
        centroids.push_back(c);
    }
    // sort list of centroids
    sort(centroids.begin(), centroids.end());

    // find median
    float median;
    if (centroids.size() % 2 == 0) { // even values
        median = centroids[(centroids.size() / 2) - 1];
    } else { // odd values
        median = centroids[(centroids.size() - 1) / 2 ];
    }

    // create vector of triangles less than median, and more than median
    std::vector<Tuple> lessThanMedian;
    std::vector<Tuple> moreThanMedian;
    for (std::tuple<Tuple, float> tuple : matchTriangleToC) {
        float triangleCentroid = std::get<1>(tuple);
        if (triangleCentroid < median) {
            lessThanMedian.push_back(std::get<0>(tuple));
        } else {
            moreThanMedian.push_back(std::get<0>(tuple));
        }
    }

    // create new nodes 
    Node nodeLess;
    nodeLess.typeLeaf = 1;
    std::tuple<glm::vec3, glm::vec3> bounds = findBounds(lessThanMedian);
    nodeLess.lowerBound = std::get<0>(bounds);
    nodeLess.upperBound = std::get<1>(bounds);
    nodeLess.indices = lessThanMedian;

    Node nodeMore;
    nodeMore.typeLeaf = 1;
    bounds = findBounds(moreThanMedian);
    nodeMore.lowerBound = std::get<0>(bounds);
    nodeMore.upperBound = std::get<1>(bounds);
    nodeMore.indices = moreThanMedian;

    std::tuple<Node, Node> result = { nodeLess, nodeMore };
    return result;
}

int BoundingVolumeHierarchy::recursiveSplit(int nodeIndex, int currentLevel, int maxLevel)
{
    Node parent = m_nodes[nodeIndex];

    // calls split recursively, and makes sure the axes alternate
    if (m_nodes[nodeIndex].indices.size() <= 1 || currentLevel == maxLevel) {
        // stop condition: one triangle in the node or we're at max, no need to split further
        return currentLevel;
    } else {
        // did not reach stop condition -> split current node 

        if (currentLevel % 3 == 0) {
            // split on the x axis
            std::tuple<Node, Node> children = split(parent, 'x');

            // add children to nodes vector 
            m_nodes.push_back(std::get<0>(children));
            int childIndexLeft = m_nodes.size() - 1;
            
                        int x = recursiveSplit(childIndexLeft, currentLevel + 1, maxLevel);

            m_nodes.push_back(std::get<1>(children));
            int childIndexRight= m_nodes.size() - 1;

            // update parent
            parent.typeLeaf = 0;
            Tuple indicesChildrenTuple = { childIndexLeft, childIndexRight };
            std::vector<Tuple> indicesChildrenVector;
            indicesChildrenVector.push_back(indicesChildrenTuple);
            parent.indices = indicesChildrenVector;
            m_nodes[nodeIndex] = parent;

            // recurse on both children
            // int x = recursiveSplit(childIndexLeft, currentLevel + 1, maxLevel);
            int y = recursiveSplit(childIndexRight, currentLevel + 1, maxLevel);
            return std::max(x, y);
        }
        else if (currentLevel % 3 == 1) {
            // split on the y axis
            std::tuple<Node, Node> children = split(m_nodes[nodeIndex], 'y');

            // add children to nodes vector 
            m_nodes.push_back(std::get<0>(children));
            int childIndexLeft = m_nodes.size() - 1;

                        int x = recursiveSplit(childIndexLeft, currentLevel + 1, maxLevel);

            m_nodes.push_back(std::get<1>(children));
            int childIndexRight = m_nodes.size() - 1;

            // update parent
            parent.typeLeaf = 0;
            Tuple indicesChildrenTuple = { childIndexLeft, childIndexRight };
            std::vector<Tuple> indicesChildrenVector;
            indicesChildrenVector.push_back(indicesChildrenTuple);
            parent.indices = indicesChildrenVector;
            m_nodes[nodeIndex] = parent;

            // recurse on both children
            // int x = recursiveSplit(childIndexLeft, currentLevel + 1, maxLevel);
            int y = recursiveSplit(childIndexRight, currentLevel + 1, maxLevel);
            return std::max(x, y);
        } else { // currentLevel % 3 == 2
            // split on the z axis
            std::tuple<Node, Node> children = split(m_nodes[nodeIndex], 'z');

            // add children to nodes vector 
            m_nodes.push_back(std::get<0>(children));
            int childIndexLeft = m_nodes.size() - 1;

                        int x = recursiveSplit(childIndexLeft, currentLevel + 1, maxLevel);

            m_nodes.push_back(std::get<1>(children));
            int childIndexRight = m_nodes.size() - 1;

            // update parent
            parent.typeLeaf = 0;
            Tuple indicesChildrenTuple = { childIndexLeft, childIndexRight };
            std::vector<Tuple> indicesChildrenVector;
            indicesChildrenVector.push_back(indicesChildrenTuple);
            parent.indices = indicesChildrenVector;
            m_nodes[nodeIndex] = parent;

            // recurse on both children
            // int x = recursiveSplit(childIndexLeft, currentLevel + 1, maxLevel);
            int y = recursiveSplit(childIndexRight, currentLevel + 1, maxLevel);
            return std::max(x, y);
        }
    }
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 1.
int BoundingVolumeHierarchy::numLevels() const
{
    return m_numLevels;
}

// Return the number of leaf nodes in the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 2.
int BoundingVolumeHierarchy::numLeaves() const
{
    return m_numLeaves;
}

// Use this function to visualize your BVH. This is useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDrawLevel(int level)
{
   recursionDrawLevel(0, level, 0);
}

void BoundingVolumeHierarchy::recursionDrawLevel(int currentLevel, int level, int nodeIndex)
{
    if (currentLevel > level) {
        return;
    } else if (currentLevel == level) {
        // draw current node   
        AxisAlignedBox aabb { m_nodes[nodeIndex].lowerBound, m_nodes[nodeIndex].upperBound };
        drawAABB(aabb, DrawMode::Wireframe);
    } else {
        if (m_nodes[nodeIndex].typeLeaf == 0) {
            // does the current node have children? recurse on the children
            recursionDrawLevel(currentLevel + 1, level, m_nodes[nodeIndex].indices[0].first);
            recursionDrawLevel(currentLevel + 1, level, m_nodes[nodeIndex].indices[0].second);
        } else {
            return;
        }
    }
}


// Use this function to visualize your leaf nodes. This is useful for debugging. The function
// receives the leaf node to be draw (think of the ith leaf node). Draw the AABB of the leaf node and all contained triangles.
// You can draw the triangles with different colors. NoteL leafIdx is not the index in the node vector, it is the
// i-th leaf node in the vector.
void BoundingVolumeHierarchy::debugDrawLeaf(int leafIdx)
{
    std::vector<Node> leaves;
    for (Node n : m_nodes) {
        if (n.typeLeaf == 1) {
            leaves.push_back(n);
        }
    }

    if (leafIdx >= leaves.size()) {
        return;
    }

    AxisAlignedBox aabb { leaves[leafIdx].lowerBound, leaves[leafIdx].upperBound };
    drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
}


// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h.
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const
{
    // If BVH is not enabled, use the naive implementation.
    if (!features.enableAccelStructure) {
        Vertex ver0;
        Vertex ver1;
        Vertex ver2;
        bool hit = false;
        // Intersect with all triangles of all meshes.
        for (const auto& mesh : m_pScene->meshes) {
            for (const auto& tri : mesh.triangles) {
                const auto v0 = mesh.vertices[tri[0]];
                const auto v1 = mesh.vertices[tri[1]];
                const auto v2 = mesh.vertices[tri[2]];
                if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                    hitInfo.material = mesh.material;
                    glm::vec3 cross = glm::cross((v0.position - v2.position), (v1.position - v2.position));
                    if (cross == glm::vec3(0, 0, 0)) {
                        hitInfo.normal = glm::vec3(1, 0, 0);
                    } else {
                        hitInfo.normal = glm::normalize(cross);
                    }

                    ver0 = v0;
                    ver1 = v1;
                    ver2 = v2;

                    hit = true;
                }
            }
        }

        // Visual debug interpolated normal
        if (features.enableNormalInterp && hit) {
            glm::vec3 intersection_point = ray.origin + ray.t * ray.direction;
            glm::vec3 barycentricCoordinates = computeBarycentricCoord(ver0.position, ver1.position, ver2.position, intersection_point);
            glm::vec3 interpolatedNormal = glm::normalize(interpolateNormal(ver0.normal, ver1.normal, ver2.normal, barycentricCoordinates));
            hitInfo.normal = interpolatedNormal;

            // interpolated normal will be green
            drawRay({ intersection_point, interpolatedNormal, 1.0f }, glm::vec3(0, 1, 0));

            // draw normals of the intersected triangles
            // normals will be drawn blue
            Ray ray0 = { ver0.position, ver0.normal, 1.0f };
            Ray ray1 = { ver1.position, ver1.normal, 1.0f };
            Ray ray2 = { ver2.position, ver2.normal, 1.0f };

            drawRay(ray0, glm::vec3(0.0f, 0.0f, 1.0f));
            drawRay(ray1, glm::vec3(0.0f, 0.0f, 1.0f));
            drawRay(ray2, glm::vec3(0.0f, 0.0f, 1.0f));
        }

        // Intersect with spheres.
        for (const auto& sphere : m_pScene->spheres) {
            if (intersectRayWithShape(sphere, ray, hitInfo)) {
                hitInfo.material = sphere.material;
                hitInfo.normal = (ray.origin + ray.direction * ray.t) - sphere.center;
                hit = true;
            } else {
                // TODO: implement here the bounding volume hierarchy traversal.
                // Please note that you should use `features.enableNormalInterp` and `features.enableTextureMapping`
                // to isolate the code that is only needed for the normal interpolation and texture mapping features.
                return false;
            }
        }

        // Adding textureCoordinates
        if (hit) {
             glm::vec3 p = ray.origin + ray.t * ray.direction;
             glm::vec3 b_crods = computeBarycentricCoord(ver0.position, ver1.position, ver2.position, p);
             hitInfo.texCoord = interpolateTexCoord(ver0.texCoord, ver1.texCoord, ver2.texCoord, b_crods);
        }

        return hit;
    } else {
        // TODO: implement here the bounding volume hierarchy traversal.
        // Please note that you should use `features.enableNormalInterp` and `features.enableTextureMapping`
        // to isolate the code that is only needed for the normal interpolation and texture mapping features.
        return false;
    }
}