#include "bounding_volume_hierarchy.h"
#include "config.h"
#include <framework/trackball.h>
#include "draw.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include "interpolate.h"
#include <glm/glm.hpp>
#include "common.h"
#include <queue>

bool overlap1D(float box1Min, float box1Max, float box2Min, float box2Max);
bool overlap(AxisAlignedBox a1, AxisAlignedBox a2);
void getIntersections(auto& q, Ray ray, Node node, std::vector<Node> nodes);
bool intersectNodes(auto& q, Ray& ray, HitInfo& hitInfo, Features features, std::vector<Node> nodes, Scene* scene, Vertex& ver0, Vertex& ver1, Vertex& ver2);
float calculateSurface(Node node);

Features currentFeatures; 
bool showX = false;
bool showY = true;
bool showZ = true;

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, const Features& features)
    : m_pScene(pScene)
{
    currentFeatures = features;

    // make root node
    Node root;
    root.typeLeaf = 1;
    std::vector<Tuple> indices = createTuples();
    root.indices = indices;
    std::tuple<glm::vec3, glm::vec3> bounds = findBounds(indices);
    root.lowerBound = std::get<0>(bounds);
    root.upperBound = std::get<1>(bounds);

    m_nodes.push_back(root);

    m_numLevels = recursiveSplit(0, 0, 10);

    m_numLeaves = 0;

    for (Node n : m_nodes) {
        if (n.typeLeaf == 1) {
            m_leaves.push_back(n);
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

std::tuple<Node, Node> BoundingVolumeHierarchy::split(Node node, char axis, float median)
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

    float m;
    if (currentFeatures.extra.enableBvhSahBinning) {
        // use median passed from findSAH
        m = median;
    } else {
        // find median using centroids
        if (centroids.size() % 2 == 0) { // even values
            m = centroids[(centroids.size() / 2) - 1];
        } else { // odd values
            m = centroids[(centroids.size() - 1) / 2];
        }
    }

    // create vector of triangles less than median, and more than median
    std::vector<Tuple> lessThanMedian;
    std::vector<Tuple> moreThanMedian;
    for (std::tuple<Tuple, float> tuple : matchTriangleToC) {
        float triangleCentroid = std::get<1>(tuple);
        if (triangleCentroid < m) {
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

        std::tuple<Node, Node> children;
        std::vector<float> medians;
        if (currentFeatures.extra.enableBvhSahBinning) {
            children = findSAH(parent);
        } else {
            // we can just pass 0 as the median, because when we're not doing SAH, 
            // we do not use that median, we calculate it
            if (currentLevel % 3 == 0) {
                // split on the x axis
                children = split(parent, 'x', 0);
            } else if (currentLevel % 3 == 1) {
                // split on the y axis
                children = split(parent, 'y', 0);
            } else { // currentLevel % 3 == 2
                // split on the z axis
                children = split(parent, 'z', 0);
            }
        }

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
        int y = recursiveSplit(childIndexRight, currentLevel + 1, maxLevel);
        return std::max(x, y);
    }
}

std::tuple<Node, Node> BoundingVolumeHierarchy::findSAH(Node node) 
{
    // splits the node based on the surface area heuristic

    std::tuple<Node, Node> result;

    float startx = node.lowerBound.x;
    float starty = node.lowerBound.y;
    float startz = node.lowerBound.z;

    float dx = node.upperBound.x - node.lowerBound.x;
    float dy = node.upperBound.y - node.lowerBound.y;
    float dz = node.upperBound.z - node.lowerBound.z;

    // calculate medians
    std::vector<float> medians;
    for (int i = 1; i < 4; i++) {
        medians.push_back((dx * i * (1 / 3.0f)) + startx);
        medians.push_back((dy * i * (1 / 3.0f)) + starty);
        medians.push_back((dz * i * (1 / 3.0f)) + startz);
    }

    // calculate options for splits
    std::vector<std::tuple<Node, Node>> splitOptions;
    for (int i = 0; i < medians.size(); i++) {
        std::tuple<Node, Node> children;
        if (i % 3 == 0) {
            // split on the x axis
            children = split(node, 'x', medians[i]);
        } else if (i % 3 == 1) {
            // split on the y axis
            children = split(node, 'y', medians[i]);
        } else { // i % 3 == 2
            // split on the z axis
            children = split(node, 'z', medians[i]);
        }
        splitOptions.push_back(children);
    }

    // calculate surface of the current node
    float s = calculateSurface(node);

    // calculate all the costs
    std::vector<float> costs;
    for (int i = 0; i < splitOptions.size(); i++) {
        float s1 = calculateSurface(std::get<0>(splitOptions[i]));
        float s2 = calculateSurface(std::get<1>(splitOptions[i]));
        float cost = ((s1 / s) * std::get<0>(splitOptions[i]).indices.size()) + ((s2 / s) * std::get<1>(splitOptions[i]).indices.size());
        costs.push_back(cost);
    }

    // find the minimum cost
    float min = std::numeric_limits<float>::max();
    int index = -1;
    for (int i = 0; i < costs.size(); i++) {
        if (costs[i] < min) {
            min = costs[i];
            index = i;
        }
    }

    // if the index is still -1, 
    // it means the current node could either not be split or the best decision is to not split the current node
    // so we return an empty result
    if (index == -1) {
        return result;
    }

    // if the current node should be split, we return the best split we found with SAH
    result = splitOptions[index];
    return result;
}

float calculateSurface(Node node) {
    // Source: https://math.stackexchange.com/questions/2541655/calculation-of-the-volume-and-surface-of-a-cubo%C3%AFd-knowing-the-coordinates-of-two
    float L = node.upperBound.x - node.lowerBound.x;
    float B = node.upperBound.y - node.lowerBound.y;
    float H = node.upperBound.z - node.lowerBound.z;
    float surfaceArea = 2 * (L * B + B * H + H * L);
    return surfaceArea;
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

        // visual debug for SAH, showing the next possible bins on the specified axis
        if (currentFeatures.extra.enableBvhSahBinning && (showX || showY || showZ)) {
            Node curr = m_nodes[nodeIndex];
            float startx = curr.lowerBound.x;
            float starty = curr.lowerBound.y;
            float startz = curr.lowerBound.z;

            float dx = curr.upperBound.x - curr.lowerBound.x;
            float dy = curr.upperBound.y - curr.lowerBound.y;
            float dz = curr.upperBound.z - curr.lowerBound.z;

            for (int i = 1; i < 4; i++) {
                AxisAlignedBox aabb = { curr.lowerBound, curr.upperBound };

                // Show options for the split on the x axis
                if (showX) {
                    float upperX = (dx * i * (1 / 3.0f)) + startx;
                    aabb.upper.x = upperX;
                    drawAABB(aabb, DrawMode::Wireframe, { 1.0f, 0.0f, 0.0f }, 1.0f);
                }

                aabb = { curr.lowerBound, curr.upperBound };

                // Show options for the split on the y axis
                if (showY) {
                    float upperY = (dy * i * (1 / 3.0f)) + starty;
                    aabb.upper.y = upperY;
                    drawAABB(aabb, DrawMode::Wireframe, { 0.0f, 1.0f, 0.0f }, 1.0f);
                }

                aabb = { curr.lowerBound, curr.upperBound };

                // Show options for the split on the z axis
                if (showZ) {
                    float upperZ = (dz * i * (1 / 3.0f)) + startz;
                    aabb.upper.z = upperZ;
                    drawAABB(aabb, DrawMode::Wireframe, { 0.0f, 0.0f, 1.0f }, 1.0f);
                }
            }
        }
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
    if (leafIdx >= m_leaves.size()) {
        return;
    }

    // draw aabb
    AxisAlignedBox aabb { m_leaves[leafIdx].lowerBound, m_leaves[leafIdx].upperBound };
    drawAABB(aabb, DrawMode::Wireframe);

    // colors for the triangle
    std::vector<glm::vec3> colorVector = {
        { 0, 1, 0 },
        { 1, 0, 1 },
        { 1, 1, 0 },
        { 0, 0, 1 },
        { 1, 0, 0 }
    };

    // draw triangles
    std::vector<Tuple> indices = m_leaves[leafIdx].indices;
    for (int i = 0; i < indices.size(); i++) {
        Tuple currentTuple = indices[i];

        Mesh mesh = m_pScene->meshes[currentTuple.first];
        glm::vec3 triangle = mesh.triangles[currentTuple.second];

        Vertex v0 = mesh.vertices[triangle.x];
        Vertex v1 = mesh.vertices[triangle.y];
        Vertex v2 = mesh.vertices[triangle.z];

        int index = i % colorVector.size();

        drawTriangle(v0, v1, v2, colorVector[index]);
    }    
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h.
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, const Features& features, const Trackball& camera) const
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

        if (hit && features.extra.enableMipmapTextureFiltering && hitInfo.material.kdTexture) {
            Config config = {};
            // Calculate the footprint of the pixel by shooting rays from edges of the pixel and taking their texture coordinates.
            Ray ray1 = { ray.origin, glm::normalize(glm::vec3(ray.direction.x + (1.0 / config.windowSize.x) * camera.halfScreenSpaceWidth(), ray.direction.y, ray.direction.z)),
                std::numeric_limits<float>::max() };
            Ray ray2 = { ray.origin, glm::normalize(glm::vec3(ray.direction.x, ray.direction.y + (1.0 / config.windowSize.y) * camera.halfScreenSpaceHeight(), ray.direction.z)),
                std::numeric_limits<float>::max() };
            Features temp = features;
            HitInfo dummy;
            temp.extra.enableMipmapTextureFiltering = false;
            if (intersect(ray1, dummy, temp, camera) && intersect(ray2, dummy, temp, camera)) {
                glm::vec3 barCoord1 = computeBarycentricCoord(ver0.position, ver1.position, ver2.position, ray1.origin + ray1.direction * ray1.t);
                glm::vec2 texCoord1 = interpolateTexCoord(ver0.texCoord, ver1.texCoord, ver2.texCoord, barCoord1);
                glm::vec3 barCoord2 = computeBarycentricCoord(ver0.position, ver1.position, ver2.position, ray2.origin + ray2.direction * ray2.t);
                glm::vec2 texCoord2 = interpolateTexCoord(ver0.texCoord, ver1.texCoord, ver2.texCoord, barCoord2);
                glm::vec2 vec1 = texCoord1 - hitInfo.texCoord;
                glm::vec2 vec2 = texCoord2 - hitInfo.texCoord;
                glm::vec2 point1 = hitInfo.texCoord + vec1 + vec2;
                glm::vec2 point2 = hitInfo.texCoord + vec1 - vec2;
                float x = std::max(std::abs(point1.x - hitInfo.texCoord.x), std::abs(point2.x - hitInfo.texCoord.x));
                float y = std::max(std::abs(point1.y - hitInfo.texCoord.y), std::abs(point2.y - hitInfo.texCoord.y));
                // Set m to the longest side of the aabb of the box formed by the texture coordinates.
                hitInfo.m = std::max(x, y) * 2;
                drawRay(ray1, glm::vec3(1, 0, 0));
                drawRay(ray2, glm::vec3(1, 0, 0));
            }
        }

        // Adding textureCoordinates
        if (hit && features.enableTextureMapping && hitInfo.material.kdTexture) {
            
            glm::vec3 p = ray.origin + ray.t * ray.direction;
            glm::vec3 b_crods = computeBarycentricCoord(ver0.position, ver1.position, ver2.position, p);
            hitInfo.texCoord = interpolateTexCoord(ver0.texCoord, ver1.texCoord, ver2.texCoord, b_crods);
            hitInfo.material.kd = acquireTexel(*hitInfo.material.kdTexture, hitInfo.texCoord, features, ray, hitInfo);
        }

        return hit;
    } else {
        // TODO: implement here the bounding volume hierarchy traversal.
        // Please note that you should use `features.enableNormalInterp` and `features.enableTextureMapping`
        // to isolate the code that is only needed for the normal interpolation and texture mapping features.
        AxisAlignedBox aabb = { m_nodes[0].lowerBound, m_nodes[0].upperBound };
        Ray rayCopy = ray;
        if (!intersectRayWithShape(aabb, rayCopy)) {
            return false;
        }
        Vertex ver0;
        Vertex ver1;
        Vertex ver2;
        auto compare = [](glm::vec2 l, glm::vec2 r) {
            return l.y > r.y;
        };
        std::priority_queue<glm::vec2, std::vector<glm::vec2>, decltype(compare)> q(compare);
        q.push(glm::vec2(0, rayCopy.t));
        getIntersections(q, ray, m_nodes[0], m_nodes);
        bool ret = intersectNodes(q, ray, hitInfo, features, m_nodes, m_pScene, ver0, ver1, ver2);
        drawTriangle(ver0, ver1, ver2, glm::vec3(1, 1, 1));

        if (features.enableNormalInterp && ret) {
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

        if (ret && features.extra.enableMipmapTextureFiltering && hitInfo.material.kdTexture) {
            Config config = {};
            // Calculate the footprint of the pixel by shooting rays from edges of the pixel and taking their texture coordinates.
            Ray ray1 = { ray.origin, glm::normalize(glm::vec3(ray.direction.x + (1.0 / config.windowSize.x) * camera.halfScreenSpaceWidth(), ray.direction.y, ray.direction.z)),
                std::numeric_limits<float>::max() };
            Ray ray2 = { ray.origin, glm::normalize(glm::vec3(ray.direction.x, ray.direction.y + (1.0 / config.windowSize.y) * camera.halfScreenSpaceHeight(), ray.direction.z)),
                std::numeric_limits<float>::max() };
            Features temp = features;
            HitInfo dummy;
            temp.extra.enableMipmapTextureFiltering = false;
            if (intersect(ray1, dummy, temp, camera) && intersect(ray2, dummy, temp, camera)) {
                glm::vec3 barCoord1 = computeBarycentricCoord(ver0.position, ver1.position, ver2.position, ray1.origin + ray1.direction * ray1.t);
                glm::vec2 texCoord1 = interpolateTexCoord(ver0.texCoord, ver1.texCoord, ver2.texCoord, barCoord1);
                glm::vec3 barCoord2 = computeBarycentricCoord(ver0.position, ver1.position, ver2.position, ray2.origin + ray2.direction * ray2.t);
                glm::vec2 texCoord2 = interpolateTexCoord(ver0.texCoord, ver1.texCoord, ver2.texCoord, barCoord2);
                glm::vec2 vec1 = texCoord1 - hitInfo.texCoord;
                glm::vec2 vec2 = texCoord2 - hitInfo.texCoord;
                glm::vec2 point1 = hitInfo.texCoord + vec1 + vec2;
                glm::vec2 point2 = hitInfo.texCoord + vec1 - vec2;
                float x = std::max(std::abs(point1.x - hitInfo.texCoord.x), std::abs(point2.x - hitInfo.texCoord.x));
                float y = std::max(std::abs(point1.y - hitInfo.texCoord.y), std::abs(point2.y - hitInfo.texCoord.y));
                // Set m to the longest side of the aabb of the box formed by the texture coordinates.
                hitInfo.m = std::max(x, y) * 2;
                drawRay(ray1, glm::vec3(1, 0, 0));
                drawRay(ray2, glm::vec3(1, 0, 0));
            }
        }

        // Adding textureCoordinates
        if (ret && features.enableTextureMapping && hitInfo.material.kdTexture) {
            glm::vec3 p = ray.origin + ray.t * ray.direction;
            glm::vec3 b_crods = computeBarycentricCoord(ver0.position, ver1.position, ver2.position, p);
            hitInfo.texCoord = interpolateTexCoord(ver0.texCoord, ver1.texCoord, ver2.texCoord, b_crods);
            hitInfo.material.kd = acquireTexel(*hitInfo.material.kdTexture, hitInfo.texCoord, features, ray, hitInfo);
        }

        return ret;
    }

}

// Add all the intersected nodes to a priority queue.
void getIntersections(auto& q, Ray ray, Node node, std::vector<Node> nodes)
{
    if (node.typeLeaf == 0) {
        Node node1 = nodes[node.indices[0].first];
        Node node2 = nodes[node.indices[0].second];
        AxisAlignedBox aabb1 = { node1.lowerBound, node1.upperBound };
        AxisAlignedBox aabb2 = { node2.lowerBound, node2.upperBound };
        Ray rayCopy1 = ray;
        Ray rayCopy2 = ray;
        if (intersectRayWithShape(aabb1, rayCopy1)) {
            // Add a vec2 of the index and the ray.t to the priority queue.
            q.push(glm::vec2(node.indices[0].first, rayCopy1.t));
            getIntersections(q, ray, node1, nodes);
        }
        if (intersectRayWithShape(aabb2, rayCopy2)) {
            // Add a vec2 of the index and the ray.t to the priority queue.
            q.push(glm::vec2(node.indices[0].second, rayCopy2.t));
            getIntersections(q, ray, node2, nodes);
        }
    }
}

bool intersectNodes(auto& q, Ray& ray, HitInfo& hitInfo, Features features, std::vector<Node> nodes, Scene* scene, Vertex& ver0, Vertex& ver1, Vertex& ver2)
{
    bool hit = false;
    AxisAlignedBox aabb;
    glm::vec3 drawColor = glm::vec3(1, 1, 1);
    bool option = true;
    while (!q.empty()) {
        Node node = nodes[q.top().x];
        q.pop();
        if (node.typeLeaf == 1) {
            if (!hit) {
                for (const Tuple t : node.indices) {
                    Mesh mesh = scene->meshes[t.first];
                    const auto v0 = mesh.vertices[mesh.triangles[t.second][0]];
                    const auto v1 = mesh.vertices[mesh.triangles[t.second][1]];
                    const auto v2 = mesh.vertices[mesh.triangles[t.second][2]];
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
                        aabb = { node.lowerBound, node.upperBound };
                        drawAABB(aabb, DrawMode::Wireframe, glm::vec3(1, 1, 1));
                        hit = true;
                    }
                }
            } else if (overlap(aabb, AxisAlignedBox { node.lowerBound, node.upperBound })) {
                for (const Tuple t : node.indices) {
                    Mesh mesh = scene->meshes[t.first];
                    const auto v0 = mesh.vertices[mesh.triangles[t.second][0]];
                    const auto v1 = mesh.vertices[mesh.triangles[t.second][1]];
                    const auto v2 = mesh.vertices[mesh.triangles[t.second][2]];
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

                    }
                }
                drawAABB({ node.lowerBound, node.upperBound }, DrawMode::Wireframe, glm::vec3(1, 1, 1));
            } else {
                // If there is already a hit and this node doesn't overlap with the aabb of the first hit stop the traversal.
                if (option) {
                    drawColor = glm::vec3(1, 0, 0);
                }
                drawAABB({ node.lowerBound, node.upperBound }, DrawMode::Wireframe, drawColor);
            }
        } else {
            if (hit && !overlap(aabb, AxisAlignedBox{ node.lowerBound, node.upperBound }) && option) {
                drawColor = glm::vec3(1, 0, 0);
                drawAABB({ node.lowerBound, node.upperBound }, DrawMode::Wireframe, drawColor);
            } else {
                drawAABB({ node.lowerBound, node.upperBound }, DrawMode::Wireframe, drawColor);
            }
        }
    }
    return hit;
}

bool overlap(AxisAlignedBox a1, AxisAlignedBox a2) {
    return overlap1D(a1.lower.x, a1.upper.x, a2.lower.x, a2.upper.x) && overlap1D(a1.lower.y, a1.upper.y, a2.lower.y, a2.upper.y)
        && overlap1D(a1.lower.z, a1.upper.z, a2.lower.z, a2.upper.z);
}

bool overlap1D(float box1Min, float box1Max, float box2Min, float box2Max) {
    return (box1Max >= box2Min && box2Max >= box1Min);
}
