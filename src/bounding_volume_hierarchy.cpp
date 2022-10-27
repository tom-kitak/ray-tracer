#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include "interpolate.h"
#include <glm/glm.hpp>


BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    // TODO: implement BVH construction.
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 1.
int BoundingVolumeHierarchy::numLevels() const
{
    return 1;
}

// Return the number of leaf nodes in the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 2.
int BoundingVolumeHierarchy::numLeaves() const
{
    return 1;
}

// Use this function to visualize your BVH. This is useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDrawLevel(int level)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    AxisAlignedBox aabb { glm::vec3(0.0f), glm::vec3(0.0f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
}


// Use this function to visualize your leaf nodes. This is useful for debugging. The function
// receives the leaf node to be draw (think of the ith leaf node). Draw the AABB of the leaf node and all contained triangles.
// You can draw the triangles with different colors. NoteL leafIdx is not the index in the node vector, it is the
// i-th leaf node in the vector.
void BoundingVolumeHierarchy::debugDrawLeaf(int leafIdx)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    AxisAlignedBox aabb { glm::vec3(0.0f), glm::vec3(0.0f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);

    // once you find the leaf node, you can use the function drawTriangle (from draw.h) to draw the contained primitives
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