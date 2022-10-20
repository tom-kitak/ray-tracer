#include "intersect.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <limits>


bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
    if (glm::cross((v0 - v2), (v1 - v2)) == glm::vec3(0, 0, 0)) {
        return false;
    }
    glm::vec3 vec0 = v1 - v0, vec1 = v2 - v0, vec2 = p - v0;
    float d00 = glm::dot(vec0, vec0);
    float d01 = glm::dot(vec0, vec1);
    float d11 = glm::dot(vec1, vec1);
    float d20 = glm::dot(vec2, vec0);
    float d21 = glm::dot(vec2, vec1);
    float denom = d00 * d11 - d01 * d01;
    float a = (d11 * d20 - d01 * d21) / denom;
    float b = (d00 * d21 - d01 * d20) / denom;
    if (a < 0) {
        return false;
    }
    if (b < 0) {
        return false;
    }
    if (a + b > 1) {
        return false;
    }
    return true;
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    if (glm::dot(ray.direction, plane.normal) == 0) {
        return false;
    }
    float t = (plane.D - glm::dot(ray.origin, plane.normal)) / glm::dot(ray.direction, plane.normal);
    if (t > ray.t || t < 0) {
        return false;
    }
    ray.t = t;
    return true;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    glm::vec3 cross = glm::cross((v0 - v2), (v1 - v2));
    glm::vec3 normal;
    if (cross == glm::vec3(0, 0, 0)) {
        normal = glm::vec3(1, 0, 0);
    } else {
        normal = glm::normalize(cross);
    }
    float d = glm::dot(normal, v0);
    Plane plane = { d, normal };
    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    float t = ray.t;
    glm::vec3 normal = hitInfo.normal;
    Plane plane = trianglePlane(v0, v1, v2);
    if (!intersectRayWithPlane(plane, ray)) {
        return false;
    }
    if (!pointInTriangle(v0, v1, v2, normal, ray.origin + ray.t * ray.direction)) {
        ray.t = t;
        return false;
    }
    return true;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    float oldT = ray.t;
    glm::vec3 origin = ray.origin - sphere.center;
    float t;
    float a = pow(ray.direction.x, 2) + pow(ray.direction.y, 2) + pow(ray.direction.z, 2);
    float b = 2 * (ray.direction.x * origin.x + ray.direction.y * origin.y + ray.direction.z * origin.z);
    float c = pow(origin.x, 2) + pow(origin.y, 2) + pow(origin.z, 2) - pow(sphere.radius, 2);
    float discriminant = pow(b, 2) - 4 * a * c;
    if (discriminant < 0) {
        return false;
    }
    if (discriminant == 0) {
        t = -b / (2 * a);
        if (t < 0) {
            return false;
        }
    } else {
        float t1 = (-b + sqrt(discriminant)) / (2 * a);
        float t2 = (-b - sqrt(discriminant)) / (2 * a);
        if (t1 >= 0 && t2 >= 0) {
            t = std::min(t1, t2);
        } else if (t1 >= 0) {
            t = t1;
        } else if (t2 >= 0) {
            t = t2;
        } else {
            return false;
        }
    }
    if (oldT < t) {
        return false;
    }
    ray.t = t;
    return true;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    float txMin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float tyMin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tzMin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float txMax = (box.upper.x - ray.origin.x) / ray.direction.x;
    float tyMax = (box.upper.y - ray.origin.y) / ray.direction.y;
    float tzMax = (box.upper.z - ray.origin.z) / ray.direction.z;
    float tInx = std::min(txMin, txMax);
    float tOutx = std::max(txMin, txMax);
    float tIny = std::min(tyMin, tyMax);
    float tOuty = std::max(tyMin, tyMax);
    float tInz = std::min(tzMin, tzMax);
    float tOutz = std::max(tzMin, tzMax);
    float tIn = std::max(std::max(tInx, tIny), tInz);
    float tOut = std::min(std::min(tOutx, tOuty), tOutz);
    if (tIn > tOut || tOut < 0) {
        return false;
    }
    if (tIn < 0) {
        ray.t = tOut;
        return true;
    }
    if (tIn < ray.t) {
        ray.t = tIn;
        return true;
    }
    return false;
}
