#include "interpolate.h"
#include <glm/geometric.hpp>

glm::vec3 computeBarycentricCoord (const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    // Given the intersection point inside a triangle, compute its barycentric coordinates
    // Sources: CG lecture slides
    float A = 0.5 * std::abs(glm::length(glm::cross((v2 - v0), (v1 - v0))));

    float alpha = (0.5 * std::abs(glm::length(glm::cross((v1 - p), (v2 - p))) / A));
    float beta = (0.5 * std::abs(glm::length(glm::cross((v0 - p), (v2 - p))) / A));
    float gamma = 1 - alpha - beta;
    return glm::vec3(alpha, beta, gamma);
}

glm::vec3 interpolateNormal (const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord)
{
    // Then use the barycentric weights to interpolate the vertices normals
    // It is computed as the normalized average of the surface normals of the faces that contain that vertex
    const glm::vec3 normal = ( barycentricCoord.x * n0 + barycentricCoord.y * n1 + barycentricCoord.z * n2 );
    return normal;
}

glm::vec2 interpolateTexCoord (const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord)
{
// TODO: implement this function.
    return t0 * barycentricCoord[0] + t1 * barycentricCoord[1] + t2 * barycentricCoord[2];
}
