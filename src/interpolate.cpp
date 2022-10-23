#include "interpolate.h"
#include <glm/geometric.hpp>

glm::vec3 computeBarycentricCoord (const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    // Given the intersection point inside a triangle, compute its barycentric coordinates
    // float A = 0.5 * std::abs(glm::length(glm::cross((v2 - v0), (v1 - v0))));
    // const glm::vec3 AB = v1 - v0;
    // const glm::vec3 AC = v2 - v0;
    // float A = 0.5 * std::sqrt((pow(AB.y * AC.z - AB.z * AC.y, 2)) + (pow(AB.z * AC.x - AB.x * AC.z, 2)) + (pow(AB.x * AC.y - AB.y * AC.x, 2)));

    // float alpha = (0.5 * std::abs(glm::length(glm::cross((v1 - p), (v2 - p))) / A));
    // float beta = (0.5 * std::abs(glm::length(glm::cross((v0 - p), (v2 - p))) / A));

    float alpha = dot((p - v0), glm::normalize(v1 - v0));
    float beta = dot((p - v0), glm::normalize(v2 - v0));
    float gamma = 1 - alpha - beta;
    return glm::vec3(alpha, beta, gamma);
}

glm::vec3 interpolateNormal (const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord)
{
    // Then use the barycentric weights to interpolate the vertices normals
    //  it is computed as the normalized average of the surface normals of the faces that contain that vertex
    const glm::vec3 normal = ( barycentricCoord.x * n0, barycentricCoord.y * n1, barycentricCoord.z * n2 );
    return normal;
}

glm::vec2 interpolateTexCoord (const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord)
{
// TODO: implement this function.
    return glm::vec2(0.0);
}
