#include "texture.h"
#include <cmath>
#include <glm/geometric.hpp>
#include <shading.h>

const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo)
{
    glm::vec3 hitPos = ray.origin + ray.t * ray.direction;
    glm::vec3 viewDir = glm::normalize(ray.origin - hitPos);
    glm::vec3 lightDir = glm::normalize(lightPosition - hitPos);
    glm::vec3 reflectDir = glm::reflect(-lightDir, glm::normalize(hitInfo.normal));
    // glm::vec3 reflectDir = glm::normalize(computeReflectionRay(ray, hitInfo).direction);
    float specAngle = std::max(glm::dot(reflectDir, viewDir), 0.0f);
    float specular = std::pow(specAngle, hitInfo.material.shininess);
    float angle = std::max(glm::dot(glm::normalize(hitInfo.normal), lightDir), 0.0f);
    return (specular * hitInfo.material.ks * lightColor) + (angle * lightColor * hitInfo.material.kd);
}


const Ray computeReflectionRay (Ray ray, HitInfo hitInfo)
{
    // Do NOT use glm::reflect!! write your own code.
    Ray reflectionRay {};
    // TODO: implement the reflection ray computation.
    return reflectionRay;
}