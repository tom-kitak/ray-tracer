#include "light.h"
#include "config.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <texture.h>

void hardShadowVisualDebug(PointLight point_light, const BvhInterface& bvh, Ray ray, const Features& features, HitInfo hitInfo);
void softShadowsVisualDebug(Ray ray, glm::vec3 color, glm::vec3 samplePos, const Features& features, const BvhInterface& bvh, HitInfo hitInfo);

// samples a segment light source
// you should fill in the vectors position and color with the sampled position and color
void sampleSegmentLight(const SegmentLight& segmentLight, glm::vec3& position, glm::vec3& color)
{
    // 1. Sample a position from segmentLight: get random vector
    glm::vec3 p0_to_p1 = segmentLight.endpoint1 - segmentLight.endpoint0;
    float random = (float)rand() / RAND_MAX;
    position = segmentLight.endpoint0 + random * p0_to_p1;

    // 2. Calculate the color
    color = segmentLight.color0 * (1 - random) + segmentLight.color1 * random;
}

// samples a parallelogram light source
// you should fill in the vectors position and color with the sampled position and color
void sampleParallelogramLight(const ParallelogramLight& parallelogramLight, glm::vec3& position, glm::vec3& color)
{
    // 1. Sample a position from parallelogramLight: get random vector

    float alpha = (float)rand() / RAND_MAX;
    float beta = (float)rand() / RAND_MAX;
    position = parallelogramLight.v0 + alpha * parallelogramLight.edge01 + beta * parallelogramLight.edge02;

    // 2. Calculate the color
    color = ((1 - alpha) * (1 - beta)) * parallelogramLight.color0 + ((alpha) * (1 - beta)) * parallelogramLight.color1 + ((1 - alpha) * (beta)) * parallelogramLight.color2 + ((alpha) * (beta)) * parallelogramLight.color3;

}

// test the visibility at a given light sample
// returns 1.0 if sample is visible, 0.0 otherwise
float testVisibilityLightSample(const glm::vec3& samplePos, const glm::vec3& debugColor, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo)
{
    // TODO: implement this function.
    glm::vec3 offset(-0.00001f);
    glm::vec3 intersection_point = ray.origin + ray.direction * ray.t + offset * ray.direction;
    
    float shadow_vec_t = glm::length(samplePos - intersection_point);
    if (shadow_vec_t == 0.0f) {
        return 1.0f;
    }
    glm::vec3 shadow_vec_dir = glm::normalize(samplePos - intersection_point);
    
    
    Ray shadow_ray { intersection_point, shadow_vec_dir, shadow_vec_t };
    
    bool hit_before = bvh.intersect(shadow_ray, hitInfo, features);

    if (hit_before) {
        return 0.0;
    } else {
        return 1.0;
    }
}

// given an intersection, computes the contribution from all light sources at the intersection point
// in this method you should cycle the light sources and for each one compute their contribution
// don't forget to check for visibility (shadows!)

// Lights are stored in a single array (scene.lights) where each item can be either a PointLight, SegmentLight or ParallelogramLight.
// You can check whether a light at index i is a PointLight using std::holds_alternative:
// std::holds_alternative<PointLight>(scene.lights[i])
//
// If it is indeed a point light, you can "convert" it to the correct type using std::get:
// PointLight pointLight = std::get<PointLight>(scene.lights[i]);
//
//
// The code to iterate over the lights thus looks like this:
// for (const auto& light : scene.lights) {
//     if (std::holds_alternative<PointLight>(light)) {
//         const PointLight pointLight = std::get<PointLight>(light);
//         // Perform your calculations for a point light.
//     } else if (std::holds_alternative<SegmentLight>(light)) {
//         const SegmentLight segmentLight = std::get<SegmentLight>(light);
//         // Perform your calculations for a segment light.
//     } else if (std::holds_alternative<ParallelogramLight>(light)) {
//         const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
//         // Perform your calculations for a parallelogram light.
//     }
// }
//
// Regarding the soft shadows for **other** light sources **extra** feature:
// To add a new light source, define your new light struct in scene.h and modify the Scene struct (also in scene.h)
// by adding your new custom light type to the lights std::variant. For example:
// std::vector<std::variant<PointLight, SegmentLight, ParallelogramLight, MyCustomLightType>> lights;
//
// You can add the light sources programmatically by creating a custom scene (modify the Custom case in the
// loadScene function in scene.cpp). Custom lights will not be visible in rasterization view.
glm::vec3 computeLightContribution(const Scene& scene, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo)
{
    Ray rayCopy = ray;
    glm::vec3 ret = glm::vec3(0, 0, 0);
    for (const auto& light : scene.lights) {
        if (std::holds_alternative<PointLight>(light)) {
            const PointLight pointLight = std::get<PointLight>(light);
            glm::vec3 retColor;
            if (features.enableShading) {
                retColor = computeShading(pointLight.position, pointLight.color, features, ray, hitInfo);
            } else if (features.enableTextureMapping) {
                retColor = acquireTexel(*hitInfo.material.kdTexture.get(), hitInfo.texCoord, features);
            } else {
                retColor = hitInfo.material.kd;
            }
            if (features.enableHardShadow && testVisibilityLightSample(pointLight.position, pointLight.color, bvh, features, ray, hitInfo) == 0.0f) {
                retColor = glm::vec3(0, 0, 0);
            }
            if (features.enableHardShadow) {
                hardShadowVisualDebug(pointLight, bvh, rayCopy, features, hitInfo);
            }
            ret += retColor;
        } else if (std::holds_alternative<SegmentLight>(light)) {
            const SegmentLight segmentLight = std::get<SegmentLight>(light);
            glm::vec3 position;
            glm::vec3 color;
            glm::vec3 retColor = glm::vec3(0, 0, 0);
            for (int i = 0; i < 100; i++) {
                glm::vec3 temp = glm::vec3(0, 0, 0);
                sampleSegmentLight(segmentLight, position, color);
                if (features.enableShading) {
                    temp = computeShading(position, color, features, ray, hitInfo);
                } else if (features.enableTextureMapping) {
                    retColor = acquireTexel(*hitInfo.material.kdTexture.get(), hitInfo.texCoord, features);
                } else {
                    temp = hitInfo.material.kd;
                }
                if (features.enableSoftShadow && testVisibilityLightSample(position, color, bvh, features, ray, hitInfo) == 0.0f) {
                    temp = glm::vec3(0, 0, 0);
                }
                if (features.enableSoftShadow) {
                    HitInfo dummy;
                    softShadowsVisualDebug(rayCopy, color, position, features, bvh, dummy);
                }
                retColor += temp;
            }
            ret += retColor / 100.0f;
        } else if (std::holds_alternative<ParallelogramLight>(light)) {
            const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
            glm::vec3 position;
            glm::vec3 color;
            glm::vec3 retColor = glm::vec3(0, 0, 0);
            for (int i = 0; i < 100; i++) {
                glm::vec3 temp = glm::vec3(0, 0, 0);
                sampleParallelogramLight(parallelogramLight, position, color);
                if (features.enableShading) {
                    temp = computeShading(position, color, features, ray, hitInfo);
                } else if (features.enableTextureMapping) {
                    retColor = acquireTexel(*hitInfo.material.kdTexture.get(), hitInfo.texCoord, features);
                } else {
                    temp = hitInfo.material.kd;
                }
                if (features.enableSoftShadow && testVisibilityLightSample(position, color, bvh, features, ray, hitInfo) == 0.0f) {
                    temp = glm::vec3(0, 0, 0);
                }
                if (features.enableSoftShadow) {
                    HitInfo dummy;
                    softShadowsVisualDebug(rayCopy, color, position, features, bvh, dummy);
                }
                retColor += temp;
            }
            ret += retColor / 100.0f;
        }
    }
    return ret;
}

void hardShadowVisualDebug(PointLight point_light, const BvhInterface& bvh, Ray ray, const Features& features, HitInfo hitInfo)
{
    glm::vec3 offset(-0.00001f);
    glm::vec3 intersection_point = ray.origin + ray.direction * ray.t + offset * ray.direction;

    glm::vec3 samplePos = point_light.position;

    float shadow_vec_t = glm::length(samplePos - intersection_point);
    if (shadow_vec_t == 0.0f) {
        drawRay(Ray { intersection_point, glm::vec3(0.0f), 0 }, glm::vec3(1.0f));
        return;
    }
    glm::vec3 shadow_vec_dir = glm::normalize(samplePos - intersection_point);

    Ray ray_towards_light { intersection_point, shadow_vec_dir, shadow_vec_t };

    bool hit_before = bvh.intersect(ray_towards_light, hitInfo, features);

    if (hit_before) {
        drawRay(ray_towards_light, glm::vec3(1.0f, 0.0f, 0.0f));
    } else {
        drawRay(ray_towards_light, glm::vec3(1.0f));
    }
}

void softShadowsVisualDebug(Ray ray, glm::vec3 color, glm::vec3 samplePos, const Features& features, const BvhInterface& bvh, HitInfo hitInfo)
{
    glm::vec3 offset(-0.0001f);
    glm::vec3 intersection_point = ray.origin + ray.direction * ray.t + offset * ray.direction;

    float light_vec_t = glm::length(samplePos - intersection_point);
    if (light_vec_t == 0.0f) {
        drawRay(Ray { intersection_point, glm::vec3(0.0f), 0 }, glm::vec3(1.0f));
        return;
    }
    glm::vec3 light_vec_dir = glm::normalize(samplePos - intersection_point);

    Ray ray_towards_light { intersection_point, light_vec_dir, light_vec_t };

    bool hit_before = bvh.intersect(ray_towards_light, hitInfo, features);

    if (hit_before) {
        drawRay(ray_towards_light, glm::vec3(1.0f, 0.0f, 0.0f));
    } else {
        drawRay(ray_towards_light, color);
    }
}
