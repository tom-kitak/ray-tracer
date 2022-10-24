#include "light.h"
#include "config.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>

std::vector<std::tuple<glm::vec3, glm::vec3>> sampledLightMultipleTimes(std::variant<PointLight, SegmentLight, ParallelogramLight> light, int n);

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
    glm::vec3 ret = glm::vec3(0, 0, 0);
    for (const auto& light : scene.lights) {
        if (std::holds_alternative<PointLight>(light)) {
            const PointLight pointLight = std::get<PointLight>(light);
            glm::vec3 retColor;
            if (features.enableShading) {
                retColor = computeShading(pointLight.position, pointLight.color, features, ray, hitInfo);
            } else {
                retColor = hitInfo.material.kd;
            }
            if (features.enableHardShadow && testVisibilityLightSample(pointLight.position, pointLight.color, bvh, features, ray, hitInfo) == 0.0f) {
                retColor = glm::vec3(0, 0, 0);
            }
            ret += retColor;
        } else if (std::holds_alternative<SegmentLight>(light)) {
            const SegmentLight segmentLight = std::get<SegmentLight>(light);
            
            std::vector<std::tuple<glm::vec3, glm::vec3>> samples = sampledLightMultipleTimes(light, 100);
           
            glm::vec3 retColor = glm::vec3(0, 0, 0);

            for (int i = 0; i < 100; i++) {
                glm::vec3 temp = glm::vec3(0, 0, 0);
                sampleSegmentLight(segmentLight, position, color);
                if (features.enableShading) {
                    temp = computeShading(position, color, features, ray, hitInfo);
                } else {
                    temp = hitInfo.material.kd;
                }
                if (features.enableSoftShadow && testVisibilityLightSample(position, color, bvh, features, ray, hitInfo) == 0.0f) {
                    temp = glm::vec3(0, 0, 0);
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
                } else {
                    temp = hitInfo.material.kd;
                }
                if (features.enableSoftShadow && testVisibilityLightSample(position, color, bvh, features, ray, hitInfo) == 0.0f) {
                    temp = glm::vec3(0, 0, 0);
                }
                retColor += temp;
            }
            ret += retColor / 100.0f;
        }
    }
    return ret;
}

std::vector<std::tuple<glm::vec3, glm::vec3>> sampledLightMultipleTimes(std::variant<PointLight, SegmentLight, ParallelogramLight> light, int n)
{
    std::vector<std::tuple<glm::vec3, glm::vec3>> vec_position_color;
    for (int i = 0; i < n; i++) {
        glm::vec3 position;
        glm::vec3 color;
        if (std::holds_alternative<SegmentLight>(light)) {
            const SegmentLight segmentLight = std::get<SegmentLight>(light);
            sampleSegmentLight(segmentLight, position, color);
        } else if (std::holds_alternative<ParallelogramLight>(light)) {
            const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
            sampleParallelogramLight(parallelogramLight, position, color);
        }
        vec_position_color.push_back(std::make_tuple(position, color));
    }

    return vec_position_color;
}

void enableSoftShadowActions(glm::vec3& color, const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, int rayDepth, HitInfo hitInfo)
{
    glm::vec3 offset(-0.0001f);
    glm::vec3 intersection_point = ray.origin + ray.direction * ray.t + offset * ray.direction;

    std::vector<std::tuple<glm::vec3, glm::vec3>> samples = sampledLightMultipleTimes(l, 100);

    softShadowsVisualDebug(ray, bvh, samples, features, hitInfo);

        // compute avg color
        if (samples.size() == 0) {
            return;
        }
        for (std::tuple<glm::vec3, glm::vec3> t : samples) {
            glm::vec3 curr_point_on_light = std::get<0>(t);
            glm::vec3 curr_color = std::get<1>(t);

            float light_vec_t = glm::length(curr_point_on_light - intersection_point);
            // if (light_vec_t == 0.0f) {
            //     continue;
            // }
            glm::vec3 light_vec_dir = glm::normalize(curr_point_on_light - intersection_point);

            Ray ray_towards_light { intersection_point, light_vec_dir, light_vec_t };

            bool hit_before = bvh.intersect(ray_towards_light, hitInfo, features);

            if (hit_before) {
                continue;
            } else {
                // color += computeShading(curr_light_pos, curr_color, features, ray, hitInfo);
                color += curr_color;
            }
        }
        color = color / glm::vec3(samples.size());
    }
}
