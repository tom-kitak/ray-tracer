#include "render.h"
#include "intersect.h"
#include "light.h"
#include "screen.h"
#include <iostream>
#include <framework/trackball.h>
#ifdef NDEBUG
#include <omp.h>
#endif

void hardShadowVisualDebug(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, HitInfo hitInfo);
void segmentLightVisualDebug(Ray ray, SegmentLight segmentLight, const BvhInterface& bvh, std::vector<std::tuple<glm::vec3, glm::vec3>> vec_position_color, const Features& features, HitInfo hitInfo);
void enableSoftShadowActions(glm::vec3& color, const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, int rayDepth, HitInfo hitInfo);
std::vector<std::tuple<glm::vec3, glm::vec3>> sampledLightMultipleTimes(std::variant<PointLight, SegmentLight, ParallelogramLight> light, int n);

    glm::vec3 getFinalColor(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, int rayDepth)
{
    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo, features)) {
        int maxRayDepth = 2;
        glm::vec3 Lo = computeLightContribution(scene, bvh, features, ray, hitInfo);

        if (features.enableRecursive && hitInfo.material.ks != glm::vec3(0, 0, 0) && rayDepth <= maxRayDepth) {
            Ray reflection = computeReflectionRay(ray, hitInfo);
            Lo += getFinalColor(scene, bvh, reflection, features, rayDepth + 1);
        }
        
        Lo = glm::vec3(std::clamp(Lo.x, 0.0f, 1.0f), std::clamp(Lo.y, 0.0f, 1.0f), std::clamp(Lo.z, 0.0f, 1.0f));


        //Tom Kitak additions enableHardShadow START
        if (features.enableHardShadow) {
            
            hardShadowVisualDebug(scene, bvh, ray, features, hitInfo);
            
            for (const auto& l : scene.lights) {
                if (std::holds_alternative<PointLight>(l)) {
                    PointLight point_light = std::get<PointLight>(l);
                    glm::vec3 samplePos = point_light.position;
                    float color_res = testVisibilityLightSample(samplePos, point_light.color, bvh, features, ray, hitInfo);

                    if (color_res == 0.0f) {
                        Lo = glm::vec3(0.0f);
                    }
                }
            }
        }

        if (features.enableSoftShadow) {
            glm::vec3 color(0.0f);
            enableSoftShadowActions(color, scene, bvh, ray, features, rayDepth, hitInfo);
  
            Lo = color;
        }
        //Tom Kitak additions enableHardShadow END

        Lo = glm::vec3(std::clamp(Lo.x, 0.0f, 1.0f), std::clamp(Lo.y, 0.0f, 1.0f), std::clamp(Lo.z, 0.0f, 1.0f));
        // Draw a white debug ray if the ray hits.
        drawRay(ray, Lo);

        // Set the color of the pixel to white if the ray hits.
        return Lo;
    } else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}

void renderRayTracing(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features)
{
    glm::ivec2 windowResolution = screen.resolution();
    // Enable multi threading in Release mode
#ifdef NDEBUG
#pragma omp parallel for schedule(guided)
#endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos {
                float(x) / float(windowResolution.x) * 2.0f - 1.0f,
                float(y) / float(windowResolution.y) * 2.0f - 1.0f
            };
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay, features));
        }
    }
}

void hardShadowVisualDebug(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, HitInfo hitInfo)
{
    glm::vec3 offset(-0.00001f);
    glm::vec3 intersection_point = ray.origin + ray.direction * ray.t + offset * ray.direction;

    for (const auto& l : scene.lights) {
        if (std::holds_alternative<PointLight>(l)) {
            PointLight point_light = std::get<PointLight>(l);
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

    }
}

void segmentLightVisualDebug(Ray ray, const BvhInterface& bvh, std::vector<std::tuple<glm::vec3, glm::vec3>> vec_position_color, const Features& features, HitInfo hitInfo)
{
    glm::vec3 offset(-0.0001f);
    glm::vec3 intersection_point = ray.origin + ray.direction * ray.t + offset * ray.direction;

    for (std::tuple<glm::vec3, glm::vec3> t : vec_position_color) {
        glm::vec3 point_on_light = std::get<0>(t);
        glm::vec3 color = std::get<1>(t);

        float light_vec_t = glm::length(point_on_light - intersection_point);
        if (light_vec_t == 0.0f) {
            drawRay(Ray { intersection_point, glm::vec3(0.0f), 0 }, glm::vec3(1.0f));
            continue;
        }
        glm::vec3 light_vec_dir = glm::normalize(point_on_light - intersection_point);

        Ray ray_towards_light { intersection_point, light_vec_dir, light_vec_t };

        bool hit_before = bvh.intersect(ray_towards_light, hitInfo, features);

        if (hit_before) {
            drawRay(ray_towards_light, glm::vec3(1.0f, 0.0f, 0.0f));
        } else {
            drawRay(ray_towards_light, color);
        }
    }
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

//std::vector<std::tuple<glm::vec3, glm::vec3>> sampledSegmentLightMultipleTimes(SegmentLight segmentLight, int n)
//{
//    std::vector<std::tuple<glm::vec3, glm::vec3>> vec_position_color;
//    for (int i = 0; i < n; i++) {
//        glm::vec3 position;
//        glm::vec3 color;
//        sampleSegmentLight(segmentLight, position, color);
//        vec_position_color.push_back(std::make_tuple(position, color));
//    }
//
//    return vec_position_color;
//}

void enableSoftShadowActions(glm::vec3& color, const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, int rayDepth, HitInfo hitInfo)
{
    glm::vec3 offset(-0.0001f);
    glm::vec3 intersection_point = ray.origin + ray.direction * ray.t + offset * ray.direction;

    for (const auto& l : scene.lights) {
        if (std::holds_alternative<SegmentLight>(l)) {

            const SegmentLight segmentLight = std::get<SegmentLight>(l);
            std::vector<std::tuple<glm::vec3, glm::vec3>> samples = sampledLightMultipleTimes(segmentLight, 100);

            segmentLightVisualDebug(ray, bvh, samples, features, hitInfo);
           
            //compute avg color
            if (samples.size() == 0) {
                return;
            }
            for (std::tuple<glm::vec3, glm::vec3> t : samples) {
                glm::vec3 curr_point_on_light = std::get<0>(t);
                glm::vec3 curr_color = std::get<1>(t);

                float light_vec_t = glm::length(curr_point_on_light - intersection_point);
                if (light_vec_t == 0.0f) {
                    continue;
                }
                glm::vec3 light_vec_dir = glm::normalize(curr_point_on_light - intersection_point);

                Ray ray_towards_light { intersection_point, light_vec_dir, light_vec_t };

                bool hit_before = bvh.intersect(ray_towards_light, hitInfo, features);

                if (hit_before) {
                    continue;
                } else {
                    //color += computeShading(curr_light_pos, curr_color, features, ray, hitInfo);
                    color += curr_color;
                }
            }
            color = color / glm::vec3(samples.size());

        } else if (std::holds_alternative<ParallelogramLight>(l)) {
            
            const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(l);
            std::vector<std::tuple<glm::vec3, glm::vec3>> samples = sampledLightMultipleTimes(parallelogramLight, 100);

            segmentLightVisualDebug(ray, bvh, samples, features, hitInfo);
        }
    }
}