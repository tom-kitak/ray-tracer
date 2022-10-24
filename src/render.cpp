#include "render.h"
#include "intersect.h"
#include "light.h"
#include "screen.h"
#include "interpolate.h"
#include <framework/trackball.h>
#ifdef NDEBUG
#include <omp.h>
#endif

void hardShadowVisualDebug(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, HitInfo hitInfo);
void softShadowsVisualDebug(Ray ray, SegmentLight segmentLight, const BvhInterface& bvh, std::vector<std::tuple<glm::vec3, glm::vec3>> vec_position_color, const Features& features, HitInfo hitInfo);
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

            for (std::variant<PointLight, SegmentLight, ParallelogramLight> l : scene.lights) { 
                PointLight point_light = std::get<PointLight>(l);
                glm::vec3 samplePos = point_light.position;
                float color_res = testVisibilityLightSample(samplePos, point_light.color, bvh, features, ray, hitInfo);
                   
                if (color_res == 0.0f) {
                    Lo = glm::vec3(0.0f);
                }
                
            }
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

    for (std::variant<PointLight, SegmentLight, ParallelogramLight> l : scene.lights) {

        PointLight point_light = std::get<PointLight>(l);
        glm::vec3 samplePos = point_light.position;

        float shadow_vec_t = glm::length(samplePos - intersection_point);
        if (shadow_vec_t == 0.0f) {
            drawRay(Ray { intersection_point, glm::vec3(0.0f), 0}, glm::vec3(1.0f));
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

//void normalInterpolationVisualDebug(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, HitInfo hitInfo)
//{
//    glm::vec3 intersection_point = ray.origin + ray.direction * ray.t * ray.direction;
//
//    if (bvh.intersect(ray, hitInfo, features)) {
//        if (!features.enableAccelStructure) {
//            bool hit = false;
//            for (const auto& mesh : scene.meshes) {
//                for (const auto& tri : mesh.triangles) {
//                    const auto v0 = mesh.vertices[tri[0]];
//                    const auto v1 = mesh.vertices[tri[1]];
//                    const auto v2 = mesh.vertices[tri[2]];
//                    if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
//                        glm::vec3 barycentricCoordinates = computeBarycentricCoord(v0.position, v1.position, v2.position, intersection_point);
//                        glm::vec3 interpolatedNormal = interpolateNormal(v0.normal, v1.normal, v2.normal, barycentricCoordinates);
//
//                        // Interpolated normal will be green
//                        Ray interpolatedNormalRay = { intersection_point, interpolatedNormal, 10.0f };
//                        drawRay(interpolatedNormalRay, glm::vec3(0.0f, 1.0f, 0.0f));
//                    }
//                }
//            }
//        }
//    }
//}