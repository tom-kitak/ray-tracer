#include "render.h"
#include "intersect.h"
#include "light.h"
#include "screen.h"
#include "interpolate.h"
#include <framework/trackball.h>
#ifdef NDEBUG
#include <omp.h>
#endif

#include <iostream>

glm::vec3 addTransparencyForPixel(glm::vec3 color, Ray ray, HitInfo hitInfo, const Features& features, const BvhInterface& bvh, const Scene& scene, int rayDepth);

glm::vec3 getFinalColor(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, int rayDepth)
{
    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo, features)) {
        int maxRayDepth = 2;
        glm::vec3 Lo = computeLightContribution(scene, bvh, features, ray, hitInfo);

        if (features.enableRecursive && hitInfo.material.ks != glm::vec3(0, 0, 0) && rayDepth <= maxRayDepth) {
           // Compute the reflection ray and contribute its light if the material is specular and the rayDepth is at most maxRayDepth.
           Ray reflection = computeReflectionRay(ray, hitInfo);
           Lo += hitInfo.material.ks * getFinalColor(scene, bvh, reflection, features, rayDepth + 1);
        }
        
        // Clamp the rgb values between 0 and 1.
        Lo = glm::vec3(std::clamp(Lo.x, 0.0f, 1.0f), std::clamp(Lo.y, 0.0f, 1.0f), std::clamp(Lo.z, 0.0f, 1.0f));

        if (features.extra.enableTransparency) {
            Lo = addTransparencyForPixel(Lo, ray, hitInfo, features, bvh, scene, rayDepth);
        }

        // Draw a debug ray with the color of Lo if the ray hits.
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

glm::vec3 addTransparencyForPixel(glm::vec3 color, Ray ray, HitInfo hitInfo, const Features& features, const BvhInterface& bvh, const Scene& scene, int rayDepth)
{
    if (hitInfo.material.transparency == 1.0f || ray.t == std::numeric_limits<float>::max()) {
        return color;
    }
    glm::vec3 offset(0.001f);
    glm::vec3 point_on_surface = ray.origin + ray.t * ray.direction + ray.direction * offset;

    Ray ray_from_point_on_surface { point_on_surface , ray.direction };
    
    HitInfo hi;
    //background set to default black
    glm::vec3 backgroundAll(0.0f);
    
    if (bvh.intersect(ray_from_point_on_surface, hi, features) && ray_from_point_on_surface.t != std::numeric_limits<float>::max()) {
        //glm::vec3 background_color = computeLightContribution(scene, bvh, features, ray_from_point_on_surface, hi); 
        //backgroundAll = addTransparencyForPixel(background_color, ray_from_point_on_surface, hi, features, bvh, scene);
        backgroundAll = getFinalColor(scene, bvh, ray_from_point_on_surface, features, rayDepth);
    }
    // Visual Debug for transparency:
    // When a ray is shot towards the surface, another ray on the other side is shot and it is colored with the background color.
    drawRay(ray_from_point_on_surface, backgroundAll);

    return glm::vec3(hitInfo.material.transparency) * color + glm::vec3(1 - hitInfo.material.transparency) * backgroundAll;
}