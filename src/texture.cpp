#include "texture.h"
#include "common.h"
#include "config.h"
#include <glm/geometric.hpp>
#include <framework/image.h>

#define PI 3.14159265

glm::vec3 getColor(Image& image, const glm::vec2& texCoord, int level);

glm::vec3 acquireTexel(Image& image, const glm::vec2& texCoord, const Features& features, Ray ray, HitInfo hitInfo)
{
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    if (features.extra.enableMipmapTextureFiltering && hitInfo.m) {
        float footPrint = std::clamp(hitInfo.m * image.width, 1.0f, image.width - 1.0f);
        float maxLevel = 0;
        float count = image.width;
        while (count > 1) {
            count /= 2;
            maxLevel++;
        }
        float level = std::clamp(std::log2(footPrint), 0.0f, maxLevel);
        // Interpolate based on the level
        return (level - floor(level)) * getColor(image, texCoord, ceil(level)) + (1 - level + floor(level)) * getColor(image, texCoord, floor(level));
    }

    int x = round(texCoord[0] * image.width - 0.5);
    int y = round(image.height - texCoord[1] * image.height - 0.5);

    // Repeating pattern
    int x_wrap = x % image.width;
    int y_wrap = y % image.height;
    return image.pixels[y_wrap * image.width + x_wrap];
}

glm::vec3 getColor(Image& image, const glm::vec2& texCoord, int level)
{
    int amount = image.width / std::pow(2, level);
    int offset = 0;
    for (int i = 0; i < level; i++) {
        offset += (image.width / std::pow(2, i)) * (image.height / std::pow(2, i));
    }
    int x = floor(texCoord[0] * amount);
    int y = floor(amount - texCoord[1] * amount);

    // Repeating pattern
    int x_wrap = x % amount;
    int y_wrap = y % amount;
    int index = std::clamp(offset + y_wrap * amount + x_wrap, 0, (int)image.pixels.size() - 1);
    return image.pixels[index];
}


