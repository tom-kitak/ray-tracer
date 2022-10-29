#include "texture.h"
#include "common.h"
#include <framework/image.h>

glm::vec3 acquireTexel(const Image& image, const glm::vec2& texCoord, const Features& features)
{
    // TODO: implement this function.
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    if (features.extra.enableMipmapTextureFiltering) {

    }

    int x = round(texCoord[0] * image.width - 0.5);
    int y = round(image.height - texCoord[1] * image.height - 0.5);

    // Repeating pattern
    int x_wrap = x % image.width;
    int y_wrap = y % image.height;
    return image.pixels[y_wrap * image.width + x_wrap];
}
