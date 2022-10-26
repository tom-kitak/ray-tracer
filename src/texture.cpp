#include "texture.h"
#include <framework/image.h>

glm::vec3 acquireTexel(const Image& image, const glm::vec2& texCoord, const Features& features)
{
    // TODO: implement this function.
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    
    //int x = texCoord.x;
    //if (x < 0) {
    //    x = x * (-1);
    //    x = x % image.width;
    //    x = image.width - x;
    //}
    //if (x > image.width) {
    //    x = x % image.width;
    //}
    //int y = texCoord[1];
    //if (y < 0) {
    //    y = y * (-1);
    //    y = y % image.height;
    //    y = image.height - y;
    //}
    //if (y > image.height) {
    //    y = y % image.height;
    //}
    //return image.pixels[y * image.width + x];
    
    int x = round(texCoord[0] * image.width - 0.5);
    int y = round(texCoord[1] * image.height - 0.5);
    return image.pixels[y * image.width + x];
}