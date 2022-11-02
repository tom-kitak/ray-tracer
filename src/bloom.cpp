#include "render.h"
#include "screen.h"
#include "common.h"
#include "bloom.h"

// Sources bloom filter: computer graphics L2 slides

void bloom(Screen& screen, const Features& features, glm::ivec2 windowRes, float t, int distance) 
{	
	// Size of the window
    auto wx = windowRes.x;
    auto wy = windowRes.y;

    // Vector that will store the filtered pixels
    std::vector<glm::vec3> filteredPixels;
	
	// Check if value < threshold, if so set to 0, then each (updated) pixel is added to the vector
	for (int x = 0; x < wx; x++) {
		for (int y = 0; y < wy; y++) {
                glm::vec3 pixel = screen.getPixel(x, y);

				if (pixel.x < t) {
                    pixel.x = 0;
				}
                if (pixel.y < t) {
                    pixel.y = 0;
                }
                if (pixel.z < t) {
                    pixel.z = 0;
                }

				filteredPixels.push_back(pixel);
		}
	}
	
    for (int x = 0; x < wx; x++) {
        for (int y = 0; y < wy; y++) {
            // Define filter size
            int y0 = glm::min(distance, y);
            int y1 = glm::min(distance, wy - 1 - y);
            int x0 = glm::min(distance, y);
            int x1 = glm::min(distance, wx - 1 - x);

            // Calculate average
            glm::vec3 v;
            int n = 0;
            for (int i = -x0; i < x1; i++) {
                int abs = std::abs(i);
                for (int j = -y0; j < y1; j++) {
                    n++;
                    int index = (x + i) * wx + y + j;
                    v = v + filteredPixels[index];
                }
            }

            v /= n;

            // Visual debug
            
            // If enabled, show only the blurred image
            if (features.extra.enableBloomEffect) {
                screen.setPixel(x, y, v);
            }

            // If not enabled, show the original image + the blurred image
            if (!features.extra.enableBloomEffect) {
                auto pixel = v + screen.getPixel(x, y);
                screen.setPixel(x, y, pixel);
            }
        }
    }
}