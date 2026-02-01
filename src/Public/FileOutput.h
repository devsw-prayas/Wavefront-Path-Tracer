#pragma once
#include <Core.h>

#include "IntegratorMathCore.h"
#include "WMath.h"

inline void writePPM(
    const char* filename,
    const WavefrontPT::Math::Vector3* framebuffer,
    int width,
    int height
) {
    std::ofstream file(filename, std::ios::out);
    file << "P3\n" << width << " " << height << "\n255\n";

    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const Vector3& c = framebuffer[y * width + x];

            int r = int(255.99f * c.X);
            int g = int(255.99f * c.Y);
            int b = int(255.99f * c.Z);

            file << r << " " << g << " " << b << "\n";
        }
    }
}
