#include <Core.h>
#include <IntegratorMathCore.h>
#include <Integrators.h>
#include <WMath.h>

#include "FileOutput.h"
#include "RaySphereIntersection.h"

namespace WavefrontPT::Integrator {
	using namespace WavefrontPT::Math;
	void pinholeImageIntegrator() {
		// -------------------------------------------------
		// Image
		// -------------------------------------------------
		constexpr size_t kImageWidth = 1920;
		constexpr size_t kImageHeight = 1080;

		Vector3* framebuffer = new Vector3[kImageWidth * kImageHeight];

		// -------------------------------------------------
		// Camera (pinhole, scalar, no matrices)
		// -------------------------------------------------
		Point3 cameraOrigin(0.0f, 0.0f, 0.0f);

		FP32 aspect = FP32(kImageWidth) / FP32(kImageHeight);

		FP32 viewportHeight = 2.0f;
		FP32 viewportWidth = viewportHeight * aspect;

		FP32 focalLength = 1.0f;

		Vector3 horizontal(viewportWidth, 0.0f, 0.0f);
		Vector3 vertical(0.0f, viewportHeight, 0.0f);

		Point3 lowerLeftCorner =
			cameraOrigin +
			Vector3(-viewportWidth * 0.5f,
					-viewportHeight * 0.5f,
					-focalLength);

		// -------------------------------------------------
		// Scene (single sphere)
		// -------------------------------------------------
		Math::Sphere sphere(
			Point3(0.0f, 0.0f, -3.0f),
			1.0f
		);

		constexpr FP32 epsilon = kEpsilon;

		// -------------------------------------------------
		// Render loop - THIS IS TASK 1.3
		// -------------------------------------------------
		for (size_t y = 0; y < kImageHeight; ++y) {
			for (size_t x = 0; x < kImageWidth; ++x) {
				// Pixel -> normalized screen space
				FP32 u = static_cast<FP32>(x) / static_cast<FP32>(kImageWidth - 1);
				FP32 v = static_cast<FP32>(y) / static_cast<FP32>(kImageHeight - 1);

				// Screen space -> point on image plane
				Point3 pixelPoint =
					lowerLeftCorner +
					scale(horizontal, u) +
					scale(vertical, v);

				// Ray direction
				Vector3 rayDir = normalize(pixelPoint - cameraOrigin);

				Math::Ray ray(cameraOrigin, rayDir);

				// Intersection (Task 1.2 reused)
				FP32 t = intersect(ray, sphere, epsilon);

				Vector3 pixelColor;

				if (t < std::numeric_limits<FP32>::infinity()) {
					// Hit position
					Point3 P = cameraOrigin + scale(rayDir, t);

					// Geometric normal
					Vector3 Ng = normalize(P - sphere.m_Center);

					// Normal -> color ([-1,1] -> [0,1])
					pixelColor = scale(
						Vector3(Ng.X + 1.0f,
								Ng.Y + 1.0f,
								Ng.Z + 1.0f),
						0.5f
					);
				} else {
					// Miss
					pixelColor = Vector3(0.0f, 0.0f, 0.0f);
				}

				framebuffer[y * kImageWidth + x] = pixelColor;
			}
		}

		// -------------------------------------------------
		// Output
		// -------------------------------------------------
		writePPM("task_1_3.ppm", framebuffer, kImageWidth, kImageHeight);

		delete[] framebuffer;
	}
}