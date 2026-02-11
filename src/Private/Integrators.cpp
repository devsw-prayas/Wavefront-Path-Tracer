#include <Core.h>
#include <IntegratorMathCore.h>
#include <Integrators.h>
#include <iostream>
#include <WMath.h>

#include "FileOutput.h"
#include "Payload.h"
#include "Scene.h"

namespace WavefrontPT::Integrator {
	using namespace WavefrontPT::Math;

	static Vector3 traceRay(const Scene& ro_Scene, const Math::Ray& ro_Ray, int v_MaxBounce) {
		Payload payload(ro_Ray);

		for (int bounce = 0; bounce < v_MaxBounce; bounce++) {
			Math::HitRecord hit = hitScene(ro_Scene, payload.m_CurrentRay);
			if (!hit.m_Hit) {
				payload.m_Radiance = payload.m_Radiance + payload.m_Throughput * Vector3(.4f, .4f, .4f);
				break;
			}
			const Materials::Material& mat = ro_Scene.m_Materials[hit.m_MatID];
			evaluateMaterialResponse(payload, hit, mat);
			if (maxFast(payload.m_Throughput.X,
						maxFast(payload.m_Throughput.Y, payload.m_Throughput.Z)) < kEpsilon)
				break;
		}
		return payload.m_Radiance;
	}

	void basicShadingIntegrator() {
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

		Scene scene;
		Math::MaterialID lightMat = registerMaterial(
			scene,
			Materials::Material(
				Vector3(1.0f, 1.0f, 1.0f),   // color (irrelevant for emission)
				Vector3(100.0f, 100.0f, 100.0f), // emission
				0.0f,                        // metalness
				0.0f                         // roughness
			)
		);

		Math::MaterialID groundMat = registerMaterial(
			scene,
			Materials::Material(
				Vector3(0.8f, 0.8f, 0.8f),   // diffuse reflectance
				Vector3(0.0f, 0.0f, 0.0f),   // no emission
				.4f,                        // dielectric
				.8f                         // fully diffuse
			)
		);

		Math::MaterialID metalMat = registerMaterial(
			scene,
			Materials::Material(
				Vector3(0.9f, 0.9f, 0.9f),
				Vector3(0.0f, 0.0f, 0.0f),
				1.0f,    // metal
				0.15f    // slight roughness
			)
		);

		Math::MaterialID redMat = registerMaterial(
			scene,
			Materials::Material(
				Vector3(0.9f, 0.2f, 0.2f),
				Vector3(0.0f, 0.0f, 0.0f),
				0.0f,
				1.0f
			)
		);

		addPlane(
			scene,
			Geometry::GPlane(
				Point3(0.0f, -1.0f, -5.0f),   // center
				Vector3(0.0f, 1.0f, 0.0f),  // normal (up)
				Vector3(1.0f, 0.0f, 0.0f),  // tangent
				Vector3(0.0f, 0.0f, 1.0f),  // bitangent
				20.0f,                        // half width
				20.0f,                        // half breadth
				groundMat,                     // material ID
				scene.m_PlaneCount
			)
		);

		addSphere(
			scene,
			Geometry::GSphere(
				Point3(0.0f, 3.0f, -6.0f),
				0.75f,
				lightMat,
				scene.m_SphereCount
			)
		);

		addSphere(
			scene,
			Geometry::GSphere(
				Point3(0.0f, -0.25f, -4.0f),
				0.75f,
				metalMat,
				scene.m_SphereCount
			)
		);

		addSphere(
			scene,
			Geometry::GSphere(
				Point3(-2.0f, -0.25f, -5.0f),
				0.75f,
				redMat,
				scene.m_SphereCount
			)
		);

		addSphere(
			scene,
			Geometry::GSphere(
				Point3(2.0f, -0.25f, -5.5f),
				0.75f,
				groundMat,
				scene.m_SphereCount
			)
		);

		const int maxBounces = 20;

		for (size_t y = 0; y < kImageHeight; ++y) {
			for (size_t x = 0; x < kImageWidth; ++x) {
				// -----------------------------------------
				// Pixel -> normalized screen space
				// -----------------------------------------
				FP32 u = (FP32(x) + 0.5f) / FP32(kImageWidth);
				FP32 v = (FP32(y) + 0.5f) / FP32(kImageHeight);

				// -----------------------------------------
				// Screen space -> image plane
				// -----------------------------------------
				Point3 pixelPoint =
					lowerLeftCorner +
					scale(horizontal, u) +
					scale(vertical, v);

				// -----------------------------------------
				// Camera ray
				// -----------------------------------------
				Vector3 rayDir = normalize(pixelPoint - cameraOrigin);
				Math::Ray cameraRay(cameraOrigin, rayDir);

				// -----------------------------------------
				// Trace
				// -----------------------------------------
				Vector3 radiance =
					traceRay(scene, cameraRay, maxBounces);

				// -----------------------------------------
				// Store (linear, unclamped)
				// -----------------------------------------
				framebuffer[y * kImageWidth + x] = radiance;
			}
			std::cout << "pixel " << y  << "\n";
		}

		writePPM("EmissivePathTracing.ppm", framebuffer, kImageWidth, kImageHeight);
		delete[] framebuffer;
	}
}