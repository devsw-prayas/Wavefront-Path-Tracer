#include <Core.h>
#include <IntegratorMathCore.h>
#include <Integrators.h>
#include <iostream>
#include <WMath.h>

#include "FileOutput.h"
#include "IntegratorOps.h"
#include "Payload.h"
#include "Scene.h"

namespace WavefrontPT::Integrator {
	using namespace WavefrontPT::Math;

	static Vector3 traceRay(const Scene& ro_Scene, const Math::Ray& ro_Ray, int v_MaxBounce, int v_Seed) {
		Payload payload(ro_Ray);
		payload.m_RngState = v_Seed;
		for (int bounce = 0; bounce < v_MaxBounce; bounce++) {
			Math::HitRecord hit = hitScene(ro_Scene, payload.m_CurrentRay);
			if (!hit.m_Hit) {
				payload.m_Radiance = payload.m_Radiance + payload.m_Throughput * Vector3(.1f, .1f, .1f);
				break;
			}
			const Materials::Material& mat = ro_Scene.m_Materials[hit.m_MatID];
			evaluateMaterialResponse(ro_Scene, payload, hit, mat);
			if (maxFast(payload.m_Throughput.X,
						maxFast(payload.m_Throughput.Y, payload.m_Throughput.Z)) < kEpsilon)
				break;
		}
		return payload.m_Radiance;
	}

	static void renderRows(
		const Scene& scene,
		Vector3* framebuffer,
		size_t yStart,
		size_t yEnd,
		size_t width,
		size_t height,
		int maxBounces,
		const Point3& cameraOrigin,
		const Point3& lowerLeftCorner,
		const Vector3& horizontal,
		const Vector3& vertical) {
		constexpr int kSamplesPerPixel = 128;

		for (size_t y = yStart; y < yEnd; ++y) {
			for (size_t x = 0; x < width; ++x) {
				Vector3 accumulated(0.0f);

				for (int s = 0; s < kSamplesPerPixel; ++s) {
					uint32_t seed = (x + y * width) * 9781u + s * 6271u + 1u;
					uint32_t jitterSeed = seed;

					FP32 u = (FP32(x) + Integrators::Ops::randomFloat(jitterSeed)) / FP32(width);
					FP32 v = (FP32(y) + Integrators::Ops::randomFloat(jitterSeed)) / FP32(height);

					Point3 pixelPoint = lowerLeftCorner + scale(horizontal, u) + scale(vertical, v);
					Vector3 rayDir = normalize(pixelPoint - cameraOrigin);

					Math::Ray cameraRay(cameraOrigin, rayDir);

					Vector3 radiance = traceRay(scene, cameraRay, maxBounces, seed);
					accumulated = accumulated + radiance;
				}
				framebuffer[y * width + x] = scale(accumulated, 1.0f / FP32(kSamplesPerPixel));
			}
		}
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
				Vector3(1.f, 1.f, 1.f),   // color (irrelevant for emission)
				Vector3(18.0f, 15.f, 2.0f), // emission
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

		const auto& time = std::chrono::steady_clock::now();
		const int maxBounces = 8	;

		const unsigned int threadCount =
			std::thread::hardware_concurrency();

		std::vector<std::thread> workers;

		size_t rowsPerThread =
			kImageHeight / threadCount;

		auto startTime = std::chrono::steady_clock::now();

		for (unsigned int t = 0; t < threadCount; ++t) {
			size_t yStart = t * rowsPerThread;
			size_t yEnd =
				(t == threadCount - 1)
				? kImageHeight
				: yStart + rowsPerThread;

			workers.emplace_back(
				renderRows,
				std::cref(scene),
				framebuffer,
				yStart,
				yEnd,
				kImageWidth,
				kImageHeight,
				maxBounces,
				std::cref(cameraOrigin),
				std::cref(lowerLeftCorner),
				std::cref(horizontal),
				std::cref(vertical)
			);
		}

		for (auto& w : workers)
			w.join();

		auto endTime = std::chrono::steady_clock::now();
		std::cout << "Time: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(
				endTime - startTime).count()
			<< " ms\n";

		const auto& end = std::chrono::steady_clock::now() - time;
		std::cout << end << "\n";

		writePPM("MultithreadedPT.ppm", framebuffer, kImageWidth, kImageHeight);
		delete[] framebuffer;
	}
}