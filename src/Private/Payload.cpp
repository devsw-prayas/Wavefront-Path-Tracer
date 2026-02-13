#include "Core.h"
#include "Payload.h"

#include <iostream>

#include "IntegratorOps.h"

namespace WavefrontPT::Integrator {
	void evaluateMaterialResponse(const Scene& ro_Scene, Payload& ro_Payload, const Math::HitRecord& ro_Hit, const Materials::Material& ro_Mat) {
		if (Math::maxFast(ro_Mat.m_Emission.X,
						  Math::maxFast(ro_Mat.m_Emission.Y, ro_Mat.m_Emission.Z)) > 0.0f) {
			ro_Payload.m_Radiance = ro_Payload.m_Radiance + ro_Payload.m_Throughput * ro_Mat.m_Emission;

			// Kill the path
			ro_Payload.m_Throughput = Math::Vector3(0.0f);
			return;
		}

		// NEE
		const Geometry::GSphere* p_LightSphere = nullptr;
		Math::Vector3 lightEmission(0.0f);

		for (size_t i = 0; i < ro_Scene.m_SphereCount; ++i) {
			const auto& sphere = ro_Scene.m_Spheres[i];
			const auto& mat = ro_Scene.m_Materials[sphere.m_MaterialID];

			if (Math::maxFast(mat.m_Emission.X,
							  Math::maxFast(mat.m_Emission.Y,
											mat.m_Emission.Z)) > 0.0f) {
				p_LightSphere = &sphere;
				lightEmission = mat.m_Emission;
				break;
			}
		}

		if (!p_LightSphere)
			return; // no light in scene

		Math::FP32 u1 = Integrators::Ops::randomFloat(ro_Payload.m_RngState);
		Math::FP32 u2 = Integrators::Ops::randomFloat(ro_Payload.m_RngState);

		Math::Vector3 sphereDir = Integrators::Ops::sampleUnfiromUnitSphere(u1, u2);
		Math::Point3 lightPoint = p_LightSphere->m_Center + Math::scale(sphereDir, p_LightSphere->m_Radius);
		Math::Vector3 lightNorm = sphereDir;

		Math::Vector3 toLight = lightPoint - ro_Hit.m_HitPoint;

		Math::FP32 dist2 = Math::dot(toLight, toLight);
		Math::FP32 dist = std::sqrt(dist2);

		Math::Vector3 wi = Math::scale(toLight, 1.0f / dist);

		Math::Ray shadowRay(ro_Hit.m_HitPoint + Math::scale(ro_Hit.m_GeometricNormal, Math::kEpsilon), wi);

		Math::HitRecord shadowHit = hitScene(ro_Scene, shadowRay);

		if (!(shadowHit.m_Hit && shadowHit.m_T < dist - Math::kEpsilon)) {
			Math::FP32 cosSurface = Math::dot(ro_Hit.m_GeometricNormal, wi);

			Math::FP32 cosLight = Math::dot(lightNorm, Math::negate(wi));

			if (cosSurface > 0.0f && cosLight > 0.0f) {
				Math::FP32 pdfArea = 1.0f / (4.0f * std::numbers::pi_v<float> *p_LightSphere->m_Radius * p_LightSphere->m_Radius);
				Math::FP32 pdfOmega = pdfArea * dist2 / cosLight;
				Math::Vector3 f = Math::scale(ro_Mat.m_Color, std::numbers::inv_pi_v<float>);
				Math::Vector3 Ld = Math::scale(f * lightEmission, cosSurface / pdfOmega);
				ro_Payload.m_Radiance = ro_Payload.m_Radiance + ro_Payload.m_Throughput * Ld;
			}
		}
		Math::Vector3 n = ro_Hit.m_GeometricNormal;
		Math::Vector3 tangent = Math::absFast(n.X) > 0.1f ? Math::Vector3(0, 1, 0) : Math::Vector3(1, 0, 0);
		tangent = Math::normalize(Math::cross(tangent, n));
		Math::Vector3 bitangent = cross(n, tangent);

		u1 = Integrators::Ops::randomFloat(ro_Payload.m_RngState);
		u2 = Integrators::Ops::randomFloat(ro_Payload.m_RngState);

		Math::Vector3 localDir = Integrators::Ops::sampleCosineHemisphere(u1, u2);
		wi = Math::scale(tangent, localDir.X) + Math::scale(bitangent, localDir.Y) + Math::scale(n,localDir.Z);

		wi = normalize(wi);
		ro_Payload.m_Throughput = ro_Payload.m_Throughput * ro_Mat.m_Color;

		ro_Payload.m_CurrentRay.m_Origin =ro_Hit.m_HitPoint +Math::scale(n, Math::kEpsilon);
		ro_Payload.m_CurrentRay.m_DirectionCosine =wi;
	}
}