#include "Core.h"
#include "Payload.h"

#include <iostream>

namespace WavefrontPT::Integrator {
	void evaluateMaterialResponse(Payload& ro_Payload, const Math::HitRecord& ro_Hit, const Materials::Material& ro_Mat) {
		if (Math::maxFast(ro_Mat.m_Emission.X,
						  Math::maxFast(ro_Mat.m_Emission.Y, ro_Mat.m_Emission.Z)) > 0.0f) {
			ro_Payload.m_Radiance = ro_Payload.m_Radiance + ro_Payload.m_Throughput * ro_Mat.m_Emission;

			// Kill the path
			ro_Payload.m_Throughput = Math::Vector3(0.0f);
			return;
		}
		ro_Payload.m_Radiance = ro_Payload.m_Radiance + ro_Payload.m_Throughput * ro_Mat.m_Emission;
		Math::Vector3 wo = Math::negate(ro_Payload.m_CurrentRay.m_DirectionCosine);
		Math::Vector3 n = ro_Hit.m_GeometricNormal;
		Math::Vector3 wSpec = wo - Math::scale(n, 2.0f * dot(wo, n));
		Math::Vector3 wDiff = n;
		Math::Vector3 wBase = scale(wDiff, 1.0f - ro_Mat.m_Metalness) + scale(wSpec, ro_Mat.m_Metalness);
		Math::Vector3 wi = Math::normalize(scale(wBase, 1.0f - ro_Mat.m_Roughness) + Math::scale(n, ro_Mat.m_Roughness));
		ro_Payload.m_Throughput = ro_Payload.m_Throughput * ro_Mat.m_Color;
		ro_Payload.m_CurrentRay.m_Origin = ro_Hit.m_HitPoint + Math::scale(n, Math::kEpsilon);
		ro_Payload.m_CurrentRay.m_DirectionCosine = wi;
	}
}
