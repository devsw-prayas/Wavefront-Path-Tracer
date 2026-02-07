#pragma once
#include "IntegratorMathCore.h"
#include "Material.h"
#include "WMath.h"

namespace WavefrontPT::Integrator {
	struct Payload final {
		Math::Vector3 m_Radiance;
		Math::Vector3 m_Throughput;
		Math::Ray m_CurrentRay;

		explicit Payload(const Math::Ray& ro_Ray)
			: m_Radiance(0.0f, 0.0f, 0.0f),
			m_Throughput(1.0f, 1.0f, 1.0f),
			m_CurrentRay(ro_Ray) {}

		Payload(const Payload&) = default;
		Payload& operator=(const Payload&) = default;
		Payload(Payload&&) noexcept = default;
		Payload& operator=(Payload&&) noexcept = default;

		~Payload() = default;
	};												  

	void evaluateMaterialResponse(Payload& ro_Payload,const Math::HitRecord& ro_Hit,const Materials::Material& ro_Mat);
}
