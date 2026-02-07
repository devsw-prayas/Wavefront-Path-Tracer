#pragma once
#include "WMath.h"

namespace WavefrontPT::Materials {

	struct Material final {
		Math::Vector3 m_Color;
		Math::Vector3 m_Emission;
		Math::FP32 m_Metalness;
		Math::FP32 m_Roughness;

		Material() = default;

		Material(const Math::Vector3& ro_Color, const Math::Vector3& ro_Emission, Math::FP32 v_Met, Math::FP32 v_Rough) :
			m_Color(ro_Color), m_Emission(ro_Emission), m_Metalness(v_Met), m_Roughness(v_Rough) {}
		
		Material(const Material&) = default;
		Material& operator=(const Material&) = default;

		Material(Material&&) noexcept = default;
		Material& operator=(Material&&) noexcept = default;

		~Material() = default;
	};
}
