#pragma once
#include "WMath.h"

namespace WavefrontPT::Integrators::Ops {
	using namespace Math;
	Math::Vector3 sampleUnfiromUnitSphere(Math::FP32 v_U1, Math::FP32 v_U2);

	uint32_t xorShift32(uint32_t& v_State);

	inline FP32 randomFloat(uint32_t& state) {
		uint32_t r = xorShift32(state);
		return FP32(r) * (1.0f / 4294967296.0f);
	}
	Vector3 sampleCosineHemisphere(FP32 u1, FP32 u2);
}
