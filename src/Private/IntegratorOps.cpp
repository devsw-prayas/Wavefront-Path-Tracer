#include <Core.h>
#include <IntegratorOps.h>

namespace WavefrontPT::Integrators::Ops {
	using namespace Math;
	Vector3 sampleUnfiromUnitSphere(Math::FP32 v_U1, Math::FP32 v_U2) {
		FP32  z = 1.0f - 2.0f * v_U1;
		FP32 phi = 2.0f * std::numbers::pi_v<float> *v_U2;

		FP32 r = sqrt(maxFast(0.0f, 1.0f - z * z));
		FP32 x = r * cosFP(phi);
		FP32 y = r * sinFP(phi);
		return { x, y, z };
	}

	uint32_t xorShift32(uint32_t& v_State) {
		v_State ^= v_State << 13;
		v_State ^= v_State >> 17;
		v_State ^= v_State << 5;
		return v_State;
	}

	Math::Vector3 sampleCosineHemisphere(
		Math::FP32 u1,
		Math::FP32 u2) {
		Math::FP32 r = std::sqrt(u1);
		Math::FP32 theta = 2.0f * std::numbers::pi_v<float> * u2;

		Math::FP32 x = r * std::cos(theta);
		Math::FP32 y = r * std::sin(theta);
		Math::FP32 z = std::sqrt(1.0f - u1);

		return Math::Vector3(x, y, z); // local space
	}

}
