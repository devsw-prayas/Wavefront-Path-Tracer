#pragma once

namespace WavefrontPT::Math::Transcendentals {
	constexpr float PI_2_HI = 1.57079625129699707031f;			
	constexpr float PI_2_LO = 7.54978941586159635335e-08f;
	constexpr float INV_PI_2 = 0.63661977236758134308f;

	struct RangeReduction final {
		float radians;
		int quadrant;
	};

	inline RangeReduction reduce(double x) {
		double kf = x * INV_PI_2;
		int k = static_cast<int>(kf >= 0 ? kf + 0.5f : kf - 0.5f);
		double r = x;
		r -= static_cast<double>(k) * PI_2_HI;
		r -= static_cast<double>(k) * PI_2_LO;

		RangeReduction out;
		out.radians = static_cast<float>(r);
		out.quadrant = k & 3;
		return out;
	}

	// 9th degree polynomial for sine

	constexpr float SIN_C3 = -1.6666654611e-1f;
	constexpr float SIN_C5 = 8.3321608736e-3f;
	constexpr float SIN_C7 = -1.9515295891e-4f;
	constexpr float SIN_C9 = 2.5925366036e-6f;

	// 8th degree polynomial for cos

	constexpr float COS_C2 = -0.4999999973f;
	constexpr float COS_C4 = 0.0416666233f;
	constexpr float COS_C6 = -0.0013886763f;
	constexpr float COS_C8 = 0.000024390448f;

	std::pair<float,float> quadrantReduction(std::pair<float, float> v_SC, int v_Quad);

	std::pair<float, float> sincos(float v_Rad);

	inline float sin(float v_Rad) {
		float s{}, c{};
		s = sincos(v_Rad).first;
		return s;
	}

	inline float cos(float v_Rad) {
		float s{}, c{};
		c = sincos(v_Rad).second;
		return 	c;
	}


}
