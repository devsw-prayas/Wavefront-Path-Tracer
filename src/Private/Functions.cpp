#include <Core.h>
#include <cassert>
#include <bit>
#include <cstdint>
#include <Functions.h>

namespace WavefrontPT::Math {
#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
	void enableFtzDaz() {
		unsigned int mxcsr = _mm_getcsr();
		mxcsr |= (1 << 15); // FTZ
		mxcsr |= (1 << 6);  // DAZ
		_mm_setcsr(mxcsr);
	}

	Float32 sanitize(Float32 v_Value) {
		return v_Value == v_Value ? v_Value : 0.f;
	}

	Reg4 sanitize(const Reg4& v_Value) {
		return _mm_and_ps(v_Value, _mm_cmpeq_ps(v_Value, v_Value));
	}

	Reg8 sanitize(const Reg8& v_Value) {
		return _mm256_and_ps(v_Value, _mm256_cmp_ps(v_Value, v_Value, _CMP_EQ_OQ));
	}

	Float32 sqrt(Float32 v_Value) {
		return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(v_Value > 0 ? v_Value : 0)));
	}

	Float32 rSqrt(Float32 v_Value) {
		if (sanitize(v_Value) < 0) return 0.f;
		Reg4 broadcast = _mm_set_ss(sanitize(v_Value));
		Reg4 rsqrt = _mm_rsqrt_ss(broadcast);
		// Peform NR

		Reg4 vDot5 = _mm_set_ss(0.5f), vOneDot5 = _mm_set_ss(1.5f);
		Reg4 inner = _mm_sub_ps(vOneDot5, _mm_mul_ps(vDot5, _mm_mul_ps(broadcast, _mm_mul_ps(rsqrt, rsqrt))));
		return _mm_cvtss_f32(_mm_mul_ps(rsqrt, inner));
	}

	Float32 absFast(Float32 v_Value) {
		return std::bit_cast<Float32>(std::bit_cast<uint32_t>(sanitize(v_Value)) & 0x7FFFFFFF);
	}

	Float32 minFast(Float32 v_A, Float32 v_B) {
		return v_A > v_B ? v_B : v_A;
	}

	Float32 maxFast(Float32 v_A, Float32 v_B) {
		return v_A > v_B ? v_A : v_B;
	}

	Float32 clampFast(Float32 v_Val, Float32 v_Min, Float32 v_Max) {
		assert(v_Max >= v_Min);
		return minFast(sanitize(v_Max), maxFast(sanitize(v_Min), sanitize(v_Val)));
	}

	bool signBitFast(Float32 v_Val) {
		return std::bit_cast<uint32_t>(sanitize(v_Val)) >> 31;
	}

	Reg4 sqrt(const Reg4& r_Val) {
		return _mm_sqrt_ps(_mm_max_ps(sanitize(r_Val), _mm_setzero_ps()));
	}

	Reg4 rSqrt(const Reg4& r_Val) {
		Reg4 rsqrt = _mm_rsqrt_ps(sanitize(r_Val));
		// Peform NR

		Reg4 vDot5 = _mm_set_ss(0.5f), vOneDot5 = _mm_set_ss(1.5f);
		Reg4 inner = _mm_sub_ps(vOneDot5, _mm_mul_ps(vDot5, _mm_mul_ps(r_Val, _mm_mul_ps(rsqrt, rsqrt))));
		return _mm_mul_ps(rsqrt, inner);
	}

	Reg4 absFast(const Reg4& r_Val) {
		return _mm_and_ps(sanitize(r_Val), _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF)));
	}

	Reg4 minFast(const Reg4& r_A, const Reg4& r_B) {
		return _mm_min_ps(sanitize(r_A), sanitize(r_B));
	}

	Reg4 maxFast(const Reg4& r_A, const Reg4& r_B) {
		return _mm_max_ps(sanitize(r_A), sanitize(r_B));
	}

	Reg4 clampFast(const Reg4& r_Val, const Reg4& r_Min, const Reg4& r_Max) {
		assert(!_mm_movemask_ps(_mm_cmpgt_ps(r_Min, r_Max)));
		return _mm_min_ps(sanitize(r_Max), _mm_max_ps(sanitize(r_Min), sanitize(r_Val)));
	}

	Reg4 signBitMask(const Reg4& r_Val) {
		const Reg4 signMask =
			_mm_castsi128_ps(_mm_set1_epi32(0x80000000));
		return _mm_and_ps(sanitize(r_Val), signMask);
	}

	Reg8 sqrt(const Reg8& r_Val) {
		return _mm256_sqrt_ps(
			_mm256_max_ps(sanitize(r_Val), _mm256_setzero_ps())
		);
	}

	Reg8 rSqrt(const Reg8& r_Val) {
		Reg8 x = sanitize(r_Val);
		x = _mm256_max_ps(x, _mm256_setzero_ps());

		Reg8 rsqrt = _mm256_rsqrt_ps(x);

		const Reg8 half = _mm256_set1_ps(0.5f);
		const Reg8 threeHalves = _mm256_set1_ps(1.5f);

		Reg8 inner = _mm256_sub_ps(threeHalves, _mm256_mul_ps(half, _mm256_mul_ps(x, _mm256_mul_ps(rsqrt, rsqrt)))
		);

		return _mm256_mul_ps(rsqrt, inner);
	}

	Reg8 absFast(const Reg8& r_Val) {
		const Reg8 mask =
			_mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF));
		return _mm256_and_ps(sanitize(r_Val), mask);
	}

	Reg8 minFast(const Reg8& r_A, const Reg8& r_B) {
		return _mm256_min_ps(sanitize(r_A), sanitize(r_B));
	}


	Reg8 maxFast(const Reg8& r_A, const Reg8& r_B) {
		return _mm256_max_ps(sanitize(r_A), sanitize(r_B));
	}


	Reg8 clampFast(const Reg8& r_Val, const Reg8& r_Min, const Reg8& r_Max) {
		assert(!_mm256_movemask_ps(_mm256_cmp_ps(r_Min, r_Max, _CMP_GT_OQ)));

		return _mm256_min_ps(
			sanitize(r_Max),
			_mm256_max_ps(sanitize(r_Min), sanitize(r_Val))
		);
	}


	Reg8 signBitMask(const Reg8& r_Val) {
		const Reg8 signMask =
			_mm256_castsi256_ps(_mm256_set1_epi32(0x80000000));
		return _mm256_and_ps(r_Val, signMask);
	}
#endif
}