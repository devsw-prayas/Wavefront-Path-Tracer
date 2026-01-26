#pragma once
#include <Core.h>

namespace WavefrontPT::Math {
#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
#include <immintrin.h>

	using Reg8 = __m256;
	using Reg4 = __m128;
	using Float32 = float;

	// Warning! Call this during thread creation or engine startup
	// to prevent denormals from leaking into fp math
	void enableFtzDaz();

	// sanitize: clears NaNs to zero; does not clamp infinities
	Float32 sanitize(Float32 v_Value);
	// sanitize: clears NaNs to zero; does not clamp infinities
	Reg4 sanitize(const Reg4& v_Value);
	// sanitize: clears NaNs to zero; does not clamp infinities
	Reg8 sanitize(const Reg8& v_Value);

	Float32 sqrt(Float32 v_Value);
	Float32 rSqrt(Float32 v_Value);     // rsqrt + 1 NR

	Float32 absFast(Float32 v_Value);
	Float32 minFast(Float32 v_A, Float32 v_B);
	Float32 maxFast(Float32 v_A, Float32 v_B);
	Float32 clampFast(Float32 v_Val, Float32 v_Min, Float32 v_Max);
	bool    signBitFast(Float32 v_Val);

	Reg4 sqrt(const Reg4& r_Val);
	Reg4 rSqrt(const Reg4& r_Val);      // rsqrt + 1 NR

	Reg4 absFast(const Reg4& r_Val);
	Reg4 minFast(const Reg4& r_A, const Reg4& r_B);
	Reg4 maxFast(const Reg4& r_A, const Reg4& r_B);
	Reg4 clampFast(const Reg4& r_Val, const Reg4& r_Min, const Reg4& r_Max);
	Reg4 signBitMask(const Reg4& r_Val);

	Reg8 sqrt(const Reg8& r_Val);
	Reg8 rSqrt(const Reg8& r_Val);      // rsqrt + 1 NR

	Reg8 absFast(const Reg8& r_Val);
	Reg8 minFast(const Reg8& r_A, const Reg8& r_B);
	Reg8 maxFast(const Reg8& r_A, const Reg8& r_B);
	Reg8 clampFast(const Reg8& r_Val, const Reg8& r_Min, const Reg8& r_Max);
	Reg8 signBitMask(const Reg8& r_Val);
#endif
}
