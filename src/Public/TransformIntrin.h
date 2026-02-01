#pragma once
#include <Transform.h>

namespace WavefrontPT::Math {
#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
	Stripe3 applyPoint(const Transform& ro_M, const Stripe3& ro_P);
	Stripe3 applyVector(const Transform& ro_M, const Stripe3& ro_V);
	Stripe3 applyNormal(const Transform& ro_M, const Stripe3& ro_N);
#endif

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
	Stripe3 transformNormal(const Mat3f& ro_M, const Stripe3& ro_N);
	Stripe3 transformVector(const Mat3f& ro_M, const Stripe3& ro_V);
#endif

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
	Stripe3 transformPoint(const Mat4f& ro_M, const Stripe3& ro_P);
	Stripe3 transformVector(const Mat4f& ro_M, const Stripe3& ro_V);
	Stripe3 transformNormal(const Mat4f& ro_M, const Stripe3& ro_N);
#endif
}
