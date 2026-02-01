#include <Core.h>
#include <WMath.h>
#include <Functions.h>

namespace WavefrontPT::Math {

	// ----------------------------------------------------------
	// Scalar bound Ops
	// ----------------------------------------------------------

	FP32 length(const Vector3& ro_Op) noexcept {
		return sqrt(dot(ro_Op, ro_Op));
	}

	Vector3 normalize(const Vector3& ro_Vec) noexcept {
		FP32 l = lengthSq(ro_Vec);
		if (l <= kEpsilonSq) return { 0.0f, 0.0f, 0.0f };
		FP32 invLen = 1.0f / std::sqrt(l);
		return scale(ro_Vec, invLen);
	}

	Vector3 negate(const Vector3& ro_Vec) noexcept {
		return { -ro_Vec.X, -ro_Vec.Y, -ro_Vec.Z };
	}

	Vector3 reflect(const Vector3& ro_Vec, const Vector3& ro_Ref) noexcept {
		FP32 vn = dot(ro_Vec, ro_Ref);
		return ro_Vec - scale(ro_Ref, 2.0f * vn);
	}

	Vector3 faceForward(const Vector3& ro_Vec, const Vector3& ro_N) noexcept {
		return dot(ro_N, ro_Vec) < 0.0f ? ro_N : negate(ro_N);
	}

	//---------------------------------------------------------------
	// Vectorized Operations
	//---------------------------------------------------------------				

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
	Stripe3 operator+(const Stripe3& ro_A, const Stripe3& ro_B) {
		return { _mm256_add_ps(ro_A.X, ro_B.X),
			_mm256_add_ps(ro_A.Y, ro_B.Y),
			_mm256_add_ps(ro_A.Z, ro_B.Z) };
	}

	Stripe3 operator-(const Stripe3& ro_A, const Stripe3& ro_B) {
		return { _mm256_sub_ps(ro_A.X, ro_B.X),
			_mm256_sub_ps(ro_A.Y, ro_B.Y),
			_mm256_sub_ps(ro_A.Z, ro_B.Z) };
	}

	Stripe3 operator*(const Stripe3& ro_A, const Stripe3& ro_B) {
		return { _mm256_mul_ps(ro_A.X, ro_B.X),
			_mm256_mul_ps(ro_A.Y, ro_B.Y),
			_mm256_mul_ps(ro_A.Z, ro_B.Z) };
	}

	Stripe3 operator/(const Stripe3& ro_A, const Stripe3& ro_B) {
		return { _mm256_div_ps(ro_A.X, ro_B.X),
			_mm256_div_ps(ro_A.Y, ro_B.Y),
			_mm256_div_ps(ro_A.Z, ro_B.Z) };
	}

	Stripe3 scale(const Stripe3& ro_Stripe, RegFP32 v_Scalar) {
		return {
			_mm256_mul_ps(ro_Stripe.X, v_Scalar),
			_mm256_mul_ps(ro_Stripe.Y, v_Scalar),
			_mm256_mul_ps(ro_Stripe.Z, v_Scalar) };
	}

	RegFP32 dot(const Stripe3& ro_A, const Stripe3& ro_B) {
		return { _mm256_fmadd_ps(ro_A.X, ro_B.X,
		_mm256_fmadd_ps(ro_A.Y, ro_B.Y,
		_mm256_fmadd_ps(ro_A.Z, ro_B.Z,
		_mm256_setzero_ps()))) };
	}

	Stripe3 cross(const Stripe3& ro_A, const Stripe3& ro_B) {
		return {
			_mm256_fmsub_ps(ro_A.Y, ro_B.Z, _mm256_mul_ps(ro_A.Z, ro_B.Y)),
			_mm256_fmsub_ps(ro_A.Z, ro_B.X, _mm256_mul_ps(ro_A.X, ro_B.Z)),
			_mm256_fmsub_ps(ro_A.X, ro_B.Y, _mm256_mul_ps(ro_A.Y, ro_B.X))
		};
	}

	RegFP32 lengthSq(const Stripe3& ro_Vec) {
		return dot(ro_Vec, ro_Vec);
	}

	RegFP32 length(const Stripe3& ro_Stripe) {
		return sqrt(dot(ro_Stripe, ro_Stripe));
	}

	Stripe3 negate(const Stripe3& ro_Stripe) {
		return scale(ro_Stripe, _mm256_set1_ps(-1.0f));
	}

	Stripe3 normalize(const Stripe3& ro_Stripe) {
		RegFP32 l = lengthSq(ro_Stripe);
		// valid lanes: lenSq > 0
		RegFP32 validMask = _mm256_cmp_ps(l, _mm256_setzero_ps(), _CMP_GT_OQ);
		RegFP32 invLen = rSqrt(l);
		invLen = _mm256_and_ps(invLen, validMask);
		return scale(ro_Stripe, invLen);
	}

	Stripe3 reflect(const Stripe3& ro_Stripe, const Stripe3& ro_Ref) {
		RegFP32 vn = dot(ro_Ref, ro_Stripe);
		return ro_Stripe - scale(ro_Ref, _mm256_mul_ps(vn, _mm256_set1_ps(2)));
	}

	Stripe3 faceForward(const Stripe3& ro_Stripe, const Stripe3& ro_N) {
		RegFP32 d = dot(ro_N, ro_Stripe);

		RegFP32 mask = _mm256_cmp_ps(
			d,
			_mm256_setzero_ps(),
			_CMP_LT_OQ
		);

		Stripe3 negN = {
			_mm256_sub_ps(_mm256_setzero_ps(), ro_N.X),
			_mm256_sub_ps(_mm256_setzero_ps(), ro_N.Y),
			_mm256_sub_ps(_mm256_setzero_ps(), ro_N.Z)
		};

		return {
			_mm256_blendv_ps(negN.X, ro_N.X, mask),
			_mm256_blendv_ps(negN.Y, ro_N.Y, mask),
			_mm256_blendv_ps(negN.Z, ro_N.Z, mask)
		};
	}

#endif
}