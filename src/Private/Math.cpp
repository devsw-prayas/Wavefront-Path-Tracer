#include <Core.h>
#include <Math.h>

namespace WavefrontPT::Math {
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
}