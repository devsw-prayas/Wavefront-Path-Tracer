#include <Core.h>
#include <TransformIntrin.h>

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
namespace WavefrontPT::Math {
	Stripe3 applyPoint(const Transform& ro_M, const Stripe3& ro_P) {
		const FP32* m = ro_M.m_Mat.m_Memory;

		RegFP32 x =
			_mm256_fmadd_ps(_mm256_set1_ps(m[0]), ro_P.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[1]), ro_P.Y,
											_mm256_fmadd_ps(_mm256_set1_ps(m[2]), ro_P.Z,
															_mm256_set1_ps(m[3]))));

		RegFP32 y =
			_mm256_fmadd_ps(_mm256_set1_ps(m[4]), ro_P.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[5]), ro_P.Y,
											_mm256_fmadd_ps(_mm256_set1_ps(m[6]), ro_P.Z,
															_mm256_set1_ps(m[7]))));

		RegFP32 z =
			_mm256_fmadd_ps(_mm256_set1_ps(m[8]), ro_P.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[9]), ro_P.Y,
											_mm256_fmadd_ps(_mm256_set1_ps(m[10]), ro_P.Z,
															_mm256_set1_ps(m[11]))));

		return Stripe3{ x, y, z };
	}

	Stripe3 applyVector(const Transform& ro_M, const Stripe3& ro_V) {
		const FP32* m = ro_M.m_Mat.m_Memory;

		RegFP32 x =
			_mm256_fmadd_ps(_mm256_set1_ps(m[0]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[1]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[2]), ro_V.Z)));

		RegFP32 y =
			_mm256_fmadd_ps(_mm256_set1_ps(m[4]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[5]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[6]), ro_V.Z)));

		RegFP32 z =
			_mm256_fmadd_ps(_mm256_set1_ps(m[8]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[9]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[10]), ro_V.Z)));

		return Stripe3{ x, y, z };
	}

	Stripe3 applyNormal(const Transform& ro_M, const Stripe3& ro_N) {
		return transformNormal(ro_M.m_Mat, ro_N);
	}

	Stripe3 transformVector(const Mat3f& ro_M, const Stripe3& ro_V) {
		const FP32* m = ro_M.m_Memory;

		RegFP32 x =
			_mm256_fmadd_ps(_mm256_set1_ps(m[0]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[1]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[2]), ro_V.Z)));

		RegFP32 y =
			_mm256_fmadd_ps(_mm256_set1_ps(m[3]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[4]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[5]), ro_V.Z)));

		RegFP32 z =
			_mm256_fmadd_ps(_mm256_set1_ps(m[6]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[7]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[8]), ro_V.Z)));

		return Stripe3{ x, y, z };
	}

	Stripe3 transformNormal(const Mat3f& ro_M, const Stripe3& ro_N) {
		Mat3f invTrans = inverse(transpose(ro_M));
		const FP32* m = invTrans.m_Memory;

		RegFP32 x =
			_mm256_fmadd_ps(_mm256_set1_ps(m[0]), ro_N.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[1]), ro_N.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[2]), ro_N.Z)));

		RegFP32 y =
			_mm256_fmadd_ps(_mm256_set1_ps(m[3]), ro_N.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[4]), ro_N.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[5]), ro_N.Z)));

		RegFP32 z =
			_mm256_fmadd_ps(_mm256_set1_ps(m[6]), ro_N.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[7]), ro_N.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[8]), ro_N.Z)));

		return Stripe3{ x, y, z };
	}

	Stripe3 transformPoint(const Mat4f& ro_M, const Stripe3& ro_P) {
		const FP32* m = ro_M.m_Memory;

		RegFP32 x =
			_mm256_fmadd_ps(_mm256_set1_ps(m[0]), ro_P.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[1]), ro_P.Y,
											_mm256_fmadd_ps(_mm256_set1_ps(m[2]), ro_P.Z,
															_mm256_set1_ps(m[3]))));

		RegFP32 y =
			_mm256_fmadd_ps(_mm256_set1_ps(m[4]), ro_P.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[5]), ro_P.Y,
											_mm256_fmadd_ps(_mm256_set1_ps(m[6]), ro_P.Z,
															_mm256_set1_ps(m[7]))));

		RegFP32 z =
			_mm256_fmadd_ps(_mm256_set1_ps(m[8]), ro_P.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[9]), ro_P.Y,
											_mm256_fmadd_ps(_mm256_set1_ps(m[10]), ro_P.Z,
															_mm256_set1_ps(m[11]))));

		return Stripe3{ x, y, z };
	}

	Stripe3 transformVector(const Mat4f& ro_M, const Stripe3& ro_V) {
		const FP32* m = ro_M.m_Memory;

		RegFP32 x =
			_mm256_fmadd_ps(_mm256_set1_ps(m[0]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[1]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[2]), ro_V.Z)));

		RegFP32 y =
			_mm256_fmadd_ps(_mm256_set1_ps(m[4]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[5]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[6]), ro_V.Z)));

		RegFP32 z =
			_mm256_fmadd_ps(_mm256_set1_ps(m[8]), ro_V.X,
							_mm256_fmadd_ps(_mm256_set1_ps(m[9]), ro_V.Y,
											_mm256_mul_ps(_mm256_set1_ps(m[10]), ro_V.Z)));

		return Stripe3{ x, y, z };
	}

	Stripe3 transformNormal(const Mat4f& ro_M, const Stripe3& ro_N) {
		Mat3f linear;
		const FP32* m = ro_M.m_Memory;

		linear.m_Memory[0] = m[0];  linear.m_Memory[1] = m[1];  linear.m_Memory[2] = m[2];
		linear.m_Memory[3] = m[4];  linear.m_Memory[4] = m[5];  linear.m_Memory[5] = m[6];
		linear.m_Memory[6] = m[8];  linear.m_Memory[7] = m[9];  linear.m_Memory[8] = m[10];

		Mat3f invTrans = inverse(transpose(linear));
		const FP32* n = invTrans.m_Memory;

		RegFP32 x =
			_mm256_fmadd_ps(_mm256_set1_ps(n[0]), ro_N.X,
							_mm256_fmadd_ps(_mm256_set1_ps(n[1]), ro_N.Y,
											_mm256_mul_ps(_mm256_set1_ps(n[2]), ro_N.Z)));

		RegFP32 y =
			_mm256_fmadd_ps(_mm256_set1_ps(n[3]), ro_N.X,
							_mm256_fmadd_ps(_mm256_set1_ps(n[4]), ro_N.Y,
											_mm256_mul_ps(_mm256_set1_ps(n[5]), ro_N.Z)));

		RegFP32 z =
			_mm256_fmadd_ps(_mm256_set1_ps(n[6]), ro_N.X,
							_mm256_fmadd_ps(_mm256_set1_ps(n[7]), ro_N.Y,
											_mm256_mul_ps(_mm256_set1_ps(n[8]), ro_N.Z)));

		return Stripe3{ x, y, z };
	}
}

#endif