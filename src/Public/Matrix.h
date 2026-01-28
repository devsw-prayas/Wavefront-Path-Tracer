#pragma once
#include <Core.h>
#include <WMath.h>

namespace WavefrontPT::Math {

	//---------------------------------
	// Row Major Layout Matrix
	//---------------------------------
	template<size_t Rows, size_t Columns>
	struct Matrix final {
		Matrix() = default;
		FP32 m_Memory[Rows * Columns];
		~Matrix() = default;
		Matrix(const Matrix&) = default;
		Matrix& operator=(const Matrix&) = default;

		Matrix(Matrix&&) noexcept = default;
		Matrix& operator=(Matrix&&) noexcept = default;
	};

	using Mat4f = Matrix<4, 4>;
	using Mat3f = Matrix<3, 3>;

	struct Transform final {
		Mat4f m_Mat;
		Mat4f m_Inverse;

		~Transform() = default;
		Transform(const Transform&) = default;
		Transform& operator=(const Transform&) = default;

		Transform(Transform&&) noexcept = default;
		Transform& operator=(Transform&&) noexcept = default;
	};

	//------------------------------------
	// 3 By 3 Matrix Compute
	//------------------------------------

	FP32 index(const Mat3f& ro_Mat, size_t v_R, size_t v_C);
	Mat3f identity3();
	Mat3f rotate3(Vector3 v_Axis, FP32 v_Angle);
	Mat3f rotate3(FP32 x_V, FP32 v_Y, FP32 v_Z);
	Mat3f scale3(const Vector3& v_Scale);

	Mat3f operator+(const Mat3f& ro_OpA, const Mat3f& ro_OpB);
	Mat3f operator-(const Mat3f& ro_OpA, const Mat3f& ro_OpB);
	Mat3f hadamard(const Mat3f& ro_OpA, const Mat3f& ro_OpB);
	Mat3f uniformScale3(FP32 v_Scalar);

	Mat3f operator*(const Mat3f& ro_OpA, const Mat3f& ro_OpB);

	Vector3 transformNormal(const Mat3f& ro_M, const Vector3& ro_N);
	Vector3 transformVector(const Mat3f& ro_M, const Vector3& ro_V);

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
	Stripe3 transformNormal(const Mat3f& ro_M, const Stripe3& ro_N);
	Stripe3 transformVector(const Mat3f& ro_M, const Stripe3& ro_V);
#endif

	Mat3f transpose(const Mat3f& ro_Mat);
	Mat3f inverse(const Mat3f& ro_Mat);

	//------------------------------------
	// 4 By 4 Matrix Compute
	//------------------------------------

	FP32 index(const Mat4f& ro_Mat, size_t v_R, size_t v_C);
	Mat4f identity4();
	Mat4f translation(const Vector3& ro_Vec);
	Mat4f scale4(const Vector3& ro_Scale);
	Mat4f uniformScale4(FP32 v_Scale);

	Mat4f rotate4(const Vector3& ro_Axis, FP32 v_Angle);
	Mat4f rotate4(FP32 v_X, FP32 v_Y, FP32 v_Z);
	Mat4f lookAt(const Point3& ro_Eye, const Point3& ro_Target, const Vector3& ro_Up);

	Mat4f operator+(const Mat4f& ro_OpA, const Mat4f& ro_OpB);
	Mat4f operator-(const Mat4f& ro_OpA, const Mat4f& ro_OpB);
	Mat4f hadamard(const Mat4f& ro_OpA, const Mat4f& ro_OpB);

	Mat4f operator*(const Mat4f ro_OpA, const Mat4f& ro_OpB);

	Point3 transformPoint(const Mat4f& ro_M, const Point3& ro_P);
	Vector3 transformVector(const Mat4f& ro_M, const Vector3& ro_V);
	Vector3 transformNormal(const Mat4f& ro_M, const Vector3& ro_N);

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
	Point3 transformPoint(const Mat4f& ro_M, const Stripe3& ro_P);
	Vector3 transformVector(const Mat4f& ro_M, const Stripe3& ro_V);
	Vector3 transformNormal(const Mat4f& ro_M, const Stripe3& ro_N);
#endif

	Mat4f transpose(const Mat4f& ro_Mat);
	Mat4f inverse(const Mat4f& ro_Mat);

	//------------------------------------
	// Transform Compute
	//------------------------------------

	Transform makeTransform(const Mat4f& ro_Mat);
	Transform compose(const Transform& ro_OpA, const Transform& ro_OpB);

	Point3 applyPoint(const Transform& ro_M, const Point3& ro_P);
	Vector3 applyVector(const Transform& ro_M, const Vector3& ro_V);
	Vector3 applyNormal(const Transform& ro_M, const Vector3& ro_N);

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else
	Stripe3 applyPoint(const Transform& ro_M, const Stripe3& ro_P);
	Stripe3 applyVector(const Transform& ro_M, const Stripe3& ro_V);
	Stripe3 applyNormal(const Transform& ro_M, const Stripe3& ro_N);
#endif



}
