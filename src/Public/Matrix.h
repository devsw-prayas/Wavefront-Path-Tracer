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
	using Mat3x1f = Matrix<3, 1>;
	using Mat1x3f = Matrix<1, 3>;

	struct Transform final {
		Mat4f m_Mat;
		Mat4f m_Inverse;
		Transform() = default;

		~Transform() = default;
		Transform(const Transform&) = default;
		Transform& operator=(const Transform&) = default;

		Transform(Transform&&) noexcept = default;
		Transform& operator=(Transform&&) noexcept = default;
	};

	//------------------------------------
	// Miscellaneous Operations
	//------------------------------------

	Mat1x3f rowMatrix(const Vector3& ro_Vec);
	Mat3x1f columnMatrix(const Vector3& ro_Vec);

	Mat3f outerProduct(const Vector3& ro_OpA, const Vector3& ro_OpB);
	Mat3f crossProdMat(const Vector3& ro_Vector);

	Mat3x1f transpose(const Mat1x3f& ro_Mat);
	Mat1x3f transpose(const Mat3x1f& ro_Mat);

	Mat1x3f multiply(const Mat1x3f& ro_OpA, const Mat3f& ro_OpB);
	Mat3x1f multiply(const Mat3f& ro_OpA, const Mat3x1f& ro_OpB);


	//------------------------------------
	// 3 By 3 Matrix Compute
	//------------------------------------

	FP32 index(const Mat3f& ro_Mat, size_t v_R, size_t v_C);
	Mat3f identity3();
	Mat3f rotate3(const Vector3& ro_Axis, FP32 v_Angle);
	Mat3f rotate3(FP32 x_V, FP32 v_Y, FP32 v_Z);
	Mat3f scale3(const Vector3& v_Scale);

	Mat3f operator+(const Mat3f& ro_OpA, const Mat3f& ro_OpB);
	Mat3f operator-(const Mat3f& ro_OpA, const Mat3f& ro_OpB);
	Mat3f hadamard(const Mat3f& ro_OpA, const Mat3f& ro_OpB);
	Mat3f uniformScale3(FP32 v_Scalar);

	Mat3f operator*(const Mat3f& ro_OpA, const Mat3f& ro_OpB);

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

	Mat4f operator*(const Mat4f& ro_OpA, const Mat4f& ro_OpB);

	Mat4f transpose(const Mat4f& ro_Mat);
	Mat4f inverse(const Mat4f& ro_Mat);
}
