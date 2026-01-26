#pragma once
#include <Core.h>
#include <Functions.h>

namespace WavefrontPT::Math {
	using FP32 = float;
	using FP64 = double;

	struct alignas(16) Point3 final {
		FP32 X, Y, Z;

		constexpr explicit Point3(FP32 v_Value) : X(v_Value), Y(v_Value), Z(v_Value) {}
		constexpr Point3(FP32 v_X, FP32 v_Y, FP32 v_Z) : X(v_X), Y(v_Y), Z(v_Z) {}
		constexpr Point3() : X(0.f), Y(0.f), Z(0.f) {}

		~Point3() = default;

		Point3(const Point3&) = default;
		Point3& operator=(const Point3&) = default;

		Point3(Point3&&) noexcept = default;
		Point3& operator=(Point3&&) noexcept = default;
	};

	struct alignas(16) Vector3 final {
		FP32 X, Y, Z;

		constexpr explicit Vector3(FP32 v_Value) : X(v_Value), Y(v_Value), Z(v_Value) {}
		constexpr Vector3(FP32 v_X, FP32 v_Y, FP32 v_Z) : X(v_X), Y(v_Y), Z(v_Z) {}
		constexpr Vector3() : X(0.f), Y(0.f), Z(0.f) {}

		~Vector3() = default;

		Vector3(const Vector3&) = default;
		Vector3& operator=(const Vector3&) = default;

		Vector3(Vector3&&) = default;
		Vector3& operator=(Vector3&&) = default;
	};

	FP32 sinFP(FP32 v);
	FP32 cosFP(FP32 v);

	// Point Ops
	constexpr Vector3 operator-(const Point3& ro_A, const Point3& ro_B) noexcept;
	constexpr Point3 operator+(const Point3& ro_A, const Vector3& ro_B) noexcept;
	constexpr Point3 operator+(const Vector3& ro_A, const Point3& ro_B) noexcept;

	// Mutators
	constexpr Vector3 operator+(const Vector3& ro_OpA, const Vector3& ro_OpB) noexcept; // Elementwise Add
	constexpr Vector3 operator-(const Vector3& ro_OpA, const Vector3& ro_OpB) noexcept; // Elementwise Subtract
	constexpr Vector3 operator*(const Vector3& ro_OpA, const Vector3& ro_OpB) noexcept; // Elementwise Product (Hadamard)
	constexpr Vector3 operator/(const Vector3& ro_OpA, const Vector3& ro_OpB) noexcept; // Elementwise Divide

	constexpr Vector3 scale(const Vector3& ro_Vec, FP32 v_Scalar) noexcept;
	constexpr FP32 dot(const Vector3& ro_OpA, const Vector3& ro_OpB) noexcept;
	constexpr Vector3 cross(const Vector3& ro_OpA, const Vector3& ro_OpB) noexcept;
	constexpr FP32 lengthSq(const Vector3& ro_Op) noexcept;

	FP32 length(const Vector3& ro_Op) noexcept;

	Vector3 normalize(const Vector3& ro_Vec) noexcept;
	Vector3 reflect(const Vector3& ro_Vec, const Vector3& ro_Ref) noexcept;
	Vector3 faceForward(const Vector3& ro_Vec, const Vector3& ro_N) noexcept;

	Vector3 negate(const Vector3& ro_Vec) noexcept;

	// Vector Ops

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 must be enabled to use vectorized operations"
#else
#include <immintrin.h>
	using RegFP32 = __m256;

	struct alignas(32) Stripe3 final {
		RegFP32 X, Y, Z;

		Stripe3(RegFP32 v_X, RegFP32 v_Y, RegFP32 v_Z) : X(v_X), Y(v_Y), Z(v_Z) {}
		Stripe3() : X(_mm256_setzero_ps()), Y(_mm256_setzero_ps()), Z(_mm256_setzero_ps()) {}

		~Stripe3() = default;

		Stripe3(const Stripe3&) = default;
		Stripe3& operator=(const Stripe3&) = default;

		Stripe3(Stripe3&&) noexcept = default;
		Stripe3& operator=(Stripe3&&) noexcept = default;
	};

	Stripe3 operator+(const Stripe3& ro_A, const Stripe3& ro_B);  // Elementwise vectorized	add
	Stripe3 operator-(const Stripe3& ro_A, const Stripe3& ro_B);  // Elementwise vectorized subtract
	Stripe3 operator*(const Stripe3& ro_A, const Stripe3& ro_B);  // Elementwise vectorized product
	Stripe3 operator/(const Stripe3& ro_A, const Stripe3& ro_B);  // Elementwise vectorized division

	RegFP32 lengthSq(const Stripe3& ro_Vec);
	Stripe3 scale(const Stripe3& ro_Stripe, RegFP32 v_Scalar);
	RegFP32 dot(const Stripe3& ro_A, const Stripe3& ro_B); // Vectorized Dot
	Stripe3 cross(const Stripe3& ro_A, const Stripe3& ro_B); // Vectorized Cross

	RegFP32 length(const Stripe3& ro_Stripe);

	Stripe3 normalize(const Stripe3& ro_Stripe);
	Stripe3 reflect(const Stripe3& ro_Stripe, const Stripe3& ro_Ref);
	Stripe3 faceForward(const Stripe3& ro_Stripe, const Stripe3& ro_N);

	Stripe3 negate(const Stripe3& ro_Stripe);
#endif
}