#pragma once

namespace WavefrontPT::Math {
	inline constexpr Vector3 operator-(const Point3& ro_A, const Point3& ro_B) noexcept {
		return { ro_A.X - ro_B.X, ro_A.Y - ro_B.Y, ro_A.Z - ro_B.Z };
	}

	inline constexpr Point3 operator+(const Point3& ro_A, const Vector3& ro_B) noexcept {
		return { ro_A.X + ro_B.X, ro_A.Y + ro_B.Y, ro_A.Z + ro_B.Z };
	}

	inline constexpr Point3 operator+(const Vector3& ro_A, const Point3& ro_B) noexcept {
		return { ro_A.X + ro_B.X, ro_A.Y + ro_B.Y, ro_A.Z + ro_B.Z };
	}

	inline constexpr Vector3 operator+(const Vector3& ro_A, const Vector3& ro_B) noexcept {
		return { ro_A.X + ro_B.X, ro_A.Y + ro_B.Y, ro_A.Z + ro_B.Z };
	}

	inline constexpr Vector3 operator-(const Vector3& ro_A, const Vector3& ro_B) noexcept {
		return { ro_A.X - ro_B.X, ro_A.Y - ro_B.Y, ro_A.Z - ro_B.Z };
	}

	inline constexpr Vector3 operator*(const Vector3& ro_A, const Vector3& ro_B) noexcept {
		return { ro_A.X * ro_B.X, ro_A.Y * ro_B.Y, ro_A.Z * ro_B.Z };
	}

	constexpr Vector3 operator/(const Vector3& ro_A, const Vector3& ro_B) noexcept {
		return { ro_A.X / ro_B.X, ro_A.Y / ro_B.Y, ro_A.Z / ro_B.Z };
	}

	constexpr FP32 dot(const Vector3& ro_OpA, const Vector3& ro_OpB) noexcept {
		return ro_OpA.X * ro_OpB.X + ro_OpA.Y * ro_OpB.Y + ro_OpA.Z * ro_OpB.Z ;
	}

	inline constexpr Vector3 cross(const Vector3& ro_OpA, const Vector3& ro_OpB) noexcept {
		return { ro_OpA.Y * ro_OpB.Z - ro_OpA.Z * ro_OpB.Y,
			ro_OpA.Z * ro_OpB.X - ro_OpB.X * ro_OpA.Z,
			ro_OpA.X * ro_OpB.Y - ro_OpB.X * ro_OpA.Y };
	}

	inline constexpr Vector3 scale(const Vector3& ro_Vec, FP32 v_Scalar) noexcept {
		return { ro_Vec.X * v_Scalar, ro_Vec.Y * v_Scalar, ro_Vec.Z * v_Scalar };	
	}

}