#pragma once
#include <Core.h>
#include <WMath.h>

namespace WavefrontPT::Integrator::Math {
	using namespace WavefrontPT::Math;
	using MaterialID = uint32_t;
	using ObjectID = uint32_t;

	constexpr MaterialID INVALID_MAT_ID = UINT32_MAX;
	constexpr ObjectID INVALID_OBJ_ID = UINT32_MAX;

	constexpr FP32 MISS = INFINITY;

	struct Ray {
		Point3 m_Origin;
		Vector3 m_DirectionCosine;

		~Ray() = default;
		Ray(Point3 v_Origin, Vector3 v_DirCos) : m_Origin(v_Origin), m_DirectionCosine(v_DirCos) {}

		Ray(const Ray&) = default;
		Ray& operator=(const Ray&) = default;

		Ray(Ray&&) noexcept = default;
		Ray& operator=(Ray&&) noexcept = default;
	};

	struct alignas(64) HitRecord final {
		Vector3 m_GeometricNormal;
		Point3 m_HitPoint;
		FP32 m_T;
		MaterialID m_MatID;
		ObjectID m_ObjID;
		bool m_Hit;

		HitRecord(const Vector3& ro_GN, const Point3& ro_Hit, FP32 v_T, MaterialID v_MatID, ObjectID v_ObjID, bool v_Hit) :
			m_GeometricNormal(ro_GN), m_HitPoint(ro_Hit), m_T(v_T), m_MatID(v_MatID), m_ObjID(v_ObjID), m_Hit(v_Hit) {
		}

		HitRecord(const HitRecord&) = default;
		HitRecord& operator=(const HitRecord&) = default;

		HitRecord(HitRecord&&) noexcept = default;
		HitRecord& operator=(HitRecord&&) noexcept = default;

		~HitRecord() = default;

		static HitRecord captureHit(const Vector3& ro_GN, const Point3& ro_HitPoint,
			FP32 v_T, MaterialID v_MatID, ObjectID v_ObjID) {
			return HitRecord{
				ro_GN,
				ro_HitPoint,
				v_T,
				v_MatID,
				v_ObjID,
				true
			};
		}

		static HitRecord captureMiss() {
			return HitRecord{
				Vector3(0.0f, 0.0f, 0.0f),
				Point3(0.0f, 0.0f, 0.0f),
				std::numeric_limits<FP32>::infinity(),
				INVALID_MAT_ID,
				INVALID_OBJ_ID,
				false
			};
		}
	};
}
