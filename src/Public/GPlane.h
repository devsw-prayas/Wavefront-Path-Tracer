#pragma once
#include "WMath.h"
#include "IntegratorMathCore.h"

namespace WavefrontPT::Geometry {
	// TODO This will break super fast if all invariants are not considered, pls be careful
	struct GPlane final {
		Math::Point3 m_Center;
		Math::Vector3 m_SurfaceNormal;
		Math::Vector3 m_Tangent;
		Math::Vector3 m_BiTangent;
		Math::FP32 m_HalfWidth;
		Math::FP32 m_HalfBreadth;
		Integrator::Math::MaterialID m_MaterialID;
		Integrator::Math::ObjectID m_ObjectID;

		GPlane() = default;

		GPlane(const Math::Point3& ro_Cen,
			  const Math::Vector3& ro_SurNorm,
			  const Math::Vector3& ro_Tan,
			  const Math::Vector3& ro_BiTan,
			  Math::FP32 v_Hw,
			  Math::FP32 v_Hb,
			  Integrator::Math::MaterialID v_MatID, Integrator::Math::ObjectID v_ObjID)
		: m_Center(ro_Cen), m_SurfaceNormal(ro_SurNorm), m_Tangent(ro_Tan), m_BiTangent(ro_BiTan),
		m_HalfWidth(v_Hw), m_HalfBreadth(v_Hb), m_MaterialID(v_MatID), m_ObjectID(v_ObjID){}

		GPlane(const GPlane&) = default;
		GPlane& operator=(const GPlane&) = default;

		GPlane(GPlane&&) noexcept = default;
		GPlane& operator=(GPlane&&) noexcept = default;

		~GPlane() = default;
	};
}
