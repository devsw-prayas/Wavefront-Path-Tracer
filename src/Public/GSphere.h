#pragma once
#include "WMath.h"
#include "IntegratorMathCore.h"

namespace WavefrontPT::Geometry {
	struct GSphere {
		Math::Point3 m_Center;
		Math::FP32 m_Radius;

		Integrator::Math::MaterialID m_MaterialID;
		Integrator::Math::ObjectID m_ObjectID;

		GSphere() = default;

		GSphere(Math::Point3 v_Center, Math::FP32 v_Rad, 
			   Integrator::Math::MaterialID v_MatID, Integrator::Math::ObjectID v_ObjID)
		: m_Center(v_Center), m_Radius(v_Rad), m_MaterialID(v_MatID), m_ObjectID(v_ObjID) {}
		~GSphere() = default;

		GSphere(const GSphere&) = default;
		GSphere& operator=(const GSphere&) = default;

		GSphere(GSphere&&) noexcept = default;
		GSphere& operator=(GSphere&&) noexcept = default;
	};
}
