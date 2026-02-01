#pragma once
#include <Core.h>
#include <IntegratorMathCore.h>
#include <WMath.h>

namespace WavefrontPT::Integrator::Math {
	using namespace WavefrontPT::Math;

	struct Sphere {
		Point3 m_Center;
		FP32 m_Radius;

		Sphere(Point3 v_Center, FP32 v_Rad) : m_Center(v_Center), m_Radius(v_Rad) {}
		~Sphere() = default;

		Sphere(const Sphere&) = default;
		Sphere& operator=(const Sphere&) = default;

		Sphere(Sphere&&) noexcept = default;
		Sphere& operator=(Sphere&&) noexcept = default;
	};

	[[nodiscard]] FP32 intersect(const Ray& ro_Ray, const Sphere& ro_Sphere, FP32 v_Epsilon);
}
