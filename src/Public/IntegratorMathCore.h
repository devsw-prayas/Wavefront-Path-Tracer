#pragma once
#include <Core.h>
#include <WMath.h>

using WavefrontPT::Math::Vector3;
using WavefrontPT::Math::Point3;

namespace WavefrontPT::Integrator::Math {
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


}
