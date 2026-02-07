#pragma once
#include "IntegratorMathCore.h"
#include "GSphere.h"
#include "GPlane.h"

namespace WavefrontPT::Geometry {
	using namespace Integrator::Math;

	[[nodiscard]] HitRecord hit(const Ray& ro_Ray, const GSphere& ro_Sphere);
	[[nodiscard]] HitRecord hit(const Ray& ro_Ray, const GPlane& ro_Plane);
}
