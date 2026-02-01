#include <Core.h>
#include <RaySphereIntersection.h>
#include <WMath.h>

namespace WavefrontPT::Integrator::Math {
	FP32 intersect(const Ray& ro_Ray, const Sphere& ro_Sphere, FP32 v_Epsilon) {
		Vector3 l = ro_Ray.m_Origin - ro_Sphere.m_Center;
		FP32 b = 2 * dot(ro_Ray.m_DirectionCosine, l);
		FP32 c = dot(l, l) - ro_Sphere.m_Radius * ro_Sphere.m_Radius;
		FP32 det = b * b - 4 * c;

		if (det < 0) return INFINITY;	   
		FP32 root = sqrt(det);
		FP32 t1 = (-b + root) / 2;
		FP32 t0 = (-b - root) / 2;
		if (t0 > v_Epsilon) return t0;
		if (t1 > v_Epsilon) return t1;
		return INFINITY;
	}
}