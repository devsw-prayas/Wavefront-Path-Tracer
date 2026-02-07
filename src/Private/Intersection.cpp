#include "Core.h"
#include "Intersection.h"

namespace WavefrontPT::Geometry {
	HitRecord hit(const Ray& ro_Ray, const GPlane& ro_Plane) {
		FP32 d = dot(ro_Plane.m_SurfaceNormal, ro_Ray.m_DirectionCosine);
		if (absFast(d) < kEpsilon) return HitRecord::captureMiss();
		FP32 t = dot((ro_Plane.m_Center - ro_Ray.m_Origin), ro_Plane.m_SurfaceNormal) / d;
		if (t < kEpsilon) return HitRecord::captureMiss();
		Point3 p = ro_Ray.m_Origin + scale(ro_Ray.m_DirectionCosine, t);
		Vector3 projection = p - ro_Plane.m_Center;
		FP32 u = dot(projection, ro_Plane.m_Tangent);
		FP32 v = dot(projection, ro_Plane.m_BiTangent);
		if (absFast(u) <= ro_Plane.m_HalfWidth && absFast(v) <= ro_Plane.m_HalfBreadth)
			return HitRecord::captureHit(ro_Plane.m_SurfaceNormal, p, t, ro_Plane.m_MaterialID, ro_Plane.m_ObjectID);
		return HitRecord::captureMiss();
	}

	HitRecord hit(const Ray& ro_Ray, const GSphere& ro_Sphere) {
		Vector3 l = ro_Ray.m_Origin - ro_Sphere.m_Center;
		FP32 b = dot(ro_Ray.m_DirectionCosine, l);
		FP32 c = lengthSq(l) - ro_Sphere.m_Radius * ro_Sphere.m_Radius;

		FP32 det = b * b - c;
		if (det < 0) return HitRecord::captureMiss();

		FP32 root = sqrt(det);
		FP32 t0 = (-b - root);
		FP32 t1 = (-b + root);

		Point3 p;
		FP32 t = 0.f;

		if (t0 > kEpsilon) {
			p = ro_Ray.m_Origin + scale(ro_Ray.m_DirectionCosine, t = t0);
		} else if (t1 > kEpsilon) {
			p = ro_Ray.m_Origin + scale(ro_Ray.m_DirectionCosine, t = t1);
		} else return HitRecord::captureMiss();

		Vector3 v = p - ro_Sphere.m_Center;
		Vector3 gn = normalize(v);

		return HitRecord::captureHit(gn, p, t, ro_Sphere.m_MaterialID, ro_Sphere.m_ObjectID);

	}
}