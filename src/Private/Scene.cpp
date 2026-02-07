#include <Core.h>
#include <Scene.h>

#include "Intersection.h"

namespace WavefrontPT::Integrator {
	Math::ObjectID addPlane(Scene& ro_Scene, const Geometry::GPlane& ro_Plane) {
		if (ro_Scene.m_PlaneCount == MAX_COUNT) return Math::INVALID_OBJ_ID;
		ro_Scene.m_Planes[ro_Scene.m_PlaneCount++] = ro_Plane;
		return ro_Scene.m_PlaneCount -1;
	}

	Math::ObjectID addSphere(Scene& ro_Scene, const Geometry::GSphere& ro_Sphere) {
		if (ro_Scene.m_SphereCount == MAX_COUNT) return Math::INVALID_OBJ_ID;
		ro_Scene.m_Spheres[ro_Scene.m_SphereCount++] = ro_Sphere;
		return ro_Scene.m_SphereCount -1;
	}

	Math::MaterialID registerMaterial(Scene& ro_Scene, const Materials::Material& ro_Mat) {
		if (ro_Scene.m_MaterialCount == MAX_COUNT) return Math::INVALID_MAT_ID;
		ro_Scene.m_Materials[ro_Scene.m_MaterialCount++] = ro_Mat;
		return ro_Scene.m_MaterialCount -1;
	}

	Math::HitRecord hitScene(const Scene& ro_Scene, const Math::Ray& ro_Ray) {
		Math::HitRecord closest = Math::HitRecord::captureMiss();
		Math::FP32 tMin = std::numeric_limits<Math::FP32>::infinity();

		// Spheres
		for (Math::ObjectID i = 0; i < ro_Scene.m_SphereCount; ++i) {
			Math::HitRecord h = Geometry::hit(ro_Ray, ro_Scene.m_Spheres[i]);
			if (h.m_Hit && h.m_T < tMin) {
				tMin = h.m_T;
				closest = h;
			}
		}

		// Planes
		for (Math::ObjectID i = 0; i < ro_Scene.m_PlaneCount; ++i) {
			Math::HitRecord h = Geometry::hit(ro_Ray, ro_Scene.m_Planes[i]);
			if (h.m_Hit && h.m_T < tMin) {
				tMin = h.m_T;
				closest = h;
			}
		}

		return closest;
	}

}
