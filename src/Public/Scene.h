#pragma once
#include "IntegratorMathCore.h"
#include "Material.h"
#include "GPlane.h"
#include "GSphere.h"

namespace WavefrontPT::Integrator {
	constexpr size_t MAX_COUNT = 20;
	struct Scene final {
		Materials::Material m_Materials[MAX_COUNT] = {};
		Geometry::GSphere m_Spheres[MAX_COUNT] = {};
		Geometry::GPlane m_Planes[MAX_COUNT] = {};
		Math::MaterialID m_MaterialCount;
		Math::ObjectID m_SphereCount;
		Math::ObjectID m_PlaneCount;

		Scene() : m_MaterialCount(0), m_SphereCount(0), m_PlaneCount(0) {}

		Scene(const Scene&) = default;
		Scene& operator=(const Scene&) = default;
		Scene(Scene&&) noexcept = default;
		Scene& operator=(Scene&&) noexcept = default;
		~Scene() = default;
	};

	Math::MaterialID registerMaterial(Scene& ro_Scene, const Materials::Material& ro_Mat);
	Math::ObjectID addSphere(Scene& ro_Scene, const Geometry::GSphere& ro_Sphere);
	Math::ObjectID addPlane(Scene& ro_Scene, const Geometry::GPlane& ro_Plane);
	Math::HitRecord hitScene(const Scene& ro_Scene, const Math::Ray& ro_Ray);
}
