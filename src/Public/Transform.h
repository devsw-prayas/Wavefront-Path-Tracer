#pragma once
#include "Matrix.h"
#include "WMath.h"

// NOTE:
// transform* functions are stateless and perform raw matrix application.
// apply* functions must be preferred in engine code as they enforce correct
// point/vector/normal semantics using cached inverses.

// NOTE:
// Normal transformation always uses inverse-transpose of the linear part.


namespace WavefrontPT::Math {
	//------------------------------------
	// Transform Compute
	//------------------------------------

	Vector3 transformNormal(const Mat3f& ro_M, const Vector3& ro_N);
	Vector3 transformVector(const Mat3f& ro_M, const Vector3& ro_V);

	Point3 transformPoint(const Mat4f& ro_M, const Point3& ro_P);
	Vector3 transformVector(const Mat4f& ro_M, const Vector3& ro_V);
	Vector3 transformNormal(const Mat4f& ro_M, const Vector3& ro_N);

	Transform makeTransform(const Mat4f& ro_Mat);
	Transform compose(const Transform& ro_OpA, const Transform& ro_OpB);

	Point3 applyPoint(const Transform& ro_M, const Point3& ro_P);
	Vector3 applyVector(const Transform& ro_M, const Vector3& ro_V);
	Vector3 applyNormal(const Transform& ro_M, const Vector3& ro_N);

}
