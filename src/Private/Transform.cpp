#include <Core.h>
#include <Transform.h>

namespace WavefrontPT::Math {

    // ------------------------------------------------------------
    // 3x3
    // ------------------------------------------------------------
    Vector3 transformVector(const Mat3f& ro_M, const Vector3& ro_V) {
        return Vector3{
            index(ro_M, 0, 0) * ro_V.X +
            index(ro_M, 0, 1) * ro_V.Y +
            index(ro_M, 0, 2) * ro_V.Z,

            index(ro_M, 1, 0) * ro_V.X +
            index(ro_M, 1, 1) * ro_V.Y +
            index(ro_M, 1, 2) * ro_V.Z,

            index(ro_M, 2, 0) * ro_V.X +
            index(ro_M, 2, 1) * ro_V.Y +
            index(ro_M, 2, 2) * ro_V.Z
        };
    }

    Vector3 transformNormal(const Mat3f& ro_M, const Vector3& ro_N) {
        Mat3f invTrans = inverse(transpose(ro_M));

        return Vector3{
            index(invTrans, 0, 0) * ro_N.X +
            index(invTrans, 0, 1) * ro_N.Y +
            index(invTrans, 0, 2) * ro_N.Z,

            index(invTrans, 1, 0) * ro_N.X +
            index(invTrans, 1, 1) * ro_N.Y +
            index(invTrans, 1, 2) * ro_N.Z,

            index(invTrans, 2, 0) * ro_N.X +
            index(invTrans, 2, 1) * ro_N.Y +
            index(invTrans, 2, 2) * ro_N.Z
        };
    }

    // ------------------------------------------------------------
    // 4x4
    // ------------------------------------------------------------
    Point3 transformPoint(const Mat4f& ro_M, const Point3& ro_P) {
        FP32 x =
            index(ro_M, 0, 0) * ro_P.X +
            index(ro_M, 0, 1) * ro_P.Y +
            index(ro_M, 0, 2) * ro_P.Z +
            index(ro_M, 0, 3);

        FP32 y =
            index(ro_M, 1, 0) * ro_P.X +
            index(ro_M, 1, 1) * ro_P.Y +
            index(ro_M, 1, 2) * ro_P.Z +
            index(ro_M, 1, 3);

        FP32 z =
            index(ro_M, 2, 0) * ro_P.X +
            index(ro_M, 2, 1) * ro_P.Y +
            index(ro_M, 2, 2) * ro_P.Z +
            index(ro_M, 2, 3);

        FP32 w =
            index(ro_M, 3, 0) * ro_P.X +
            index(ro_M, 3, 1) * ro_P.Y +
            index(ro_M, 3, 2) * ro_P.Z +
            index(ro_M, 3, 3);

        if (w != static_cast<FP32>(1) && w != static_cast<FP32>(0)) {
            FP32 invW = static_cast<FP32>(1) / w;
            x *= invW;
            y *= invW;
            z *= invW;
        }

        return Point3{ x, y, z };
    }

    Vector3 transformVector(const Mat4f& ro_M, const Vector3& ro_V) {
        return Vector3{
            index(ro_M, 0, 0) * ro_V.X +
            index(ro_M, 0, 1) * ro_V.Y +
            index(ro_M, 0, 2) * ro_V.Z,

            index(ro_M, 1, 0) * ro_V.X +
            index(ro_M, 1, 1) * ro_V.Y +
            index(ro_M, 1, 2) * ro_V.Z,

            index(ro_M, 2, 0) * ro_V.X +
            index(ro_M, 2, 1) * ro_V.Y +
            index(ro_M, 2, 2) * ro_V.Z
        };
    }

    Vector3 transformNormal(const Mat4f& ro_M, const Vector3& ro_N) {
        Mat3f linear;

        // Row 0
        linear.m_Memory[0] = ro_M.m_Memory[0]; // (0,0)
        linear.m_Memory[1] = ro_M.m_Memory[1]; // (0,1)
        linear.m_Memory[2] = ro_M.m_Memory[2]; // (0,2)

        // Row 1
        linear.m_Memory[3] = ro_M.m_Memory[4]; // (1,0)
        linear.m_Memory[4] = ro_M.m_Memory[5]; // (1,1)
        linear.m_Memory[5] = ro_M.m_Memory[6]; // (1,2)

        // Row 2
        linear.m_Memory[6] = ro_M.m_Memory[8];  // (2,0)
        linear.m_Memory[7] = ro_M.m_Memory[9];  // (2,1)
        linear.m_Memory[8] = ro_M.m_Memory[10]; // (2,2)

        Mat3f invTrans = inverse(transpose(linear));

        return Vector3{
            invTrans.m_Memory[0] * ro_N.X +
            invTrans.m_Memory[1] * ro_N.Y +
            invTrans.m_Memory[2] * ro_N.Z,

            invTrans.m_Memory[3] * ro_N.X +
            invTrans.m_Memory[4] * ro_N.Y +
            invTrans.m_Memory[5] * ro_N.Z,

            invTrans.m_Memory[6] * ro_N.X +
            invTrans.m_Memory[7] * ro_N.Y +
            invTrans.m_Memory[8] * ro_N.Z
        };
    }


    // ------------------------------------------------------------
    // Transform
    // ------------------------------------------------------------
    Transform makeTransform(const Mat4f& ro_Mat) {
        Transform t;
        t.m_Mat = ro_Mat;
        t.m_Inverse = inverse(ro_Mat);
        return t;
    }

    Transform compose(const Transform& ro_OpA, const Transform& ro_OpB) {
        Transform out;
        out.m_Mat = ro_OpA.m_Mat * ro_OpB.m_Mat;
        out.m_Inverse = ro_OpB.m_Inverse * ro_OpA.m_Inverse;
        return out;
    }

    // ------------------------------------------------------------
    // Apply
    // ------------------------------------------------------------
    Point3 applyPoint(const Transform& ro_M, const Point3& ro_P) {
        return transformPoint(ro_M.m_Mat, ro_P);
    }

    Vector3 applyVector(const Transform& ro_M, const Vector3& ro_V) {
        return transformVector(ro_M.m_Mat, ro_V);
    }

    Vector3 applyNormal(const Transform& ro_M, const Vector3& ro_N) {
        return transformNormal(ro_M.m_Mat, ro_N);
    }

} // namespace WavefrontPT::Math
