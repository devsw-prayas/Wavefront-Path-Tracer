#include <Core.h>
#include <Matrix.h>

namespace WavefrontPT::Math {
	//------------------------------------
	// Miscellaneous Operations
	//------------------------------------

	Mat1x3f rowMatrix(const Vector3& ro_Vec) {
		Mat1x3f mat;
		mat.m_Memory[0] = ro_Vec.X; mat.m_Memory[1] = ro_Vec.Y;	mat.m_Memory[2] = ro_Vec.Z;
		return mat;
	}

	Mat3x1f columnMatrix(const Vector3& ro_Vec) {
		Mat3x1f mat;
		mat.m_Memory[0] = ro_Vec.X; mat.m_Memory[1] = ro_Vec.Y;	mat.m_Memory[2] = ro_Vec.Z;
		return mat;
	}

	Mat3x1f transpose(const Mat1x3f& ro_Mat) {
		Mat3x1f mat;
		mat.m_Memory[0] = ro_Mat.m_Memory[0];
		mat.m_Memory[1] = ro_Mat.m_Memory[1];
		mat.m_Memory[2] = ro_Mat.m_Memory[2];
		return mat;
	}

	Mat1x3f transpose(const Mat3x1f& ro_Mat) {
		Mat1x3f mat;
		mat.m_Memory[0] = ro_Mat.m_Memory[0];
		mat.m_Memory[1] = ro_Mat.m_Memory[1];
		mat.m_Memory[2] = ro_Mat.m_Memory[2];
		return mat;
	}

	// Computes the outer product (3x1 x 1x3) producing a 3x3 rank-1 matrix.
	Mat3f outerProduct(const Vector3& ro_OpA, const Vector3& ro_OpB) {
		Mat3f m;

		m.m_Memory[0] = ro_OpA.X * ro_OpB.X;
		m.m_Memory[1] = ro_OpA.X * ro_OpB.Y;
		m.m_Memory[2] = ro_OpA.X * ro_OpB.Z;

		m.m_Memory[3] = ro_OpA.Y * ro_OpB.X;
		m.m_Memory[4] = ro_OpA.Y * ro_OpB.Y;
		m.m_Memory[5] = ro_OpA.Y * ro_OpB.Z;

		m.m_Memory[6] = ro_OpA.Z * ro_OpB.X;
		m.m_Memory[7] = ro_OpA.Z * ro_OpB.Y;
		m.m_Memory[8] = ro_OpA.Z * ro_OpB.Z;

		return m;
	}

	Mat3f crossProdMat(const Vector3& ro_Vector) {
		Mat3f hat;
		hat.m_Memory[0] = 0; hat.m_Memory[1] = -ro_Vector.Z; hat.m_Memory[2] = ro_Vector.Y;
		hat.m_Memory[3] = ro_Vector.Z; hat.m_Memory[4] = 0; hat.m_Memory[5] = -ro_Vector.X;
		hat.m_Memory[6] = -ro_Vector.Y; hat.m_Memory[7] = ro_Vector.X; hat.m_Memory[8] = 0;
		return hat;
	}

	// Multiplies a 3c3 matrix with a 3x1 column vector, returning a 3x1 column vector.
	Mat3x1f multiply(const Mat3f& ro_OpA, const Mat3x1f& ro_OpB) {
		Mat3x1f r;
		r.m_Memory[0] = ro_OpA.m_Memory[0] * ro_OpB.m_Memory[0]
			+ ro_OpA.m_Memory[1] * ro_OpB.m_Memory[1]
			+ ro_OpA.m_Memory[2] * ro_OpB.m_Memory[2];

		r.m_Memory[1] = ro_OpA.m_Memory[3] * ro_OpB.m_Memory[0]
			+ ro_OpA.m_Memory[4] * ro_OpB.m_Memory[1]
			+ ro_OpA.m_Memory[5] * ro_OpB.m_Memory[2];

		r.m_Memory[2] = ro_OpA.m_Memory[6] * ro_OpB.m_Memory[0]
			+ ro_OpA.m_Memory[7] * ro_OpB.m_Memory[1]
			+ ro_OpA.m_Memory[8] * ro_OpB.m_Memory[2];

		return r;
	}

	// Multiplies a 1x3 row vector with a 3x3 matrix, returning a 1x3 row vector.
	Mat1x3f multiply(const Mat1x3f& ro_OpA, const Mat3f& ro_OpB) {
		Mat1x3f r;
		r.m_Memory[0] =
			ro_OpA.m_Memory[0] * ro_OpB.m_Memory[0] +
			ro_OpA.m_Memory[1] * ro_OpB.m_Memory[3] +
			ro_OpA.m_Memory[2] * ro_OpB.m_Memory[6];

		r.m_Memory[1] =
			ro_OpA.m_Memory[0] * ro_OpB.m_Memory[1] +
			ro_OpA.m_Memory[1] * ro_OpB.m_Memory[4] +
			ro_OpA.m_Memory[2] * ro_OpB.m_Memory[7];

		r.m_Memory[2] =
			ro_OpA.m_Memory[0] * ro_OpB.m_Memory[2] +
			ro_OpA.m_Memory[1] * ro_OpB.m_Memory[5] +
			ro_OpA.m_Memory[2] * ro_OpB.m_Memory[8];

		return r;
	}

	//------------------------------------
	// 3 By 3 Matrix Compute
	//------------------------------------

	FP32 index(const Mat3f& ro_Mat, size_t v_R, size_t v_C) {
		return ro_Mat.m_Memory[v_R * 3 + v_C];
	}

	Mat3f identity3() {
		Mat3f mat{};
		mat.m_Memory[0] = 1;
		mat.m_Memory[4] = 1;
		mat.m_Memory[8] = 1;
		return mat;
	}

	// Rodrigues Angle-Axis Rotation, axis must be pre-normalized
	Mat3f rotate3(const Vector3& ro_Axis, FP32 v_Angle) {
		const float cosTheta = cosFP(v_Angle);
		const float sinTheta = sinFP(v_Angle);

		const Mat3f i = identity3();
		const Mat3f hat = outerProduct(ro_Axis, ro_Axis);
		const Mat3f A = uniformScale3(cosTheta);
		const Mat3f B = uniformScale3(1.0f - cosTheta);
		const Mat3f C = uniformScale3(sinTheta);

		const Mat3f r = hadamard(i, A) + hadamard(hat, B) + hadamard(C, crossProdMat(ro_Axis));
		return r;
	}

	// Builds a 3x3 rotation matrix using XYZ Euler angles (Rx -> Ry -> Rz, applied in that order).
	Mat3f rotate3(FP32 x_V, FP32 v_Y, FP32 v_Z) {
		const FP32 cx = cosFP(x_V);
		const FP32 sx = sinFP(x_V);
		const FP32 cy = cosFP(v_Y);
		const FP32 sy = sinFP(v_Y);
		const FP32 cz = cosFP(v_Z);
		const FP32 sz = sinFP(v_Z);

		Mat3f R;

		// Row-major layout
		R.m_Memory[0] = cy * cz;
		R.m_Memory[1] = cz * sx * sy - cx * sz;
		R.m_Memory[2] = cx * cz * sy + sx * sz;

		R.m_Memory[3] = cy * sz;
		R.m_Memory[4] = cx * cz + sx * sy * sz;
		R.m_Memory[5] = -cz * sx + cx * sy * sz;

		R.m_Memory[6] = -sy;
		R.m_Memory[7] = cy * sx;
		R.m_Memory[8] = cx * cy;

		return R;
	}

	Mat3f scale3(const Vector3& v_Scale) {
		Mat3f S;

		S.m_Memory[0] = v_Scale.X;  S.m_Memory[1] = 0.0f;       S.m_Memory[2] = 0.0f;
		S.m_Memory[3] = 0.0f;       S.m_Memory[4] = v_Scale.Y;  S.m_Memory[5] = 0.0f;
		S.m_Memory[6] = 0.0f;       S.m_Memory[7] = 0.0f;       S.m_Memory[8] = v_Scale.Z;

		return S;
	}

	Mat3f operator+(const Mat3f& ro_OpA, const Mat3f& ro_OpB) {
		Mat3f r;
		for (size_t i = 0; i < 9; ++i)
			r.m_Memory[i] = ro_OpA.m_Memory[i] + ro_OpB.m_Memory[i];
		return r;
	}

	Mat3f operator-(const Mat3f& ro_OpA, const Mat3f& ro_OpB) {
		Mat3f r;
		for (size_t i = 0; i < 9; ++i)
			r.m_Memory[i] = ro_OpA.m_Memory[i] - ro_OpB.m_Memory[i];
		return r;
	}

	Mat3f hadamard(const Mat3f& ro_OpA, const Mat3f& ro_OpB) {
		Mat3f r;
		for (size_t i = 0; i < 9; ++i)
			r.m_Memory[i] = ro_OpA.m_Memory[i] * ro_OpB.m_Memory[i];
		return r;
	}

	Mat3f uniformScale3(FP32 v_Scalar) {
		Mat3f r;

		r.m_Memory[0] = v_Scalar; r.m_Memory[1] = 0.0f;     r.m_Memory[2] = 0.0f;
		r.m_Memory[3] = 0.0f;     r.m_Memory[4] = v_Scalar; r.m_Memory[5] = 0.0f;
		r.m_Memory[6] = 0.0f;     r.m_Memory[7] = 0.0f;     r.m_Memory[8] = v_Scalar;

		return r;
	}

	Mat3f operator*(const Mat3f& A, const Mat3f& B) {
		Mat3f C;

		C.m_Memory[0] = A.m_Memory[0] * B.m_Memory[0] + A.m_Memory[1] * B.m_Memory[3] + A.m_Memory[2] * B.m_Memory[6];
		C.m_Memory[1] = A.m_Memory[0] * B.m_Memory[1] + A.m_Memory[1] * B.m_Memory[4] + A.m_Memory[2] * B.m_Memory[7];
		C.m_Memory[2] = A.m_Memory[0] * B.m_Memory[2] + A.m_Memory[1] * B.m_Memory[5] + A.m_Memory[2] * B.m_Memory[8];

		C.m_Memory[3] = A.m_Memory[3] * B.m_Memory[0] + A.m_Memory[4] * B.m_Memory[3] + A.m_Memory[5] * B.m_Memory[6];
		C.m_Memory[4] = A.m_Memory[3] * B.m_Memory[1] + A.m_Memory[4] * B.m_Memory[4] + A.m_Memory[5] * B.m_Memory[7];
		C.m_Memory[5] = A.m_Memory[3] * B.m_Memory[2] + A.m_Memory[4] * B.m_Memory[5] + A.m_Memory[5] * B.m_Memory[8];

		C.m_Memory[6] = A.m_Memory[6] * B.m_Memory[0] + A.m_Memory[7] * B.m_Memory[3] + A.m_Memory[8] * B.m_Memory[6];
		C.m_Memory[7] = A.m_Memory[6] * B.m_Memory[1] + A.m_Memory[7] * B.m_Memory[4] + A.m_Memory[8] * B.m_Memory[7];
		C.m_Memory[8] = A.m_Memory[6] * B.m_Memory[2] + A.m_Memory[7] * B.m_Memory[5] + A.m_Memory[8] * B.m_Memory[8];

		return C;
	}

	Mat3f transpose(const Mat3f& ro_Mat) {
		Mat3f T;

		T.m_Memory[0] = ro_Mat.m_Memory[0];
		T.m_Memory[1] = ro_Mat.m_Memory[3];
		T.m_Memory[2] = ro_Mat.m_Memory[6];

		T.m_Memory[3] = ro_Mat.m_Memory[1];
		T.m_Memory[4] = ro_Mat.m_Memory[4];
		T.m_Memory[5] = ro_Mat.m_Memory[7];

		T.m_Memory[6] = ro_Mat.m_Memory[2];
		T.m_Memory[7] = ro_Mat.m_Memory[5];
		T.m_Memory[8] = ro_Mat.m_Memory[8];

		return T;
	}

	Mat3f inverse(const Mat3f& ro_Mat) {
		const FP32 a = ro_Mat.m_Memory[0], b = ro_Mat.m_Memory[1], c = ro_Mat.m_Memory[2];
		const FP32 d = ro_Mat.m_Memory[3], e = ro_Mat.m_Memory[4], f = ro_Mat.m_Memory[5];
		const FP32 g = ro_Mat.m_Memory[6], h = ro_Mat.m_Memory[7], i = ro_Mat.m_Memory[8];

		const FP32 A = (e * i - f * h);
		const FP32 B = -(d * i - f * g);
		const FP32 C = (d * h - e * g);

		const FP32 D = -(b * i - c * h);
		const FP32 E = (a * i - c * g);
		const FP32 F = -(a * h - b * g);

		const FP32 G = (b * f - c * e);
		const FP32 H = -(a * f - c * d);
		const FP32 I = (a * e - b * d);

		const FP32 det = a * A + b * B + c * C;

		// TODO assert(abs(det) > epsilon);

		const FP32 invDet = static_cast <FP32>(1) / det;

		Mat3f inv;

		inv.m_Memory[0] = A * invDet;
		inv.m_Memory[1] = D * invDet;
		inv.m_Memory[2] = G * invDet;

		inv.m_Memory[3] = B * invDet;
		inv.m_Memory[4] = E * invDet;
		inv.m_Memory[5] = H * invDet;

		inv.m_Memory[6] = C * invDet;
		inv.m_Memory[7] = F * invDet;
		inv.m_Memory[8] = I * invDet;

		return inv;
	}

	//------------------------------------
	// 4 By 4 Matrix Compute
	//------------------------------------

	FP32 index(const Mat4f& ro_Mat, size_t v_R, size_t v_C) {
		return ro_Mat.m_Memory[v_R * 4 + v_C];
	}

	Mat4f identity4() {
		Mat4f mat{};
		mat.m_Memory[0] = 1;
		mat.m_Memory[5] = 1;
		mat.m_Memory[10] = 1;
		mat.m_Memory[15] = 1;
		return  mat;
	}

	Mat4f translation(const Vector3& ro_Vec) {
		Mat4f T;

		T.m_Memory[0] = 1.0f; T.m_Memory[1] = 0.0f; T.m_Memory[2] = 0.0f; T.m_Memory[3] = ro_Vec.X;
		T.m_Memory[4] = 0.0f; T.m_Memory[5] = 1.0f; T.m_Memory[6] = 0.0f; T.m_Memory[7] = ro_Vec.Y;
		T.m_Memory[8] = 0.0f; T.m_Memory[9] = 0.0f; T.m_Memory[10] = 1.0f; T.m_Memory[11] = ro_Vec.Z;
		T.m_Memory[12] = 0.0f; T.m_Memory[13] = 0.0f; T.m_Memory[14] = 0.0f; T.m_Memory[15] = 1.0f;

		return T;
	}

	Mat4f scale4(const Vector3& ro_Scale) {
		Mat4f S;

		S.m_Memory[0] = ro_Scale.X; S.m_Memory[1] = 0.0f;       S.m_Memory[2] = 0.0f;       S.m_Memory[3] = 0.0f;
		S.m_Memory[4] = 0.0f;       S.m_Memory[5] = ro_Scale.Y; S.m_Memory[6] = 0.0f;       S.m_Memory[7] = 0.0f;
		S.m_Memory[8] = 0.0f;       S.m_Memory[9] = 0.0f;       S.m_Memory[10] = ro_Scale.Z; S.m_Memory[11] = 0.0f;
		S.m_Memory[12] = 0.0f;       S.m_Memory[13] = 0.0f;       S.m_Memory[14] = 0.0f;       S.m_Memory[15] = 1.0f;

		return S;
	}

	Mat4f uniformScale4(FP32 v_Scale) {
		Mat4f U;

		U.m_Memory[0] = v_Scale; U.m_Memory[1] = 0.0f;     U.m_Memory[2] = 0.0f;     U.m_Memory[3] = 0.0f;
		U.m_Memory[4] = 0.0f;     U.m_Memory[5] = v_Scale; U.m_Memory[6] = 0.0f;     U.m_Memory[7] = 0.0f;
		U.m_Memory[8] = 0.0f;     U.m_Memory[9] = 0.0f;     U.m_Memory[10] = v_Scale; U.m_Memory[11] = 0.0f;
		U.m_Memory[12] = 0.0f;     U.m_Memory[13] = 0.0f;     U.m_Memory[14] = 0.0f;     U.m_Memory[15] = 1.0f;

		return U;
	}

	Mat4f rotate4(const Vector3& ro_Axis, FP32 v_Angle) {
		const Mat3f R3 = rotate3(ro_Axis, v_Angle);

		Mat4f R;

		R.m_Memory[0] = R3.m_Memory[0]; R.m_Memory[1] = R3.m_Memory[1]; R.m_Memory[2] = R3.m_Memory[2]; R.m_Memory[3] = 0.0f;
		R.m_Memory[4] = R3.m_Memory[3]; R.m_Memory[5] = R3.m_Memory[4]; R.m_Memory[6] = R3.m_Memory[5]; R.m_Memory[7] = 0.0f;
		R.m_Memory[8] = R3.m_Memory[6]; R.m_Memory[9] = R3.m_Memory[7]; R.m_Memory[10] = R3.m_Memory[8]; R.m_Memory[11] = 0.0f;

		R.m_Memory[12] = 0.0f;
		R.m_Memory[13] = 0.0f;
		R.m_Memory[14] = 0.0f;
		R.m_Memory[15] = 1.0f;

		return R;
	}

	Mat4f rotate4(FP32 v_X, FP32 v_Y, FP32 v_Z) {
		const Mat3f R3 = rotate3(v_X, v_Y, v_Z);

		Mat4f R;

		R.m_Memory[0] = R3.m_Memory[0]; R.m_Memory[1] = R3.m_Memory[1]; R.m_Memory[2] = R3.m_Memory[2]; R.m_Memory[3] = 0.0f;
		R.m_Memory[4] = R3.m_Memory[3]; R.m_Memory[5] = R3.m_Memory[4]; R.m_Memory[6] = R3.m_Memory[5]; R.m_Memory[7] = 0.0f;
		R.m_Memory[8] = R3.m_Memory[6]; R.m_Memory[9] = R3.m_Memory[7]; R.m_Memory[10] = R3.m_Memory[8]; R.m_Memory[11] = 0.0f;

		R.m_Memory[12] = 0.0f;
		R.m_Memory[13] = 0.0f;
		R.m_Memory[14] = 0.0f;
		R.m_Memory[15] = 1.0f;

		return R;
	}

	Mat4f lookAt(const Point3& ro_Eye,const Point3& ro_Target,const Vector3& ro_Up) {

		const Vector3 f = normalize(ro_Target - ro_Eye);
		const Vector3 r = normalize(cross(f, ro_Up));
		const Vector3 u = cross(r, f);

		Mat4f V;
		const Vector3 eye = ro_Eye - Point3{ 0, 0, 0 };
		V.m_Memory[0] = r.X;  V.m_Memory[1] = r.Y;  V.m_Memory[2] = r.Z;  V.m_Memory[3] = -dot(r, eye);
		V.m_Memory[4] = u.X;  V.m_Memory[5] = u.Y;  V.m_Memory[6] = u.Z;  V.m_Memory[7] = -dot(u, eye);
		V.m_Memory[8] = -f.X; V.m_Memory[9] = -f.Y; V.m_Memory[10] = -f.Z; V.m_Memory[11] = dot(f, eye);

		V.m_Memory[12] = 0.0f;
		V.m_Memory[13] = 0.0f;
		V.m_Memory[14] = 0.0f;
		V.m_Memory[15] = 1.0f;

		return V;
	}

	Mat4f operator+(const Mat4f& A, const Mat4f& B) {
		Mat4f R;
		for (size_t i = 0; i < 16; ++i)
			R.m_Memory[i] = A.m_Memory[i] + B.m_Memory[i];
		return R;
	}

	Mat4f operator-(const Mat4f& A, const Mat4f& B) {
		Mat4f R;
		for (size_t i = 0; i < 16; ++i)
			R.m_Memory[i] = A.m_Memory[i] - B.m_Memory[i];
		return R;
	}

	Mat4f hadamard(const Mat4f& A, const Mat4f& B) {
		Mat4f R;
		for (size_t i = 0; i < 16; ++i)
			R.m_Memory[i] = A.m_Memory[i] * B.m_Memory[i];
		return R;
	}

	Mat4f operator*(const Mat4f& A, const Mat4f& B) {
		Mat4f C;

		C.m_Memory[0] = A.m_Memory[0] * B.m_Memory[0] + A.m_Memory[1] * B.m_Memory[4] + A.m_Memory[2] * B.m_Memory[8] + A.m_Memory[3] * B.m_Memory[12];
		C.m_Memory[1] = A.m_Memory[0] * B.m_Memory[1] + A.m_Memory[1] * B.m_Memory[5] + A.m_Memory[2] * B.m_Memory[9] + A.m_Memory[3] * B.m_Memory[13];
		C.m_Memory[2] = A.m_Memory[0] * B.m_Memory[2] + A.m_Memory[1] * B.m_Memory[6] + A.m_Memory[2] * B.m_Memory[10] + A.m_Memory[3] * B.m_Memory[14];
		C.m_Memory[3] = A.m_Memory[0] * B.m_Memory[3] + A.m_Memory[1] * B.m_Memory[7] + A.m_Memory[2] * B.m_Memory[11] + A.m_Memory[3] * B.m_Memory[15];

		C.m_Memory[4] = A.m_Memory[4] * B.m_Memory[0] + A.m_Memory[5] * B.m_Memory[4] + A.m_Memory[6] * B.m_Memory[8] + A.m_Memory[7] * B.m_Memory[12];
		C.m_Memory[5] = A.m_Memory[4] * B.m_Memory[1] + A.m_Memory[5] * B.m_Memory[5] + A.m_Memory[6] * B.m_Memory[9] + A.m_Memory[7] * B.m_Memory[13];
		C.m_Memory[6] = A.m_Memory[4] * B.m_Memory[2] + A.m_Memory[5] * B.m_Memory[6] + A.m_Memory[6] * B.m_Memory[10] + A.m_Memory[7] * B.m_Memory[14];
		C.m_Memory[7] = A.m_Memory[4] * B.m_Memory[3] + A.m_Memory[5] * B.m_Memory[7] + A.m_Memory[6] * B.m_Memory[11] + A.m_Memory[7] * B.m_Memory[15];

		C.m_Memory[8] = A.m_Memory[8] * B.m_Memory[0] + A.m_Memory[9] * B.m_Memory[4] + A.m_Memory[10] * B.m_Memory[8] + A.m_Memory[11] * B.m_Memory[12];
		C.m_Memory[9] = A.m_Memory[8] * B.m_Memory[1] + A.m_Memory[9] * B.m_Memory[5] + A.m_Memory[10] * B.m_Memory[9] + A.m_Memory[11] * B.m_Memory[13];
		C.m_Memory[10] = A.m_Memory[8] * B.m_Memory[2] + A.m_Memory[9] * B.m_Memory[6] + A.m_Memory[10] * B.m_Memory[10] + A.m_Memory[11] * B.m_Memory[14];
		C.m_Memory[11] = A.m_Memory[8] * B.m_Memory[3] + A.m_Memory[9] * B.m_Memory[7] + A.m_Memory[10] * B.m_Memory[11] + A.m_Memory[11] * B.m_Memory[15];

		C.m_Memory[12] = A.m_Memory[12] * B.m_Memory[0] + A.m_Memory[13] * B.m_Memory[4] + A.m_Memory[14] * B.m_Memory[8] + A.m_Memory[15] * B.m_Memory[12];
		C.m_Memory[13] = A.m_Memory[12] * B.m_Memory[1] + A.m_Memory[13] * B.m_Memory[5] + A.m_Memory[14] * B.m_Memory[9] + A.m_Memory[15] * B.m_Memory[13];
		C.m_Memory[14] = A.m_Memory[12] * B.m_Memory[2] + A.m_Memory[13] * B.m_Memory[6] + A.m_Memory[14] * B.m_Memory[10] + A.m_Memory[15] * B.m_Memory[14];
		C.m_Memory[15] = A.m_Memory[12] * B.m_Memory[3] + A.m_Memory[13] * B.m_Memory[7] + A.m_Memory[14] * B.m_Memory[11] + A.m_Memory[15] * B.m_Memory[15];

		return C;
	}

	Mat4f transpose(const Mat4f& ro_Mat) {
		Mat4f T;

		T.m_Memory[0] = ro_Mat.m_Memory[0];
		T.m_Memory[1] = ro_Mat.m_Memory[4];
		T.m_Memory[2] = ro_Mat.m_Memory[8];
		T.m_Memory[3] = ro_Mat.m_Memory[12];

		T.m_Memory[4] = ro_Mat.m_Memory[1];
		T.m_Memory[5] = ro_Mat.m_Memory[5];
		T.m_Memory[6] = ro_Mat.m_Memory[9];
		T.m_Memory[7] = ro_Mat.m_Memory[13];

		T.m_Memory[8] = ro_Mat.m_Memory[2];
		T.m_Memory[9] = ro_Mat.m_Memory[6];
		T.m_Memory[10] = ro_Mat.m_Memory[10];
		T.m_Memory[11] = ro_Mat.m_Memory[14];

		T.m_Memory[12] = ro_Mat.m_Memory[3];
		T.m_Memory[13] = ro_Mat.m_Memory[7];
		T.m_Memory[14] = ro_Mat.m_Memory[11];
		T.m_Memory[15] = ro_Mat.m_Memory[15];

		return T;
	}

	Mat4f inverse(const Mat4f& ro_Mat) {
		Mat4f inv;

		inv.m_Memory[0] =
			ro_Mat.m_Memory[5] * ro_Mat.m_Memory[10] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[5] * ro_Mat.m_Memory[11] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[9] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[9] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[13] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[11] -
			ro_Mat.m_Memory[13] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[10];

		inv.m_Memory[1] =
			-ro_Mat.m_Memory[1] * ro_Mat.m_Memory[10] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[1] * ro_Mat.m_Memory[11] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[9] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[9] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[13] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[11] +
			ro_Mat.m_Memory[13] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[10];

		inv.m_Memory[2] =
			ro_Mat.m_Memory[1] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[1] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[5] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[5] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[13] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[7] -
			ro_Mat.m_Memory[13] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[6];

		inv.m_Memory[3] =
			-ro_Mat.m_Memory[1] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[11] +
			ro_Mat.m_Memory[1] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[10] +
			ro_Mat.m_Memory[5] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[11] -
			ro_Mat.m_Memory[5] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[10] -
			ro_Mat.m_Memory[9] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[7] +
			ro_Mat.m_Memory[9] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[6];

		inv.m_Memory[4] =
			-ro_Mat.m_Memory[4] * ro_Mat.m_Memory[10] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[11] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[11] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[10];

		inv.m_Memory[5] =
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[10] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[11] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[11] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[10];

		inv.m_Memory[6] =
			-ro_Mat.m_Memory[0] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[7] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[6];

		inv.m_Memory[7] =
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[11] -
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[10] -
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[11] +
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[10] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[7] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[6];

		inv.m_Memory[8] =
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[9] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[11] * ro_Mat.m_Memory[13] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[5] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[13] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[5] * ro_Mat.m_Memory[11] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[9];

		inv.m_Memory[9] =
			-ro_Mat.m_Memory[0] * ro_Mat.m_Memory[9] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[11] * ro_Mat.m_Memory[13] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[13] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[11] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[9];

		inv.m_Memory[10] =
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[5] * ro_Mat.m_Memory[15] -
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[13] -
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[15] +
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[13] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[7] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[5];

		inv.m_Memory[11] =
			-ro_Mat.m_Memory[0] * ro_Mat.m_Memory[5] * ro_Mat.m_Memory[11] +
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[7] * ro_Mat.m_Memory[9] +
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[11] -
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[9] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[7] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[3] * ro_Mat.m_Memory[5];

		inv.m_Memory[12] =
			-ro_Mat.m_Memory[4] * ro_Mat.m_Memory[9] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[10] * ro_Mat.m_Memory[13] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[5] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[13] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[5] * ro_Mat.m_Memory[10] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[9];

		inv.m_Memory[13] =
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[9] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[10] * ro_Mat.m_Memory[13] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[13] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[10] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[9];

		inv.m_Memory[14] =
			-ro_Mat.m_Memory[0] * ro_Mat.m_Memory[5] * ro_Mat.m_Memory[14] +
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[13] +
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[14] -
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[13] -
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[6] +
			ro_Mat.m_Memory[12] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[5];

		inv.m_Memory[15] =
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[5] * ro_Mat.m_Memory[10] -
			ro_Mat.m_Memory[0] * ro_Mat.m_Memory[6] * ro_Mat.m_Memory[9] -
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[10] +
			ro_Mat.m_Memory[4] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[9] +
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[1] * ro_Mat.m_Memory[6] -
			ro_Mat.m_Memory[8] * ro_Mat.m_Memory[2] * ro_Mat.m_Memory[5];

		const FP32 det =
			ro_Mat.m_Memory[0] * inv.m_Memory[0] +
			ro_Mat.m_Memory[1] * inv.m_Memory[4] +
			ro_Mat.m_Memory[2] * inv.m_Memory[8] +
			ro_Mat.m_Memory[3] * inv.m_Memory[12];

		// assert(abs(det) > epsilon);

		const FP32 invDet = static_cast<FP32>(1) / det;

		for (auto& mem : inv.m_Memory) mem *= invDet;

		return inv;
	}


}