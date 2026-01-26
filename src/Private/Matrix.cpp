#include <Core.h>
#include <Matrix.h>

namespace WavefrontPT::Math {
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
}

#if !defined(EDITOR_MODE) && !defined(__AVX2__)
#error "AVX2 flag must be enabled to use vectorized operations"
#else

#endif