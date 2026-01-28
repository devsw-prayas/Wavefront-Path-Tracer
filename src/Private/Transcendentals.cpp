#include <Core.h>
#include <Transcendentals.h>

namespace WavefrontPT::Math::Transcendentals {
	std::pair<float, float> quadrantReduction(std::pair<float, float> v_SC, int v_Quad) {
		switch (v_Quad & 3) {
		case 0: return v_SC;
		case 1:
			{
				const float t = v_SC.first;
				v_SC.first = v_SC.second;
				v_SC.second = -t;
				return v_SC;
			}
		case 2:
			return { -v_SC.first, -v_SC.second };
		case 3:
			{
				const float t1 = v_SC.first;
				v_SC.first = -v_SC.second;
				v_SC.second = t1;
				return v_SC;
			}
		default:
			return { 0.f,0.f };
		}
	}

	std::pair<float, float> sincos(float v_Rad) {
		// Homer polynomials for sin and cos
		const RangeReduction reduced = reduce(v_Rad);
		std::pair<float, float> output;
		const float r = reduced.radians, r2 = r * r;
		output.first =  r*(1.f + r2* (SIN_C3 + r2 * (SIN_C5 + r2 * (SIN_C7 + r2 * SIN_C9))));
		output.second = 1.f + r2 * (COS_C2 + r2 * (COS_C4 + r2 * (COS_C6 + r2 * COS_C8)));
		return quadrantReduction(output, reduced.quadrant);
	}

}