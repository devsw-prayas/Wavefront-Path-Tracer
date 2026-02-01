#include <Transcendentals.h>

namespace WavefrontPT::Math::Transcendentals {

    std::pair<float, float>
        quadrantReduction(std::pair<double, double> v_SC, int64_t v_Quad) {
        switch (v_Quad & 3) {
        case 0:
            return { (float)v_SC.first, (float)v_SC.second };
        case 1:
            {
                double t = v_SC.first;
                return { (float)v_SC.second, (float)-t };
            }
        case 2:
            return { (float)-v_SC.first, (float)-v_SC.second };
        case 3:
            {
                double t = v_SC.first;
                return { (float)-v_SC.second, (float)t };
            }
        default:
            return { 0.f, 0.f };
        }
    }

    std::pair<float, float> sincos(float v_Rad) {
        // Reduce in double (input already float by contract)
        const RangeReduction reduced = reduce((double)v_Rad);

        const double r = reduced.radians;
        const double r2 = r * r;

        // Horner form
        double s = r * (1.0 +r2 * (SIN_C3 +r2 * (SIN_C5 +r2 * (SIN_C7 +r2 * SIN_C9))));
        double c = 1.0 +r2 * (COS_C2 +r2 * (COS_C4 +r2 * (COS_C6 +r2 * COS_C8)));
        return quadrantReduction({ s, c }, reduced.quadrant);
    }

}
