#pragma once

namespace WavefrontPT::Math::Transcendentals {

    // High-precision split constants (NO float literals)
    constexpr double PI_2_HI = 1.57079625129699707031;
    constexpr double PI_2_LO = 7.54978941586159635335e-08;
    constexpr double INV_PI_2 = 0.63661977236758134308;

    struct RangeReduction final {
        double  radians;
        int64_t quadrant;
    };

    inline RangeReduction reduce(double x) {
        // IEEE-correct rounding
        int64_t k = static_cast<int64_t>(std::nearbyint(x * INV_PI_2));

        double r = x;
        r -= k * PI_2_HI;
        r -= k * PI_2_LO;

        return { r, k & 3 };
    }

    // Sine polynomial (double precision coefficients)
    constexpr double SIN_C3 = -1.6666654611e-1;
    constexpr double SIN_C5 = 8.3321608736e-3;
    constexpr double SIN_C7 = -1.9515295891e-4;
    constexpr double SIN_C9 = 2.5925366036e-6;

    // Cosine polynomial
    constexpr double COS_C2 = -0.4999999973;
    constexpr double COS_C4 = 0.0416666233;
    constexpr double COS_C6 = -0.0013886763;
    constexpr double COS_C8 = 0.000024390448;

    std::pair<float, float>
        quadrantReduction(std::pair<double, double> v_SC, int64_t v_Quad);

    std::pair<float, float> sincos(float v_Rad);

    inline float sin(float v_Rad) {
        return sincos(v_Rad).first;
    }

    inline float cos(float v_Rad) {
        return sincos(v_Rad).second;
    }

}
