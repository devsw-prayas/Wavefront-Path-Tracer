#include <cstdio>
#include <vector>
#include <WMath.h>
#include <numbers>

using WavefrontPT::Math::sinFP;
using WavefrontPT::Math::cosFP;

constexpr float PI = std::numbers::pi_v<float>;
constexpr float TAU = 2.0f * PI;

inline bool near(float a, float b, float eps = 2e-6f) {
    float d = a - b;
    return (d < eps) && (d > -eps);
}

int main() {
    bool ok = true;

    // ---------------------------------
    // 1. Exact angle tests
    // ---------------------------------
    struct Exact {
        const char* name;
        float x;
        float sinE;
        float cosE;
    };

    Exact exacts[] = {
        { "0",        0.0f,        0.0f,  1.0f },
        { "pi/2",     PI * 0.5f,   1.0f,  0.0f },
        { "pi",       PI,          0.0f, -1.0f },
        { "3pi/2",    PI * 1.5f,  -1.0f,  0.0f },
        { "2pi",      TAU,         0.0f,  1.0f },
    };

    for (const auto& e : exacts) {
        float s = sinFP(e.x);
        float c = cosFP(e.x);

        if (!near(s, e.sinE) || !near(c, e.cosE)) {
            ok = false;
            std::printf(
                "[FAIL exact] %-6s  sin=%.6f cos=%.6f\n",
                e.name, s, c
            );
        }
    }

    // ---------------------------------
    // 2. Identity test: sin^2 + cos^2 = 1
    // ---------------------------------
    for (int i = -10000; i <= 10000; ++i) {
        float x = i * 0.001f;
        float s = sinFP(x);
        float c = cosFP(x);
        float n = s * s + c * c;

        if (!near(n, 1.0f, 4e-6f)) {
            ok = false;
            std::printf(
                "[FAIL norm] x=%f  s^2+c^2=%.8f\n",
                x, n
            );
            break;
        }
    }

    // ---------------------------------
    // 3. Odd / even symmetry
    // ---------------------------------
    for (int i = 0; i < 1000; ++i) {
        float x = i * 0.01f;

        if (!near(sinFP(-x), -sinFP(x)) ||
            !near(cosFP(-x), cosFP(x))) {
            ok = false;
            std::printf("[FAIL symmetry] x=%f\n", x);
            break;
        }
    }

    // ---------------------------------
    // 4. Periodicity (range reduction stress)
    // ---------------------------------
    for (int i = 0; i < 1000; ++i) {
        float x = i * 0.1f;

        if (!near(sinFP(x), sinFP(x + TAU)) ||
            !near(cosFP(x), cosFP(x + TAU))) {
            ok = false;
            std::printf("[FAIL periodic] x=%f\n", x);
            break;
        }
    }

    if (ok) {
        std::printf("\nAll sinFP / cosFP tests PASSED \n");
        return 0;
    } else {
        std::printf("\nSome sinFP / cosFP tests FAILED \n");
        return 1;
    }
}
