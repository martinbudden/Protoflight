#include <cmath>

class FastMath {
public:
    static inline float sinOrder5Unchecked(float x) {
        // assumes x is in the range [0, PI/2]
        // https://breder.org/sine-polynomial, for degree 5 approximation
        static constexpr float c1 =  0.99977140751539240F;
        static constexpr float c3 = -0.16582704279148017F;
        static constexpr float c5 =  0.0075742477643552034F;
        const float x2 = x*x;
        return x*(c1 + x2*(c3 + x2*c5));
    }
    static inline float sinOrder5(float x) {
        // we know x is in the rang [0, PI], having calculated it
        if (x > 0.5F*M_PI_F) { // get x into range [0, PI/2]
            x = M_PI_F - x;
        }
        return sinOrder5Unchecked(x);
    }
    static inline float sinOrder7(float x) {
        // we know x is in the rang [0, PI], having calculated it
        if (x > 0.5F*M_PI_F) { // get x into range [0, PI/2]
            x = M_PI_F - x;
        }
        // https://joelkp.frama.io/tech/modified-taylor.html, for degree 7 approximation
        static constexpr float c1 =  0.99999661599039058046F;
        static constexpr float c3 = -0.99988967477352697077F / 6.0F;
        static constexpr float c5 =  0.99675900242734494228F / 120.0F;
        static constexpr float c7 = -0.92552840500237565369F / 5040.0F;
        const float x2 = x*x;
        return x*(c1 + x2*(c3 + x2*(c5 + x2*c7)));
    }
    static inline float cosOrder5(float x) {
        // we know x is in the rang [0, PI], having calculated it
        // shift x into the range [0, PI/2], for calculating via sin()
        return x < 0.5F*M_PI_F ? sinOrder5Unchecked(0.5F*M_PI_F - x) : -sinOrder5Unchecked(x - 0.5F*M_PI_F);
    }
    static inline float cosOrder7(float x) {
        // x is in the range [0, PI], ensure it stays in that range when shifted for calculating cos
        return x < 0.5F*M_PI_F ? sinOrder7(0.5F*M_PI_F - x) : -sinOrder7(x - 0.5F*M_PI_F);
    }
private:
    // see [Optimized Trigonometric Functions on TI Arm Cores](https://www.ti.com/lit/an/sprad27a/sprad27a.pdf)
    // for explanation of range mapping and coefficients
    // r (remainder) is in range [-0.5, 0.5] and pre-scaled by 2/PI
    static inline float sinPoly5R(float r) {
        static constexpr float c1 =  1.57078719139F;
        static constexpr float c3 = -0.64568519592F;
        static constexpr float c5 =  0.077562883496F;
        const float r2 = r * r;
        return r*(c1 + r2*(c3 + r2*c5));
    }
    static inline float cosPoly6R(float r) {
        static constexpr float c2 = -1.23369765282F;
        static constexpr float c4 =  0.25360107422F;
        static constexpr float c6 = -0.020408373326F;
        const float r2 = r * r;
        return 1.0F + r2*(c2 + r2*(c4 + r2*c6));
    }
    // For sin/cos quadrant helper functions:
    // 2 least significant bits of q are quadrant index, ie [0, 1, 2, 3].
    static inline float sinQuadrant(float r, int q) {
        if (q & 1) {
            // odd quadrant: use cos
            const float c = cosPoly6R(r);
            return (q & 2) ? -c : c; // q=3 -cos, q=1 +cos
        }
        // even quadrant: use sin
        const float s = sinPoly5R(r);
        return (q & 2) ? -s : s; // q=4 -sin, q=2 +sin
    }
    static inline float cosQuadrant(float r, int q) {
        if (q & 1) {
            // odd quadrant: use sin
            const float s = -sinPoly5R(r);
            return (q & 2) ? -s : s; // q=3 -sin, q=1 +sin
        }
        // even quadrant: use cos
        const float c = cosPoly6R(r);
        return (q & 2) ? -c : c; // q=4 -cos, q=2 +cos
    }
    static inline void sincosQuadrant(float r, int q, float& sin, float& cos) {
        const float sb = sinPoly5R(r);
        const float cb = cosPoly6R(r);

        // map values according to quadrant
        float s = (q & 1) ?  cb : sb;
        float c = (q & 1) ? -sb : cb;

        if (q & 2) { // negate for quadrants 2 and 3
            sin = -s;
            cos = -c;
        } else {
            sin = s;
            cos = c;
        }
    }
public:
    static inline float sin(float x) {
        const float t = x * TWO_OVER_PI;
        const float q = roundf(t);       
        const float r = t - q;
        return sinQuadrant(r, static_cast<int>(q));
    }
    static inline float cos(float x) {
        const float t = x * TWO_OVER_PI;
        const float q = roundf(t);
        const float r = t - q;
        return cosQuadrant(r, static_cast<int>(q));
    }
    static inline void sincos(float x, float& sin, float& cos) {
        const float t = x * TWO_OVER_PI; // so remainder will be scaled from range [-PI/4, PI/4] ([-45, 45] degrees) to [-0.5, 0.5]
        const float q = roundf(t);       // nearest quadrant
        const float r = t - q;           // remainder in range [-0.5, 0.5]
        sincosQuadrant(r, static_cast<int>(q), sin, cos);
    }
public:
    static constexpr float M_PI_F = 3.141592653589793F;
    static constexpr float TWO_OVER_PI = 2.0F / M_PI_F;
};
