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
            x =  M_PI_F - x;
        }
        return sinOrder5Unchecked(x);
    }
    static inline float sinOrder7(float x) {
        // we know x is in the rang [0, PI], having calculated it
        if (x > 0.5F*M_PI_F) { // get x into range [0, PI/2]
            x =  M_PI_F - x;
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
public:
    static constexpr float M_PI_F = 3.141592653589793F;
};
