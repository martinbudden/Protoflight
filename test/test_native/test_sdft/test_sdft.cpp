#include <dynamic_notch_filter.h>
#include <sdft.h>

#include <debug.h>

#include <unity.h>


void setUp()
{
}

void tearDown()
{
}

static constexpr dynamic_notch_filter_config_t config = {
    .dyn_notch_min_hz = 100,
    .dyn_notch_max_hz = 600,
    .dyn_notch_q = 300,
    .dyn_notch_count = 3,
    .dyn_notch_smoothing = 0,
};

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_sdft()
{
    enum { SDFT_SAMPLE_COUNT = 72, SDFT_BIN_COUNT = SDFT_SAMPLE_COUNT/2 };
    Sdft<SDFT_SAMPLE_COUNT> sdft;

    constexpr float R = 0.9999F;
    const float m = static_cast<float>(M_PI) / static_cast<float>(SDFT_BIN_COUNT);
    TEST_ASSERT_EQUAL_FLOAT(R*cosf(12.0F*m), sdft.get_twiddle(12).real());
    TEST_ASSERT_EQUAL_FLOAT(R*sinf(12.0F*m), sdft.get_twiddle(12).imag());

    const uint32_t looptimeUs = 125;

    const float looptime_seconds = static_cast<float>(looptimeUs) * 1E-6F;
    const float looprate_hz = 1.0F / looptime_seconds;
    TEST_ASSERT_EQUAL_FLOAT(8000.0F, looprate_hz);

    const float nyquist_frequency_hz = looprate_hz / 2.0F;
    const float min_hz = config.dyn_notch_min_hz;
    float max_hz = std::fmaxf(min_hz, config.dyn_notch_max_hz);

    max_hz = std::fminf(max_hz, nyquist_frequency_hz); // Ensure to not go above the nyquist limit
    TEST_ASSERT_EQUAL(600, max_hz);

    const size_t sample_count = std::max(1, static_cast<int>(nyquist_frequency_hz / max_hz)); // max_hz = 600 & looprate_hz = 8000 -> sample_count = 6
    TEST_ASSERT_EQUAL(6, sample_count);

    const float sample_count_reciprocal = 1.0F / static_cast<float>(sample_count);
    const float sample_rate_hz = looprate_hz * sample_count_reciprocal;
    const float resolutionHz = sample_rate_hz / static_cast<float>(SDFT_SAMPLE_COUNT); // 18.5hz per bin at 8k and 600Hz max_hz

    const size_t start_bin = lrintf(min_hz / resolutionHz);
    TEST_ASSERT_EQUAL(5, start_bin);

    const size_t end_bin = lrintf(max_hz / resolutionHz);
    TEST_ASSERT_EQUAL(32, end_bin);

    sdft.init(start_bin, end_bin, sample_count);
    TEST_ASSERT_EQUAL(4, sdft.get_batch_size());
    TEST_ASSERT_EQUAL(6, sdft.get_batch_count());
    TEST_ASSERT_EQUAL(0, sdft.get_index());

    sdft.push(1000, 0);
    TEST_ASSERT_EQUAL(0, sdft.get_index());
    sdft.push(1002, 2);
    TEST_ASSERT_EQUAL(0, sdft.get_index());
    sdft.push(1003, 3);
    TEST_ASSERT_EQUAL(0, sdft.get_index());
    sdft.push(1004, 4);
    TEST_ASSERT_EQUAL(0, sdft.get_index());
    sdft.push(1005, 5);
    TEST_ASSERT_EQUAL(1, sdft.get_index());
    sdft.push(1006, 6);
    TEST_ASSERT_EQUAL(1, sdft.get_index());
    sdft.push(1007, 7);
    TEST_ASSERT_EQUAL(1, sdft.get_index());
    sdft.push(1008, 8);
    TEST_ASSERT_EQUAL(1, sdft.get_index());


    // 100Hz to 600Hz
    sdft.init(start_bin, end_bin, sample_count);

    const float signalFrequency = 140.0F;
    static constexpr auto M_PI_F = static_cast<float>(M_PI);

    static std::array<float, SDFT_SAMPLE_COUNT> samples;

    const float ratio = 2.0F * M_PI_F * signalFrequency / looprate_hz; // Fraction of a complete cycle stored at each sample (in radians)
    for (int ii = 0; ii < SDFT_SAMPLE_COUNT; ++ii) {
        const float x = static_cast<float>(ii) * ratio;
        samples[ii] = 2*sinf(x) + 2*sinf(2*x) + 2*sinf(3*x) + 0*sinf(4*x);
    }

    size_t sample_index = 0;
    for (int ii = 0; ii < SDFT_SAMPLE_COUNT; ++ii) {
        sdft.push(samples[ii], sample_index);
        ++sample_index;
        if (sample_index == sample_count) {
            sample_index = 0;
        }
    }
    static std::array<float, SDFT_BIN_COUNT> sdft_data;
    sdft_data.fill(0.0F);
    sdft.calculate_window_squared(&sdft_data[0]);


#if false
    std::array<char, 256> buf;
    for (int ii = 0; ii < 36; ++ii) {
        sprintf(&buf[0], "%2d=%3.1f, ", ii, static_cast<double>(sdft_data[ii])); UnityPrint(&buf[0]);
    }
#endif
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdft_data[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdft_data[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdft_data[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdft_data[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdft_data[4]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdft_data[33]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdft_data[34]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdft_data[35]);
}

void test_dynamic_notch_filter()
{
    const float looptime_seconds = 125.0F / 1000000.0F;

    static Debug debug;
    static DynamicNotchFilter dynamic_notch_filter(looptime_seconds);

    dynamic_notch_filter.set_config(config);

    TEST_ASSERT_EQUAL(6, dynamic_notch_filter.get_sample_count());
    TEST_ASSERT_EQUAL(5, dynamic_notch_filter.get_start_bin());
    TEST_ASSERT_EQUAL(32, dynamic_notch_filter.get_end_bin());
    TEST_ASSERT_EQUAL_FLOAT(18.51852F, dynamic_notch_filter.get_resolution_hz());

    // after set_config notch frequencies are evenly distributed in the range [100, 600]
    const float* center_frequency_hz = dynamic_notch_filter.getCenter_frequency_hzX();
    TEST_ASSERT_EQUAL_FLOAT(183.3333F, center_frequency_hz[0]);
    TEST_ASSERT_EQUAL_FLOAT(350.0F, center_frequency_hz[1]);
    TEST_ASSERT_EQUAL_FLOAT(516.6666F, center_frequency_hz[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, center_frequency_hz[3]);

    std::array<float, DynamicNotchFilter::SDFT_BIN_COUNT> testSdftData = {
        // peaks at 6(5.0F), 11(8.0F), 15(0.5F), 23(2.0F)
        // 0    1     2     3     4     5     6     7     8     9
        1.0F, 8.0F, 2.0F, 6.0F, 5.0F, 3.0F, 5.0F, 2.0F, 1.0F, 0.0F,
        0.0F, 8.0F, 0.0F, 0.0F, 0.1F, 0.5F, 0.2F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 1.0F, 2.0F, 1.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    };
    auto& sdft_data = dynamic_notch_filter.getSdftData();
    std::copy_n(testSdftData.begin(), testSdftData.size(), sdft_data.begin());

    dynamic_notch_filter.set_state({DynamicNotchFilter::STEP_DETECT_PEAKS, DynamicNotchFilter::X});
    dynamic_notch_filter.update_notch_frequencies(debug);
    DynamicNotchFilter::state_t state = dynamic_notch_filter.get_state();
    TEST_ASSERT_EQUAL(DynamicNotchFilter::STEP_CALCULATE_FREQUENCIES, state.step);
    TEST_ASSERT_EQUAL(DynamicNotchFilter::X, state.axis);

    // Check that peaks are sorted into ascending BIN order
    // dyn_notch_count = 3, so 4th peak should be ignored
    auto peaks = dynamic_notch_filter.get_peaks();
    TEST_ASSERT_EQUAL_FLOAT(5.0F, peaks[0].value);
    TEST_ASSERT_EQUAL(6, peaks[0].bin);
    TEST_ASSERT_EQUAL_FLOAT(8.0F, peaks[1].value);
    TEST_ASSERT_EQUAL(11, peaks[1].bin);
    TEST_ASSERT_EQUAL_FLOAT(2.0F, peaks[2].value);
    TEST_ASSERT_EQUAL(23, peaks[2].bin);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, peaks[3].value);
    TEST_ASSERT_EQUAL(0, peaks[3].bin);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, peaks[4].value);
    TEST_ASSERT_EQUAL(0, peaks[4].bin);

    dynamic_notch_filter.update_notch_frequencies(debug);
    state = dynamic_notch_filter.get_state();
    TEST_ASSERT_EQUAL(DynamicNotchFilter::STEP_UPDATE_FILTERS, state.step);
    TEST_ASSERT_EQUAL(DynamicNotchFilter::X, state.axis);

    if (config.dyn_notch_smoothing) {
        TEST_ASSERT_EQUAL_FLOAT(180.6423F, center_frequency_hz[0]); // bin 6
        TEST_ASSERT_EQUAL_FLOAT(344.6851F, center_frequency_hz[1]); // bin 11
        TEST_ASSERT_EQUAL_FLOAT(513.4822F, center_frequency_hz[2]); // bin 23
        TEST_ASSERT_EQUAL_FLOAT(0.0F, center_frequency_hz[3]);
    } else {
        TEST_ASSERT_EQUAL_FLOAT(109.2592F, center_frequency_hz[0]); // bin 6 =  5.89   * 18.51852, peak skewed to bin 5
        TEST_ASSERT_EQUAL_FLOAT(203.7037F, center_frequency_hz[1]); // bin 11 = 11     * 18.51852, peak centered on bin 11
        TEST_ASSERT_EQUAL_FLOAT(429.0123F, center_frequency_hz[2]); // bin 23 = 23.166 * 18.51852, peak skewed to bin 24
        TEST_ASSERT_EQUAL_FLOAT(0.0F, center_frequency_hz[3]);
    }

}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

// See for example for testing GCR encoding https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
// see also https://elmagnifico.tech/2023/04/07/bi-directional-DSHOT/
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_sdft);
    RUN_TEST(test_dynamic_notch_filter);

    UNITY_END();
}
