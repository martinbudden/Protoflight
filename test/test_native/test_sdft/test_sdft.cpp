#include <Debug.h>
#include <DynamicNotchFilter.h>
#include <IMU_Filters.h> // test code won't build if this not included
#include <SDFT.h>
#include <unity.h>


void setUp()
{
}

void tearDown()
{
}

static constexpr DynamicNotchFilter::config_t config = {
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
    SDFT<SDFT_SAMPLE_COUNT> sdft;

    const uint32_t looptimeUs = 125;

    const float looptimeSeconds = static_cast<float>(looptimeUs) * 1E-6F;
    const float looprateHz = 1.0F / looptimeSeconds;
    TEST_ASSERT_EQUAL_FLOAT(8000.0F, looprateHz);

    const float nyquistFrequencyHz = looprateHz / 2.0F;
    const float minHz = config.dyn_notch_min_hz;
    float maxHz = std::fmaxf(minHz, config.dyn_notch_max_hz);

    maxHz = std::fminf(maxHz, nyquistFrequencyHz); // Ensure to not go above the nyquist limit
    TEST_ASSERT_EQUAL(600, maxHz);

    const size_t sampleCount = std::max(1, static_cast<int>(nyquistFrequencyHz / maxHz)); // maxHz = 600 & looprateHz = 8000 -> sampleCount = 6
    TEST_ASSERT_EQUAL(6, sampleCount);

    const float sampleCountReciprocal = 1.0F / static_cast<float>(sampleCount);
    const float sampleRateHz = looprateHz * sampleCountReciprocal;
    const float resolutionHz = sampleRateHz / static_cast<float>(SDFT_SAMPLE_COUNT); // 18.5hz per bin at 8k and 600Hz maxHz

    const size_t startBin = lrintf(minHz / resolutionHz);
    TEST_ASSERT_EQUAL(5, startBin);

    const size_t endBin = lrintf(maxHz / resolutionHz);
    TEST_ASSERT_EQUAL(32, endBin);

    sdft.init(startBin, endBin, sampleCount);
    TEST_ASSERT_EQUAL(4, sdft.getBatchSize());
    TEST_ASSERT_EQUAL(6, sdft.getBatchCount());
    TEST_ASSERT_EQUAL(0, sdft.getIndex());

    sdft.push(1000, 0);
    TEST_ASSERT_EQUAL(0, sdft.getIndex());
    sdft.push(1002, 2);
    TEST_ASSERT_EQUAL(0, sdft.getIndex());
    sdft.push(1003, 3);
    TEST_ASSERT_EQUAL(0, sdft.getIndex());
    sdft.push(1004, 4);
    TEST_ASSERT_EQUAL(0, sdft.getIndex());
    sdft.push(1005, 5);
    TEST_ASSERT_EQUAL(1, sdft.getIndex());
    sdft.push(1006, 6);
    TEST_ASSERT_EQUAL(1, sdft.getIndex());
    sdft.push(1007, 7);
    TEST_ASSERT_EQUAL(1, sdft.getIndex());
    sdft.push(1008, 8);
    TEST_ASSERT_EQUAL(1, sdft.getIndex());


    // 100Hz to 600Hz
    sdft.init(startBin, endBin, sampleCount);

    const float signalFrequency = 140.0F;
    static constexpr auto M_PI_F = static_cast<float>(M_PI);

    static std::array<float, SDFT_SAMPLE_COUNT> samples;

    const float ratio = 2.0F * M_PI_F * signalFrequency / looprateHz; // Fraction of a complete cycle stored at each sample (in radians)
    for (int ii = 0; ii < SDFT_SAMPLE_COUNT; ++ii) {
        const float x = static_cast<float>(ii) * ratio;
        samples[ii] = 2*sinf(x) + 2*sinf(2*x) + 2*sinf(3*x) + 0*sinf(4*x);
    }

    size_t sampleIndex = 0;
    for (int ii = 0; ii < SDFT_SAMPLE_COUNT; ++ii) {
        sdft.push(samples[ii], sampleIndex);
        ++sampleIndex;
        if (sampleIndex == sampleCount) {
            sampleIndex = 0;
        }
    }
    static std::array<float, SDFT_BIN_COUNT> sdftData;
    sdftData.fill(0.0F);
    sdft.calculateWindowSquared(&sdftData[0]);


#if false
    std::array<char, 256> buf;
    for (int ii = 0; ii < 36; ++ii) {
        sprintf(&buf[0], "%2d=%3.1f, ", ii, static_cast<double>(sdftData[ii])); UnityPrint(&buf[0]);
    }
#endif
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdftData[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdftData[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdftData[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdftData[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdftData[4]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdftData[33]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdftData[34]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sdftData[35]);
}

void test_dynamic_notch_filter()
{
    const uint32_t looptimeUs = 125;

    static Debug debug;
    static DynamicNotchFilter dynamicNotchFilter(debug, looptimeUs);

    dynamicNotchFilter.setConfig(config);

    TEST_ASSERT_EQUAL(6, dynamicNotchFilter.getSampleCount());
    TEST_ASSERT_EQUAL(5, dynamicNotchFilter.getStartBin());
    TEST_ASSERT_EQUAL(32, dynamicNotchFilter.getEndBin());
    TEST_ASSERT_EQUAL_FLOAT(18.51852F, dynamicNotchFilter.getResolutionHz());

    // after setConfig notch frequencies are evenly distributed in the range [100, 600]
    const float* centerFrequencyHz = dynamicNotchFilter.getCenterFrequencyHzX();
    TEST_ASSERT_EQUAL_FLOAT(183.3333F, centerFrequencyHz[0]);
    TEST_ASSERT_EQUAL_FLOAT(350.0F, centerFrequencyHz[1]);
    TEST_ASSERT_EQUAL_FLOAT(516.6666F, centerFrequencyHz[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, centerFrequencyHz[3]);

    std::array<float, DynamicNotchFilter::SDFT_BIN_COUNT> testSdftData = {
        // peaks at 6(5.0F), 11(8.0F), 15(0.5F), 23(2.0F)
        // 0    1     2     3     4     5     6     7     8     9
        1.0F, 8.0F, 2.0F, 6.0F, 5.0F, 3.0F, 5.0F, 2.0F, 1.0F, 0.0F, 
        0.0F, 8.0F, 0.0F, 0.0F, 0.1F, 0.5F, 0.2F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 1.0F, 2.0F, 1.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    };
    auto& sdftData = dynamicNotchFilter.getSdftData();
    std::copy_n(testSdftData.begin(), testSdftData.size(), sdftData.begin());

    dynamicNotchFilter.setState({DynamicNotchFilter::STEP_DETECT_PEAKS, DynamicNotchFilter::X});
    dynamicNotchFilter.updateNotchFrequencies();
    DynamicNotchFilter::state_t state = dynamicNotchFilter.getState();
    TEST_ASSERT_EQUAL(DynamicNotchFilter::STEP_CALCULATE_FREQUENCIES, state.step);
    TEST_ASSERT_EQUAL(DynamicNotchFilter::X, state.axis);

    // Check that peaks are sorted into ascending BIN order
    // dyn_notch_count = 3, so 4th peak should be ignored
    auto peaks = dynamicNotchFilter.getPeaks();
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

    dynamicNotchFilter.updateNotchFrequencies();
    state = dynamicNotchFilter.getState();
    TEST_ASSERT_EQUAL(DynamicNotchFilter::STEP_UPDATE_FILTERS, state.step);
    TEST_ASSERT_EQUAL(DynamicNotchFilter::X, state.axis);

    if (config.dyn_notch_smoothing) {
        TEST_ASSERT_EQUAL_FLOAT(180.6423F, centerFrequencyHz[0]); // bin 6 
        TEST_ASSERT_EQUAL_FLOAT(344.6851F, centerFrequencyHz[1]); // bin 11
        TEST_ASSERT_EQUAL_FLOAT(513.4822F, centerFrequencyHz[2]); // bin 23
        TEST_ASSERT_EQUAL_FLOAT(0.0F, centerFrequencyHz[3]);
    } else {
        TEST_ASSERT_EQUAL_FLOAT(109.2592F, centerFrequencyHz[0]); // bin 6 =  5.89   * 18.51852, peak skewed to bin 5
        TEST_ASSERT_EQUAL_FLOAT(203.7037F, centerFrequencyHz[1]); // bin 11 = 11     * 18.51852, peak centered on bin 11
        TEST_ASSERT_EQUAL_FLOAT(429.0123F, centerFrequencyHz[2]); // bin 23 = 23.166 * 18.51852, peak skewed to bin 24
        TEST_ASSERT_EQUAL_FLOAT(0.0F, centerFrequencyHz[3]);
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
