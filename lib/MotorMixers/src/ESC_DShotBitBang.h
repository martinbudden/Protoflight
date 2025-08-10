#include "DShotCodec.h"
#include <array>
#include <cstddef>
#include <cstdint>

#if defined(USE_ARDUINO_STM32)
#define BIT_BANGING_V1
#else
#define GPIO_BSRR_BR_0 (0x1UL << (16U))
#define GPIO_BSRR_BS_0 (0x1UL << (0U))
#endif

enum { DSHOT_MODE = 300 };              // 150/300/600/1200
enum { DSHOT_BB_BUFFER_LENGTH = 18 };   // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
enum { BDSHOT_RESPONSE_BITRATE = DSHOT_MODE * 4 / 3 };    // in SymonB's tests this value was not 5/4 * DSHOT_MODE as documentation suggests but 4/3*DSHOT_MODE
enum { BDSHOT_RESPONSE_LENGTH = 21 };   // number of bits in ESC response
enum { DSHOT_BUFFER_LENGTH = 18 };      // 16 bits of Dshot and 2 for clearing

#if defined(BIT_BANGING_V2)
    enum { DSHOT_BB_0_LENGTH = 26 };        // number of sections for 0-bit, BITBANG_0_LENGTH
    enum { DSHOT_BB_1_LENGTH = 13 };        // number of sections for 1-bit, BITBANG_1_LENGTH
    enum { DSHOT_BB_FRAME_LENGTH = 35 };    // how many counts of the timer gives one bit frame
    enum { DSHOT_BB_FRAME_SECTIONS = 3 };   // in how many sections is bit frame divided
#else
    enum { DSHOT_BB_0_LENGTH = 10 };        // number of sections for 0-bit, BITBANG_0_LENGTH
    enum { DSHOT_BB_1_LENGTH = 4 };         // number of sections for 1-bit, BITBANG_1_LENGTH
    enum { DSHOT_BB_FRAME_LENGTH = 140 };   // how many counts of the timer gives one bit frame
    enum { DSHOT_BB_FRAME_SECTIONS = 14 };  // in how many sections is bit frame divided (for V1 must be factor of DSHOT_BB_FRAME_LENGTH)
#endif

class ESC_DShotBitbang {
public:
    enum { MOTOR_1 = 3 }; // PA3
    enum { MOTOR_2 = 0 }; // PB0
    enum { MOTOR_3 = 1 }; // PB1
    enum { MOTOR_4 = 2 }; // PA2
    enum { MOTOR_COUNT = 4 };  // number of motors

    static constexpr uint16_t RESPONSE_OVERSAMPLING = 3;  // it has to be a factor of DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BIDIRECTIONAL_DSHOT_RESPONSE_BITRATE

    ESC_DShotBitbang();
    void init();
    void presetDMA_outputBuffers();
    void setDMA_outputBuffers(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
    void presetDMA_outputBuffersV1();
    void setDMA_outputBuffersV1(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
    void presetDMA_outputBuffersV2();
    void setDMA_outputBuffersV2(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);

    void outputToMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
    void update_motors_rpm();

    static uint32_t samples_to_GCR21(const uint32_t* samples, uint32_t motorMask);
    static void GCR21_to_samples(uint32_t* samples, uint32_t motorMask, uint32_t gcr21); // for test code
    void read_BDshot_response(uint32_t gcr21, uint8_t motor);
    static uint16_t prepare_BDshot_package(uint16_t value) {
        // value is in range of 2000-4000 so need to transform it into Dshot range (48-2047)
        value -= 1953;
        if (value > 0 && value < 48) {
            value = 48;
        }
        return ((value << 5) | DShotCodec::checksumBidirectional(value<<1));
    }
    int32_t getMotorERPM(size_t motorIndex) { return _eRPMs[motorIndex]; }
public:
    static ESC_DShotBitbang* self; // alias of `this` to be used in ISR

private:
    std::array<int32_t, MOTOR_COUNT> _eRPMs {};
    std::array<int32_t, MOTOR_COUNT> _motorErrors {};
public:
    // flags for reception or transmission:
    bool bdshot_reception_1 {true};
    bool bdshot_reception_2 {true};
    // BDSHOT response is being sampled just after transmission. There is ~33 [us] break before response (additional sampling) and bitrate is increased by 5/4:
    // 2 receiving buffers, one for motors 1&4, the other for motors 2&3
    std::array<uint32_t, (int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * RESPONSE_OVERSAMPLING> dma_input_buffer_1_4 {};
    std::array<uint32_t, (int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * RESPONSE_OVERSAMPLING> dma_input_buffer_2_3 {};
private:
    // 2 sending buffers, one for motors 1&4, the other for motors 2&3
    std::array<uint32_t, DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS> dma_output_buffer_1_4 {};
    std::array<uint32_t, DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS> dma_output_buffer_2_3 {};
};
