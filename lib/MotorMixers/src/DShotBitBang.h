#include <cstdint>

#if defined(USE_ARDUINO_STM32)
#define BIT_BANGING_V2
#else
#define GPIO_BSRR_BR_0 (0x1UL << (16U))
#define GPIO_BSRR_BS_0 (0x1UL << (0U))
#endif

enum { DSHOT_MODE = 300 };              // 150/300/600/1200
enum { DSHOT_BB_FRAME_LENGTH = 140 };   //	how many counts of the timer gives one bit frame
enum { DSHOT_BB_BUFFER_LENGTH = 18 };   // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
enum { BDSHOT_RESPONSE_BITRATE = DSHOT_MODE * 4 / 3 };    // in SymonB's tests this value was not 5/4 * DSHOT_MODE as documentation suggests but 4/3*DSHOT_MODE
enum { BDSHOT_RESPONSE_LENGTH = 21 };   // number of bits in ESC response
enum { DSHOT_BB_FRAME_SECTIONS = 7 };   // in how many sections is bit frame divided (must be factor of DSHOT_BB_FRAME_LENGTH)
enum { DSHOT_BUFFER_LENGTH = 18 };      // 16 bits of Dshot and 2 for clearing

static constexpr uint16_t BDSHOT_RESPONSE_OVERSAMPLING = 3;  // it has to be a factor of DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BIDIRECTIONAL_DSHOT_RESPONSE_BITRATE

class DShotBitBang {
public:
    enum { MOTOR_1 = 3 }; // PA3
    enum { MOTOR_2 = 0 }; // PB0
    enum { MOTOR_3 = 1 }; // PB1
    enum { MOTOR_4 = 2 }; // PA2
    enum { MOTOR_COUNT = 4 };  // number of motors

    enum { DSHOT_BB_0_LENGTH = 2 };// number of sections for 0-bit, BITBANG_0_LENGTH
    enum { DSHOT_BB_1_LENGTH = 5 }; // number of sections for 1-bit, BITBANG_1_LENGTH

    static void init();
    static void preset_bb_BDshot_buffersV1();
    static void preset_bb_BDshot_buffersV2();
    static void fill_bb_BDshot_bufferV1(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
    static void fill_bb_BDshot_bufferV2(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
    void update_motors();
    void update_motors_rpm();
    static uint32_t get_BDshot_response(const uint32_t* raw_buffer, const uint8_t motor_shift);
    void read_BDshot_response(uint32_t value, uint8_t motor);
    static uint16_t prepare_BDshot_package(uint16_t value) {
        // value is in range of 2000-4000 so need to transform it into Dshot range (48-2047)
        value -= 1953;
        if (value > 0 && value < 48) {
            value = 48;
        }
        return ((value << 5) | calculate_BDshot_checksum(value));
    }

    static uint16_t calculate_BDshot_checksum(uint16_t value) {
        // 12th bit for telemetry on/off (1/0):
        value <<= 1;
        return (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
    }
    static bool BDshot_check_checksum(uint16_t value) {
        // BDshot frame has 4 last bits CRC:
        return (((value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12)) & 0x0F) == 0x0F) ? true : false;
    }
public:
    // flags for reception or transmission:
    static bool bdshot_reception_1;
    static bool bdshot_reception_2;
    static uint32_t dshot_bb_buffer_1_4[DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS];
    static uint32_t dshot_bb_buffer_2_3[DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS];
    // BDSHOT response is being sampled just after transmission. There is ~33 [us] break before response (additional sampling) and bitrate is increased by 5/4:
    static uint32_t dshot_bb_buffer_1_4_r[(int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
    static uint32_t dshot_bb_buffer_2_3_r[(int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
private:
    uint32_t _motorPoleCount {14};
    uint16_t motor_1_value {};
    uint16_t motor_2_value {};
    uint16_t motor_3_value {};
    uint16_t motor_4_value {};

    // assign your variable addresses into pointers:
    uint16_t *motor_1_value_pointer = &motor_1_value;
    uint16_t *motor_2_value_pointer = &motor_2_value;
    uint16_t *motor_3_value_pointer = &motor_3_value;
    uint16_t *motor_4_value_pointer = &motor_4_value;
    uint32_t motors_rpm[MOTOR_COUNT] {};
    float motors_error[MOTOR_COUNT] {};
};
