#include "DShotCodec.h"
#include <array>
#include <cstddef>
#include <cstdint>
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)

#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal.h>
#endif

#endif

#define BIT_BANGING_V1

/*
Implementation ported from: https://github.com/symonb/Bidirectional-DSHOT-and-RPM-Filter/blob/main/Src/bdshot.c

Explanation of code at:     https://symonb.github.io/docs/drone/ESC/ESC_prot_impl_2_2

FLLF-KIWIF4 uses same pins

Motor pins etc still hard coded:
# resources
resource MOTOR 1 A03
resource MOTOR 2 B00
resource MOTOR 3 B01
resource MOTOR 4 A02

# timer
timer A03 AFx
# pin A03: TIM1 CH1 (AFx)
timer B00 AFx
# pin B00: TIM8 CH1 (AFx)
timer B01 AFx
# pin B01: TIM8 CH1 (AFx)
timer A02 AFx
# pin A02: TIM1 CH1 (AFx)

# dma
dma pin A03 1
# pin A03: DMA2 Stream 2 Channel 6
dma pin B00 1
# pin B00: DMA2 Stream 2 Channel 2
dma pin B01 1
# pin B01: DMA2 Stream 2 Channel 2
dma pin A02 1
# pin A02: DMA2 Stream 2 Channel 6
*/


enum { DSHOT_MODE = 300 };              // 150/300/600/1200
enum { DSHOT_FRAME_LENGTH = 16 };   // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
enum { DSHOT_BB_BUFFER_LENGTH = DSHOT_FRAME_LENGTH + 2 };   // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
enum { BDSHOT_RESPONSE_BITRATE = DSHOT_MODE * 4 / 3 };    // in SymonB's tests this value was not 5/4 * DSHOT_MODE as documentation suggests but 4/3*DSHOT_MODE
enum { BDSHOT_RESPONSE_LENGTH = 21 };   // number of bits in ESC response, ie length of GRC21 frame
enum { DSHOT_BUFFER_LENGTH = 18 };      // 16 bits of Dshot and 2 for clearing

#if defined(BIT_BANGING_V2)
    enum { DSHOT_BB_1_LENGTH = 26 };        // number of sections for 0-bit, BITBANG_0_LENGTH
    enum { DSHOT_BB_0_LENGTH = 13 };        // number of sections for 1-bit, BITBANG_1_LENGTH
    enum { DSHOT_BB_FRAME_SECTIONS = 3 };   // in how many sections is bit frame divided
    enum { DSHOT_BB_FRAME_LENGTH = 35 };    // how many counts of the timer gives one bit frame
#else
    enum { DSHOT_BB_1_LENGTH = 10 };        // number of sections for 1-bit, BITBANG_1_LENGTH
    enum { DSHOT_BB_0_LENGTH = 4 };         // number of sections for 0-bit, BITBANG_0_LENGTH
    enum { DSHOT_BB_FRAME_SECTIONS = 14 };  // in how many sections is bit frame divided
    enum { DSHOT_BB_FRAME_LENGTH = 140 };   // how many counts of the timer gives one bit frame (must be multiple of DSHOT_BB_FRAME_SECTIONS)
#endif

class ESC_DShotBitbang {
public:
    enum { MOTOR_1 = 3 }; // PA3
    enum { MOTOR_4 = 2 }; // PA2
    enum { MOTOR_2 = 0 }; // PB0
    enum { MOTOR_3 = 1 }; // PB1
    enum { MOTOR_COUNT = 4 };

    static constexpr uint16_t RESPONSE_OVERSAMPLING = 3;  // it has to be a factor of DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BIDIRECTIONAL_DSHOT_RESPONSE_BITRATE

    ESC_DShotBitbang();
    void init();
    void presetDMA_outputBuffers();
    void setDMA_outputBuffers(uint16_t m1_frame, uint16_t m2_frame, uint16_t m3_frame, uint16_t m4_frame);

    void outputToMotors(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value); // values should be in the DShot range [47,2047]
    void update_motors_rpm();
    int32_t getMotorERPM(size_t motorIndex) { return _eRPMs[motorIndex]; }

    static uint32_t samples_to_GCR21(const uint32_t* samples, uint32_t motorMask);
    static void GCR21_to_samples(uint32_t* samples, uint32_t motorMask, uint32_t gcr21); // for test code
public:
    static ESC_DShotBitbang* self; // alias of `this` to be used in ISR
    struct port_t {
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
        GPIO_TypeDef* GPIO;
        uint32_t GPIO_input;
        uint32_t GPIO_output;
        uint32_t GPIO_PUPDR;
#if !defined(FRAMEWORK_STM32_CUBE_F1)
        DMA_Stream_TypeDef* DMA_Stream;
#endif
        TIM_TypeDef* TIM;
#endif
        bool reception; // flag for reception or transmission:
        // BDSHOT response is being sampled just after transmission. There is ~33 [us] break before response (additional sampling) and bitrate is increased by 5/4:
        std::array<uint32_t, (int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * RESPONSE_OVERSAMPLING> dmaInputBuffer;
        std::array<uint32_t, DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS> dmaOutputBuffer;
    };
    port_t& getPortA() { return _portA; }
    port_t& getPortB() { return _portB; }
    static void IRQ_Handler(port_t& port);
private:
    std::array<int32_t, MOTOR_COUNT> _eRPMs {};
    std::array<int32_t, MOTOR_COUNT> _motorErrors {};
    port_t _portA {};
    port_t _portB {};
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static void setupGPIO(GPIO_TypeDef*GPIO, uint32_t GPIOxEN, uint32_t GPIO_OSPEEDER_OSPEEDRn);
#if !defined(FRAMEWORK_STM32_CUBE_F1)
    static void setupDMA(DMA_Stream_TypeDef* TIM, uint32_t DMAxEN);
#endif
    static void setupTimers(TIM_TypeDef* TIM, uint32_t TIMxEN);
#endif
};
