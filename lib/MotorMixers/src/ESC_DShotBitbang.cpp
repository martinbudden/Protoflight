
#include "ESC_DShotBitbangIRQ.h"
#include <algorithm>
#include <array>

#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
void ESC_DShotBitbang::IRQ_Handler(port_t& port)
{
#if !defined(FRAMEWORK_STM32_CUBE_F1)
    // set GPIOs as inputs:
    //port.GPIO->MODER &= ~GPIO_MODER_MODER2;
    //port.GPIO->MODER &= ~GPIO_MODER_MODER3;
    port.GPIO->MODER &= port.GPIO_input;
    // set pull up for those pins:
    port.GPIO->PUPDR |= port.GPIO_PUPDR;

    // set timer:
    port.TIM->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / ESC_DShotBitbang::RESPONSE_OVERSAMPLING - 1;
    port.TIM->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / ESC_DShotBitbang::RESPONSE_OVERSAMPLING;

    // Set DMA to copy GPIOA->IDR register value to the _dmaInputBufferA buffer).
    port.DMA_Stream->CR &= ~(DMA_SxCR_DIR);
    port.DMA_Stream->PAR = reinterpret_cast<uint32_t>(&(GPIOA->IDR)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    port.DMA_Stream->M0AR = reinterpret_cast<uint32_t>(&ESC_DShotBitbang::self->_portA.dmaInputBuffer[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // Main idea:
    // After sending DShot frame to ESC start receiving GPIO values.
    // Capture data (probing longer than ESC response).
    // There is ~33 [us] gap before the response so it is necessary to add more samples:
    // NDTR: number of data register
    port.DMA_Stream->NDTR = (33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * ESC_DShotBitbang::RESPONSE_OVERSAMPLING;

    port.DMA_Stream->CR |= DMA_SxCR_EN;
#endif
    port.reception = false;
}
#endif

ESC_DShotBitbang::ESC_DShotBitbang()
{
    self = this;
}

void ESC_DShotBitbang::init()
{
    presetDMA_outputBuffers();
#if (defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32))

    _portA.GPIO = GPIOA;
    _portB.GPIO = GPIOB;
 #if !defined(FRAMEWORK_STM32_CUBE_F1)
    _portB.GPIO_input = ~(GPIO_MODER_MODER2   | GPIO_MODER_MODER3);
    _portA.GPIO_output =  GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0;
    _portA.GPIO_PUPDR =   GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0;
    setupGPIO(_portA.GPIO, RCC_AHB1ENR_GPIOAEN, GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3);

     _portB.GPIO_input = ~(GPIO_MODER_MODER0   | GPIO_MODER_MODER1);
    _portB.GPIO_output =  GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0;
    _portB.GPIO_PUPDR =   GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;
    setupGPIO(_portB.GPIO, RCC_AHB1ENR_GPIOBEN, GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1);

    _portA.DMA_Stream = DMA2_Stream6;
    setupDMA(_portA.DMA_Stream, RCC_AHB1ENR_DMA1EN);
    _portB.DMA_Stream = DMA2_Stream2;
    setupDMA(_portB.DMA_Stream, RCC_AHB1ENR_DMA2EN);
#endif

    _portA.TIM = TIM1;
    setupTimers(_portA.TIM, RCC_APB2ENR_TIM1EN);
#if defined(FRAMEWORK_STM32_CUBE_F1)
    _portB.TIM = TIM4;
    setupTimers(_portB.TIM, RCC_APB1ENR_TIM4EN);
#else
    _portB.TIM = TIM8;
    setupTimers(_portB.TIM, RCC_APB2ENR_TIM8EN);
    // Nested Vectored Interrupt Controller
    // enable DMA interrupts
    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    NVIC_SetPriority(DMA2_Stream6_IRQn, 13);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_SetPriority(DMA2_Stream2_IRQn, 14);
#endif
#endif
}

/* IMPORTANT:
APB2 max frequency is 84 [MHz], 168 [MHz] only for timers
APB1 max frequency is 42 [MHz], 84 [MHz] only for timers
*/

#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
void ESC_DShotBitbang::setupGPIO(GPIO_TypeDef* GPIO, uint32_t GPIOxEN, uint32_t GPIO_OSPEEDER_OSPEEDRn)
{
#if defined(FRAMEWORK_STM32_CUBE_F1)
#else
    // enable GPIOA clock:
    RCC->AHB1ENR |= GPIOxEN;
    // mode (00-input; 01-output; 10-alternate) will be set later
    // set speed (max speed):
    GPIO->OSPEEDR |= GPIO_OSPEEDER_OSPEEDRn;
#endif
}

#if (defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)) && !defined(FRAMEWORK_STM32_CUBE_F1)
void ESC_DShotBitbang::setupDMA(DMA_Stream_TypeDef* DMA_Stream, uint32_t DMAxEN)
{
    RCC->AHB1ENR |= DMAxEN;

    DMA_Stream->CR = 0x0;
    while (DMA_Stream->CR & DMA_SxCR_EN) {
        ; // wait
    }
    DMA_Stream->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
    // all the other parameters will be set later
}
#endif

void ESC_DShotBitbang::setupTimers(TIM_TypeDef* TIM, uint32_t TIMxEN)
{
// TIM - only for generating time basement all outputs are set by GPIOs:
    // enable TIM1 clock:
    // RCC: Reset and Clock Control
    RCC->APB2ENR |= TIMxEN;
    // register is buffered and overflow DMA request:

    // CR: control register
    TIM->CR1 = 0x0;
    TIM->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

    // DMA request:
    // DIER: DMA/interrupt enable register
    // CCR: capture/compare register
    // PSC: prescaler
    // ARR: auto-reload register
    TIM->DIER |= TIM_DIER_CC1DE; // CC1DE: Capture/Compare 1 DMA request enable
    // TIM1 is 168 MHz:
    TIM->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
#if defined(BIT_BANGING_V1)
    TIM->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
    TIM->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
#elif defined(BIT_BANGING_V2)
    TIM->DIER |= TIM_DIER_CC2DE; // channel 2 request
    TIM->DIER |= TIM_DIER_CC3DE; // channel 3 request
    TIM->CCR1 = 0;
    TIM->CCR2 = DSHOT_BB_0_LENGTH;
    TIM->CCR3 = DSHOT_BB_1_LENGTH;
    TIM->ARR = DSHOT_BB_FRAME_LENGTH - 1;
#endif

    // TIM enable:
    // EGR: event generation register
    TIM->EGR |= TIM_EGR_UG;  // UG: Update Generation
    TIM->CR1 |= TIM_CR1_CEN; //CEN: counter enable
}
#endif //FRAMEWORK_STM32

// values should be in the DShot range [47,2047]
void ESC_DShotBitbang::outputToMotors(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value)
{
    update_motors_rpm();

    setDMA_outputBuffers(
        DShotCodec::frameBidirectional(m1_value),
        DShotCodec::frameBidirectional(m2_value),
        DShotCodec::frameBidirectional(m3_value),
        DShotCodec::frameBidirectional(m4_value)
    );

    _portA.reception = true;
    _portB.reception = true;

#if (defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)) && !defined(FRAMEWORK_STM32_CUBE_F1)
    // set GPIOs as output:
    // MODER: GPIO port mode register
    _portA.GPIO->MODER |= _portA.GPIO_output;
    _portB.GPIO->MODER |= _portB.GPIO_output;

    // CR:   DMA stream x configuration register
    // NDTR: DMA stream x number of data register
    // PAR:  DMA stream x peripheral address register
    // M0AR: DMA stream x memory 0 address register
    _portA.DMA_Stream->CR |= DMA_SxCR_DIR_0;
    _portA.DMA_Stream->PAR = reinterpret_cast<uint32_t>(&(_portA.GPIO->BSRR));
    _portA.DMA_Stream->M0AR = reinterpret_cast<uint32_t>(&_portA.dmaOutputBuffer[0]);
    _portA.DMA_Stream->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

    _portB.DMA_Stream->CR |= DMA_SxCR_DIR_0;
    _portB.DMA_Stream->PAR = reinterpret_cast<uint32_t>(&(_portB.GPIO->BSRR));
    _portB.DMA_Stream->M0AR = reinterpret_cast<uint32_t>(&_portB.dmaOutputBuffer[0]);
    _portB.DMA_Stream->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

#if defined(BIT_BANGING_V1)
    // Main idea:
    // Every bit frame is divided in sections and for each section DMA request is generated.
    // After some sections (at beginning, after 0-bit time and after 1-bit time) to GPIO register can be sent value to set 1 or to set 0.
    // For rest of the sections 0x0 is sent so GPIOs don't change values.
    // It uses only 1 CCR on each timer.
    // Idea for reception is the same.

    // portA TIM setup:
    _portA.TIM->CR1 &= ~TIM_CR1_CEN;
    _portA.TIM->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
    _portA.TIM->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

    // porB TIM setup:
    _portB.TIM->CR1 &= ~TIM_CR1_CEN;
    _portB.TIM->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
    _portB.TIM->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

    // send
    _portA.DMA_Stream->CR |= DMA_SxCR_EN;
    _portB.DMA_Stream->CR |= DMA_SxCR_EN;

    // EGR: event generation register
    // CR: Control Register
    _portA.TIM->EGR |= TIM_EGR_UG; // Update Generation
    _portA.TIM->CR1 |= TIM_CR1_CEN; // Counter Enable
    _portB.TIM->EGR |= TIM_EGR_UG;
    _portB.TIM->CR1 |= TIM_CR1_CEN;
#elif defined(BIT_BANGING_V2)
    // Main idea:
    // DMA requests generated at beginning, after 0-bit time and after 1-bit time
    // for each bit there is only 3 sections -> buffers are much smaller than in version 1
    // but uses 3 CCR (capture/compare register) for each timer (probably not a big deal)
    // It works for transferring but for reception it is not useful

    //    TIM1 setup:
    _portA.TIM->CCR1 = 0;
    _portA.TIM->CCR2 = DSHOT_BB_0_LENGTH;
    _portA.TIM->CCR3 = DSHOT_BB_1_LENGTH;
    _portA.TIM->ARR = DSHOT_BB_FRAME_LENGTH - 1;

    // TIM8 setup:
    _portB.TIM->CCR1 = 0;
    _portB.TIM->CCR2 = DSHOT_BB_0_LENGTH;
    _portB.TIM->CCR3 = DSHOT_BB_1_LENGTH;
    _portB.TIM->ARR = DSHOT_BB_FRAME_LENGTH - 1;

    //  send:
    _portA.TIM->EGR |= TIM_EGR_UG;
    _portA.TIM->CR1 |= TIM_CR1_CEN;
    _portB.TIM->EGR |= TIM_EGR_UG;
    _portB.TIM->CR1 |= TIM_CR1_CEN;

    _portA.DMA_Stream->CR |= DMA_SxCR_EN;
    _portB.DMA_Stream->CR |= DMA_SxCR_EN;
#endif
#endif // FRAMEWORK_STM32
}

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)
/*!
Method 1 divides bit into 7 fixed length sections and changes GPIO value at one of these fixed times
This requires a larger output buffer, but fewer timer channels
The larger buffers increase the DMA load as well.

DSHOT_BB_FRAME_SECTIONS = 14
DSHOT_BB_0_LENGTH = 10
DSHOT_BB_1_LENGTH = 4
        01234567890123
0 bit = 00011111111111
1 bit = 00000000011111

Note bidirectional DShot is inverted, corresponding outputs for unidirectional DShot are:
0 bit = 11100000000000  T0H,T0L
1 bit = 11111111100000  T1H,T1L

DShot 300 specification is
    T0H = 1250ns (data low pulse width)
    T0L = 2090ns (data low gap width)
    T1H = 2500ns (data high pulse width)
    T1L =  840ns (data high gap width)
    TxH+TxL = 3340ns  (T0H + T0L or T1H + T1L)

actual timings are 3340*(10-1)/14 = 2147
actual timings are 3340*(4-1)/14  = 715

Note:
3340*10/14 = 2385
3340*10/16 = 2087
3340*4/14  = 954
3340*4/16 = 835

Method 2 uses 3 sections for each bit and variable length sections
0 bit = 011
1 bit = 001

This requires smaller buffers, has lower DMA load, has more precise timing, but requires more timer channels
*/

void ESC_DShotBitbang::presetDMA_outputBuffers()
{
#if (defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)) && !defined(FRAMEWORK_STM32_CUBE_F1)
    // this values are constant so they can be set once here
#if defined(BIT_BANGING_V1)
    _portA.dmaOutputBuffer.fill(0);
    _portB.dmaOutputBuffer.fill(0);

    // for DSHOT_BB_FRAME_SECTIONS=14, DSHOT_BB_1_LENGTH=10 we have
    // r means BR (Bit Reset) (ie set to 0)
    // s means BS (Bit Set) (ie set to 1)
    //                  01234567890123
    // buffer:          r00000000s0000
    // resultant GPIO:  00000000011111
    const size_t highOffset = DSHOT_BB_1_LENGTH - 1;
#else
    // make 2 high frames after Dshot frame:
    for (auto i = 0; i < DSHOT_BB_FRAME_SECTIONS * 2; i++) {
        _portA.dmaOutputBuffer[DSHOT_BUFFER_LENGTH*DSHOT_BB_FRAME_SECTIONS - i - 1] = (GPIO_BSRR_BS_0 << MOTOR_1) | (GPIO_BSRR_BS_0 << MOTOR_4);
        _portB.dmaOutputBuffer[DSHOT_BUFFER_LENGTH*DSHOT_BB_FRAME_SECTIONS - i - 1] = (GPIO_BSRR_BS_0 << MOTOR_2) | (GPIO_BSRR_BS_0 << MOTOR_3);
    }
    const size_t highOffset = 2;
#endif

    for (auto i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++) { // last 2 bits are always high (logic 0)
        const auto index = i * DSHOT_BB_FRAME_SECTIONS;
        //  first section always lower edge:
        _portA.dmaOutputBuffer[index] = (GPIO_BSRR_BR_0 << MOTOR_1) | (GPIO_BSRR_BR_0 << MOTOR_4);
        _portB.dmaOutputBuffer[index] = (GPIO_BSRR_BR_0 << MOTOR_2) | (GPIO_BSRR_BR_0 << MOTOR_3);

        // last section always rise edge:
        _portA.dmaOutputBuffer[index + highOffset] = (GPIO_BSRR_BS_0 << MOTOR_1) | (GPIO_BSRR_BS_0 << MOTOR_4);
        _portB.dmaOutputBuffer[index + highOffset] = (GPIO_BSRR_BS_0 << MOTOR_2) | (GPIO_BSRR_BS_0 << MOTOR_3);
    }
#endif
}

void ESC_DShotBitbang::setDMA_outputBuffers(uint16_t m1_frame, uint16_t m2_frame, uint16_t m3_frame, uint16_t m4_frame)
{
/*
For each output bit DMA is transfering DSHOT_BB_FRAME_SECTIONS (14) times data into GPIO register.
But we need to decide only about 3 values (begining, 0-bit time, 1-bit time).
For Rest values we send 0x0 into register so GPIOs stay the same.
Moreover each output bit is preset (lowering edge at first and rising edge after DSHOT_BB_1_LENGTH
remaining values are 0x0 so there will be no changes in GPIO output registers).
Now it is needed to only decide about rising edge after DSHOT_BB_0_LENGTH (if bit is 0)
or seting 0 so LOW state will stay until DSHOT_BB_1_LENGTH.

In addition last 2 frame bits are set always high
(ESC needs time for proper signal detection and those high values define end off the transmission).
*/
// METHOD1:
// note: bidirectional DShot is inverted so we have (eg) 0001111111111 rather than 111000000000
// for DSHOT_BB_FRAME_SECTIONS=14, DSHOT_BB_0_LENGTH=4 we have
//
// in case where bit in motor frame bit is set
//                      01234567890123
//     buffer (preset): r000000000s000
//     buffer (set):    r00s000000s000
//     resultant GPIO:  00011111111111
// in case where bit in motor frame bit is not set
//                      01234567890123
//     buffer (preset): r00000000s0000
//     buffer (set):    r00000000s0000
//     resultant GPIO:  00000000011111
//
// At end of loop we will have 16 frames of form r00s000000s000
// followed by 2 frames of form 00000000000000 (set in preset) - these two frames will keep output high

#if (defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)) && !defined(FRAMEWORK_STM32_CUBE_F1)

#if defined(BIT_BANGING_V1)
    size_t index = DSHOT_BB_0_LENGTH - 1;
#else
    size_t index = 1;
#endif
    for (uint32_t bitMask = 0x8000; bitMask !=0; bitMask >>= 1) {
        _portA.dmaOutputBuffer[index] = (bitMask & m1_frame) ? 0x00 : GPIO_BSRR_BS_0 << MOTOR_1;
        if (bitMask & m4_frame) {
            _portA.dmaOutputBuffer[index] |= GPIO_BSRR_BS_0 << MOTOR_4;
        }
        _portB.dmaOutputBuffer[index] = (bitMask & m2_frame) ? 0x00 : GPIO_BSRR_BS_0 << MOTOR_2;
        if (bitMask & m3_frame) {
            _portB.dmaOutputBuffer[index] |= GPIO_BSRR_BS_0 << MOTOR_3;
        }
        index += DSHOT_BB_FRAME_SECTIONS;
    }
#else
    (void)m1_frame;
    (void)m2_frame;
    (void)m3_frame;
    (void)m4_frame;
#endif
}

/*!
Decodes the samples in the raw buffer.

There are 5 samples per bit, so for 21 bits there are 105 samples - in the ideal case
0 is encoded as 0,0,1,1,1
1 is encoded as 0,0,0,0,1

In actual received data these may vary because of jitter.

Converts a buffer of samples (obtained from DMA) to a gcr21 value
*/
uint32_t ESC_DShotBitbang::samples_to_GCR21(const uint32_t* samples, uint32_t motorMask)
{

    uint16_t i = 0;
    uint16_t previous_i = 0;
    uint16_t end_i = 0;
    uint32_t previous_value = 1;

    // Reception starts just after transmission, so there is a lot of HIGH samples. Find first LOW bit:
    while (i < (33 * BDSHOT_RESPONSE_BITRATE / 1000 * RESPONSE_OVERSAMPLING)) {
        if (!(samples[i] & motorMask)) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            previous_value = 0;
            previous_i = i;
            end_i = i + BDSHOT_RESPONSE_LENGTH * RESPONSE_OVERSAMPLING;
            break;
        }
        i++;
    }
    if (previous_value == 1) {
        // LOW edge was not found so return incorrect motor response:
        return 0xFFFFFFFF;
    }

    // if LOW edge was detected:
    uint32_t bitCount = 0;
    uint32_t gcr21 = 0;
    while (i < end_i) {
        // then look for changes in bits values and compute BDSHOT bits:
        const uint32_t value = samples[i] & motorMask; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (value != previous_value) {
            const uint32_t rlen = (i - previous_i) / RESPONSE_OVERSAMPLING;
            const uint32_t len = (rlen > 1) ? rlen : 1; // how many bits had the same value
            bitCount += len;
            gcr21 <<= len;
            if (previous_value != 0) {
                gcr21 |= (0x1FFFFF >> (21 - len)); // 21 ones right-shifted by 20 or less
            }
            previous_value = value;
            previous_i = i;
        }
        i++;
    }
    // if last bits were 1 they were not added so far
    gcr21 <<= (BDSHOT_RESPONSE_LENGTH - bitCount);
    gcr21 |= 0x1FFFFF >> bitCount; // 21 ones right-shifted

    return gcr21;
}

/*!
Converts gcr21 value to a buffer of samples, used for test code
*/
void ESC_DShotBitbang::GCR21_to_samples(uint32_t* samples, uint32_t motorMask, uint32_t gcr21)
{
    *samples = 0;
    (void)motorMask;
    (void)gcr21;
}

void ESC_DShotBitbang::update_motors_rpm()
{
    // BDshot bit banging reads whole GPIO register.
    // Now it's time to create BDshot responses from all motors (made of individual bits).
    const std::array<uint32_t, 4> motorGCR21s  = {
        samples_to_GCR21(&_portA.dmaInputBuffer[0], 1 << MOTOR_1),
        samples_to_GCR21(&_portB.dmaInputBuffer[0], 1 << MOTOR_2),
        samples_to_GCR21(&_portB.dmaInputBuffer[0], 1 << MOTOR_3),
        samples_to_GCR21(&_portA.dmaInputBuffer[0], 1 << MOTOR_4)
    };
    for (size_t ii = 0; ii < motorGCR21s.size(); ++ii) {
        const uint32_t motorGCR20 = DShotCodec::GCR21_to_GCR20(motorGCR21s[ii]);
        const uint16_t eRPM = DShotCodec::GCR20_to_eRPM(motorGCR20);
        if (DShotCodec::checksumBidirectionalIsOK(eRPM)) {
            DShotCodec::telemetry_type_e telemetryType {};
            const auto eRPM_periodMicroseconds = DShotCodec::decodeTelemetryFrame(eRPM >> 4, telemetryType);
            enum { ONE_MINUTE_IN_MICROSECONDS = 60000000 };
            // value is eRPM period in microseconds
            _eRPMs[ii] = static_cast<int32_t>(ONE_MINUTE_IN_MICROSECONDS / eRPM_periodMicroseconds);
        }
    }

}

// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)
