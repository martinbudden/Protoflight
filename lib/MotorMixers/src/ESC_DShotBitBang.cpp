#include <ESC_DShotBitbang.h>
#include <algorithm>
#include <array>
#if defined(USE_ARDUINO_STM32)
#include <stm32f4xx.h>
#endif

/*
Implementation ported from: https://github.com/symonb/Bidirectional-DSHOT-and-RPM-Filter/blob/main/Src/bdshot.c

Explanation of code at:     https://symonb.github.io/docs/drone/ESC/ESC_prot_impl_2_2
*/


ESC_DShotBitbang* ESC_DShotBitbang::self; // alias of `this` to be used in ISR

#if defined(USE_ARDUINO_STM32)
void DMA2_Stream6_IRQHandler()
{
    // DMA:
    // HISR:  DMA high interrupt status register
    // HIFCR: DMA high interrupt flag clear register
    if (DMA2->HISR & DMA_HISR_TCIF6) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF6;

        if (ESC_DShotBitbang::self->_reception_1_4) {
            // set GPIOs as inputs:
            GPIOA->MODER &= ~GPIO_MODER_MODER2;
            GPIOA->MODER &= ~GPIO_MODER_MODER3;
            // set pull up for those pins:
            GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0;

            // set timer:
            TIM1->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / ESC_DShotBitbang::RESPONSE_OVERSAMPLING - 1;
            TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / ESC_DShotBitbang::RESPONSE_OVERSAMPLING;

            // Set DMA to copy GPIOA->IDR register value to the _dmaInputBuffer_1_4 buffer).
            DMA2_Stream6->CR &= ~(DMA_SxCR_DIR);
            DMA2_Stream6->PAR = (uint32_t)(&(GPIOA->IDR));
            DMA2_Stream6->M0AR = (uint32_t)(&ESC_DShotBitbang::self->_dmaInputBuffer_1_4[0]);
            // Main idea:
            // After sending DShot frame to ESC start receiving GPIO values.
            // Capture data (probing longer than ESC response).
            // There is ~33 [us] gap before the response so it is necessary to add more samples:
            // NDTR: number of data register
            DMA2_Stream6->NDTR = ((int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * ESC_DShotBitbang::RESPONSE_OVERSAMPLING);

            DMA2_Stream6->CR |= DMA_SxCR_EN;
            ESC_DShotBitbang::self->_reception_1_4 = false;
        }
    }

    if (DMA2->HISR & DMA_HISR_HTIF6) {
        DMA2->HIFCR |= DMA_HIFCR_CHTIF6;
    }
    if (DMA2->HISR & DMA_HISR_DMEIF6) {
        DMA2->HIFCR |= DMA_HIFCR_CDMEIF6;
    }
    if (DMA2->HISR & DMA_HISR_TEIF6) {
        DMA2->HIFCR |= DMA_HIFCR_CTEIF6;
    }
}

void DMA2_Stream2_IRQHandler()
{
    // DMA:
    // LISR:  DMA low interrupt status register
    // LIFCR: DMA low interrupt flag clear register
    if (DMA2->LISR & DMA_LISR_TCIF2) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;

        if (ESC_DShotBitbang::self->_reception_2_3) {
            // set GPIOs as inputs:
            GPIOB->MODER &= ~GPIO_MODER_MODER0;
            GPIOB->MODER &= ~GPIO_MODER_MODER1;
            // set pull up for these pins:
            GPIOB->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;

            // set timer:
            TIM8->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / ESC_DShotBitbang::RESPONSE_OVERSAMPLING - 1;
            TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / ESC_DShotBitbang::RESPONSE_OVERSAMPLING;

            DMA2_Stream2->CR &= ~(DMA_SxCR_DIR);
            DMA2_Stream2->PAR = (uint32_t)(&(GPIOB->IDR));
            DMA2_Stream2->M0AR = (uint32_t)(&ESC_DShotBitbang::self->_dmaInputBuffer_2_3[0]);
            // Main idea:
            // After sending DShot frame to ESC start receiving GPIO values.
            // Capture data (probing longer than ESC response).
            // There is ~33 [us] gap before the response so it is necessary to add more samples:
            DMA2_Stream2->NDTR = ((int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * ESC_DShotBitbang::RESPONSE_OVERSAMPLING);

            DMA2_Stream2->CR |= DMA_SxCR_EN;
            ESC_DShotBitbang::self->_reception_2_3 = false;
        }
    }

    if (DMA2->LISR & DMA_LISR_HTIF2) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF2;
    }
    if (DMA2->LISR & DMA_LISR_DMEIF2) {
        DMA2->LIFCR |= DMA_LIFCR_CDMEIF2;
    }
    if (DMA2->LISR & DMA_LISR_TEIF2) {
        DMA2->LIFCR |= DMA_LIFCR_CTEIF2;
    }
}
#endif

ESC_DShotBitbang::ESC_DShotBitbang()
{
    self = this;
}


/* IMPORTANT:
APB2 max frequency is 84 [MHz], 168 [MHz] only for timers
APB1 max frequency is 42 [MHz], 84 [MHz] only for timers
*/


void ESC_DShotBitbang::setupGPIO()
{
#if defined(USE_ARDUINO_STM32)
    // GPIOA (pin 2 - motor; pin 3 - motor)
    // GPIOB (pin 0 - motor; pin 1 - motor)

    // enable GPIOA clock:
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    //    set mode (00-input; 01-output; 10-alternate):
    // will be set in bdshot routine

    // set speed (max speed):
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3);
    // enable GPIOB clock:
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    //    set mode (00-input; 01-output; 10-alternate):
    // will be set in bdshot routine

    // set speed: (max speed)
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1);
#endif
}

void ESC_DShotBitbang::setupDMA()
{
#if defined(USE_ARDUINO_STM32)
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // bidirectional DSHOT:
    // for TIM1
    DMA2_Stream6->CR = 0x0;
    while (DMA2_Stream6->CR & DMA_SxCR_EN)
    {
        ; // wait
    }
    DMA2_Stream6->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
    // all the other parameters will be set afterward

    DMA2_Stream2->CR = 0x0;
    while (DMA2_Stream2->CR & DMA_SxCR_EN)
    {
        ; // wait
    }
    DMA2_Stream2->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
    // all the other parameters will be set afterward
#endif
}

void ESC_DShotBitbang::setupNVIC()
{
#if defined(USE_ARDUINO_STM32)
    // Nested Vectored Interrupt Controller
    // enable DMA interrupts
    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    NVIC_SetPriority(DMA2_Stream6_IRQn, 13);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_SetPriority(DMA2_Stream2_IRQn, 14);
#endif
}

void ESC_DShotBitbang::init()
{
    presetDMA_outputBuffers();
#if defined(USE_ARDUINO_STM32)
    setupGPIO();
    setupDMA();
    setupNVIC();
// TIM1 - only for generating time basement all outputs are set by GPIOs:
    // enable TIM1 clock:
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    // register is buffered and overflow DMA request:

    // CR: control register
    TIM1->CR1 = 0x0;
    TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

    // DMA request:
    // DIER: DMA/interrupt enable register
    // CCR: capture/compare register
    // PSC: prescaler
    // ARR: auto-reload register
    TIM1->DIER |= TIM_DIER_CC1DE; // CC1DE: Capture/Compare 1 DMA request enable
    // TIM1 is 168 MHz:
    TIM1->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
#if defined(BIT_BANGING_V1)
    TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
#elif defined(BIT_BANGING_V2)
    TIM1->DIER |= TIM_DIER_CC2DE; // channel 2 request
    TIM1->DIER |= TIM_DIER_CC3DE; // channel 3 request
    TIM1->CCR1 = 0;
    TIM1->CCR2 = DSHOT_BB_0_LENGTH;
    TIM1->CCR3 = DSHOT_BB_1_LENGTH;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH - 1;
#endif

    // TIM1 enable:
    // EGR: event generation register
    TIM1->EGR |= TIM_EGR_UG;  // UG: Update Generation
    TIM1->CR1 |= TIM_CR1_CEN; //CEN: counter enable

// TIM8 - only for generating time basement all outputs are set by GPIOs:
    // enable TIM8 clock:
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    // register is buffered and overflow DMA request:
    TIM8->CR1 = 0x0;
    TIM8->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;
    // TIM8 is 168 MHz:
    TIM8->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;

    // DMA request:
    TIM8->DIER |= TIM_DIER_CC1DE; // channel 1 request
#if defined(BIT_BANGING_V1)
    TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
#elif defined(BIT_BANGING_V2)
    TIM8->DIER |= TIM_DIER_CC2DE; // channel 2 request
    TIM8->DIER |= TIM_DIER_CC3DE; // channel 3 request
    TIM8->CCR1 = 0;
    TIM8->CCR2 = DSHOT_BB_0_LENGTH;
    TIM8->CCR3 = DSHOT_BB_1_LENGTH;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH - 1;
#endif
    // TIM8 enable:
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;
#endif // USE_ARDUINO_STM32
}

void ESC_DShotBitbang::outputToMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    update_motors_rpm();

    // !!TODO: m1..m4 above in the range [0,1000], need to convert to Dshot range [48,2047]
    setDMA_outputBuffersV2(m1, m2, m3, m4);
    // was
    // setDMA_outputBuffersV2(prepare_BDshot_package(motor_1_value),...

    _reception_1_4 = true;
    _reception_2_3 = true;

#if defined(USE_ARDUINO_STM32)
    // set GPIOs as output:
    GPIOB->MODER |= GPIO_MODER_MODER0_0;
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    GPIOA->MODER |= GPIO_MODER_MODER2_0;
    GPIOA->MODER |= GPIO_MODER_MODER3_0;

    // CR:   DMA stream x configuration register
    // NDTR: DMA stream x number of data register
    // PAR:  DMA stream x peripheral address register
    // M0AR: DMA stream x memory 0 address register
    DMA2_Stream6->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream6->PAR = reinterpret_cast<uint32_t>(&(GPIOA->BSRR));
    DMA2_Stream6->M0AR = reinterpret_cast<uint32_t>(&_dmaOutputBuffer_1_4[0]);
    DMA2_Stream6->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

    DMA2_Stream2->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream2->PAR = reinterpret_cast<uint32_t>(&(GPIOB->BSRR));
    DMA2_Stream2->M0AR = reinterpret_cast<uint32_t>(&_dmaOutputBuffer_2_3[0]);
    DMA2_Stream2->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

#if defined(BIT_BANGING_V1)
    // Main idea:
    // Every bit frame is divided in sections and for each section DMA request is generated.
    // After some sections (at beginning, after 0-bit time and after 1-bit time) to GPIO register can be sent value to set 1 or to set 0.
    // For rest of the sections 0x0 is sent so GPIOs don't change values.
    // It uses only 1 CCR on each timer.
    // Idea for reception is the same.

    //    TIM1 setup:
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
    TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

    // TIM8 setup:
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

    //  send:
    DMA2_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;

    // EGR: event generation register
    // CR: Control Register
    TIM1->EGR |= TIM_EGR_UG; // Update Generation
    TIM1->CR1 |= TIM_CR1_CEN; // Counter Enable
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;
#elif defined(BIT_BANGING_V2)
    // Main idea:
    // DMA requests generated at beginning, after 0-bit time and after 1-bit time
    // for each bit there is only 3 sections -> buffers are much smaller than in version 1
    // but uses 3 CCR (capture/compare register) for each timer (probably not a big deal)
    // It works for transferring but for reception it is not useful

    //    TIM1 setup:
    TIM1->CCR1 = 0;
    TIM1->CCR2 = DSHOT_BB_0_LENGTH;
    TIM1->CCR3 = DSHOT_BB_1_LENGTH;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH - 1;

    // TIM8 setup:
    TIM8->CCR1 = 0;
    TIM8->CCR2 = DSHOT_BB_0_LENGTH;
    TIM8->CCR3 = DSHOT_BB_1_LENGTH;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH - 1;

    //  send:
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;

    DMA2_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;
#endif
#endif // USE_ARDUINO_STM32
}

void ESC_DShotBitbang::presetDMA_outputBuffers()
{
#if defined(BIT_BANGING_V1)
    presetDMA_outputBuffersV1();
#elif defined(BIT_BANGING_V2)
    presetDMA_outputBuffersV2();
#endif
}
void ESC_DShotBitbang::setDMA_outputBuffers(uint16_t m1_frame, uint16_t m2_frame, uint16_t m3_frame, uint16_t m4_frame)
{
#if defined(BIT_BANGING_V2)
    setDMA_outputBuffersV2(m1_frame, m2_frame, m3_frame, m4_frame);
#else
    setDMA_outputBuffersV1(m1_frame, m2_frame, m3_frame, m4_frame);
#endif
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
*/

void ESC_DShotBitbang::presetDMA_outputBuffersV1()
{
    _dmaOutputBuffer_1_4.fill(0);
    _dmaOutputBuffer_2_3.fill(0);

    // for DSHOT_BB_FRAME_SECTIONS=14, DSHOT_BB_1_LENGTH=10 we have
    // r means BR (Bit Reset) (ie set to 0)
    // s means BS (Bit Set) (ie set to 1)
    //                  01234567890123
    // buffer:          r00000000s0000
    // resultant GPIO:  00000000011111
    for (size_t i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++) {
        const size_t index = i * DSHOT_BB_FRAME_SECTIONS;
        // 2 last bit will stay always high (it is for ESC to capture dshot frame to the end)
        // each bit is starting with lowering edge and after DSHOT_BB_1_LENGTH is rising (for 0-bit it rises earlier but always is high after 1-bit time)
        // set low edge at the beginning of each bit:
        _dmaOutputBuffer_1_4[index] = (GPIO_BSRR_BR_0 << MOTOR_1) | (GPIO_BSRR_BR_0 << MOTOR_4); // NOLINT(bugprone-implicit-widening-of-multiplication-result)
        _dmaOutputBuffer_2_3[index] = (GPIO_BSRR_BR_0 << MOTOR_2) | (GPIO_BSRR_BR_0 << MOTOR_3); // NOLINT(bugprone-implicit-widening-of-multiplication-result)
        // set transition to high after DSHOT_BB_1_LENGTH length:
        _dmaOutputBuffer_1_4[index + DSHOT_BB_1_LENGTH - 1] = (GPIO_BSRR_BS_0 << MOTOR_1) | (GPIO_BSRR_BS_0 << MOTOR_4);
        _dmaOutputBuffer_2_3[index + DSHOT_BB_1_LENGTH - 1] = (GPIO_BSRR_BS_0 << MOTOR_2) | (GPIO_BSRR_BS_0 << MOTOR_3);
    }
}

void ESC_DShotBitbang::setDMA_outputBuffersV1(uint16_t m1_frame, uint16_t m2_frame, uint16_t m3_frame, uint16_t m4_frame)
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

    /*for (size_t i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++) { // last 2 bits are always high (logic 0)
        const size_t index  = i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1;
        const uint32_t bitMask = 1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i);*/
    size_t index = DSHOT_BB_0_LENGTH - 1;
    for (uint32_t bitMask = 0x8000; bitMask !=0; bitMask >>= 1) {
        _dmaOutputBuffer_1_4[index] = (bitMask & m1_frame) ? 0x00 : GPIO_BSRR_BS_0 << MOTOR_1;
        if (bitMask & m4_frame) {
            _dmaOutputBuffer_1_4[index] |= GPIO_BSRR_BS_0 << MOTOR_4;
        }
        _dmaOutputBuffer_2_3[index] = (bitMask & m2_frame) ? 0x00 : GPIO_BSRR_BS_0 << MOTOR_2;
        if (bitMask & m3_frame) {
            _dmaOutputBuffer_2_3[index] |= GPIO_BSRR_BS_0 << MOTOR_3;
        }
        index += DSHOT_BB_FRAME_SECTIONS;
    }
}

/*!
Method 2 uses 3 sections for each bit and variable length sections
0 bit = 011
1 bit = 001

This requires smaller buffers, has lower DMA load, has more precise timing, but requires more timer channels

We can preset the DMA output buffer because whether the output bit is 0 or 1, the first section is zero and the last section is one.
*/
void ESC_DShotBitbang::presetDMA_outputBuffersV2()
{

    // this values are constant so they can be set once in the setup routine:

    //static std::array<uint32_t, DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS> _dmaOutputBuffer_1_4;
    // make 2 high frames after Dshot frame:
    for (auto i = 0; i < DSHOT_BB_FRAME_SECTIONS * 2; i++) {
        _dmaOutputBuffer_1_4[DSHOT_BUFFER_LENGTH*DSHOT_BB_FRAME_SECTIONS - i - 1] = GPIO_BSRR_BS_0 << MOTOR_1 | GPIO_BSRR_BS_0 << MOTOR_4;
        _dmaOutputBuffer_2_3[DSHOT_BUFFER_LENGTH*DSHOT_BB_FRAME_SECTIONS - i - 1] = GPIO_BSRR_BS_0 << MOTOR_2 | GPIO_BSRR_BS_0 << MOTOR_3;
    }
    for (auto i = 0; i < (DSHOT_BB_BUFFER_LENGTH - 2); i++) { // last 2 bits are always high (logic 0)
        //  first section always lower edge:
        _dmaOutputBuffer_1_4[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_1 | GPIO_BSRR_BR_0 << MOTOR_4; // NOLINT(bugprone-implicit-widening-of-multiplication-result)
        _dmaOutputBuffer_2_3[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_2 | GPIO_BSRR_BR_0 << MOTOR_3; // NOLINT(bugprone-implicit-widening-of-multiplication-result)

        // last section always rise edge:
        _dmaOutputBuffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 2] = GPIO_BSRR_BS_0 << MOTOR_1 | GPIO_BSRR_BS_0 << MOTOR_4;
        _dmaOutputBuffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 2] = GPIO_BSRR_BS_0 << MOTOR_2 | GPIO_BSRR_BS_0 << MOTOR_3;
    }
}

void ESC_DShotBitbang::setDMA_outputBuffersV2(uint16_t m1_frame, uint16_t m2_frame, uint16_t m3_frame, uint16_t m4_frame)
{
    // each bite frame is divided in sections where slope can vary:
    size_t index = 1;
    for (uint32_t bitMask = 0x8000; bitMask !=0; bitMask >>= 1) {
    //for (auto i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++) { // last 2 bits are always high (logic 0)
    //    const auto index = i * DSHOT_BB_FRAME_SECTIONS + 1;
    //    const uint16_t bitMask = 1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i);
        _dmaOutputBuffer_1_4[index] = (bitMask & m1_frame) ? 0x00 : GPIO_BSRR_BS_0 << MOTOR_1;
        if (bitMask & m4_frame) {
            _dmaOutputBuffer_1_4[index] |= GPIO_BSRR_BS_0 << MOTOR_4;
        }
        _dmaOutputBuffer_2_3[index] = (bitMask & m2_frame) ? 0x00 : GPIO_BSRR_BS_0 << MOTOR_2;
        if (bitMask & m3_frame) {
            _dmaOutputBuffer_2_3[index] |= GPIO_BSRR_BS_0 << MOTOR_3;
        }
        index += DSHOT_BB_FRAME_SECTIONS;
    }
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

#if false
void ESC_DShotBitbang::read_BDshot_response(uint32_t gcr21, uint8_t motor)
{
    // BDshot frame contain 21 bytes but first is always 0 (used only for detection).
    // Next 20 bits are 4 sets of 5-bits which are mapped with 4-bits real value.
    // After all, value is 16-bit long with 12-bit eRPM value (actually it is a period of eRPM) and 4-bit CRC.
    // 12-bit eRPM value has 3 first bits od left shifting and 9-bit mantissa.

    // put nibbles in the array in places of mapped values (to reduce empty elements smallest mapped value will always be subtracted)
    // now it is easy to create real value - mapped value indicate array element which contain nibble value:
    static constexpr uint32_t iv = 0xFFFFFFFF;
    static const std::array<uint32_t, 32> GCR_table = {
        iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
        iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv};

    const uint32_t gcr20 = (gcr21 ^ (gcr21 >> 1)); // now we have GCR value

    uint32_t decoded_value = GCR_table[(gcr20 & 0x1F)];
    decoded_value |= GCR_table[((gcr20 >> 5) & 0x1F)] << 4;
    decoded_value |= GCR_table[((gcr20 >> 10) & 0x1F)] << 8;
    decoded_value |= GCR_table[((gcr20 >> 15) & 0x1F)] << 12;

    // if wrongly decoded decoded_value will be bigger than uint16_t:
    if (decoded_value < 0xFFFF && DShotCodec::checksumUnidirectionalIsOK(static_cast<uint16_t>(decoded_value))) {
        // if checksum is correct real save real RPM.
        // value sent by ESC is a period between each pole changes [us].
        // to achieve eRPM we need to find out how many of these changes are in one minute.
        // eRPM = (60*1000 000)/T_us next RPM can be achieved -> RPM = eRPM/(poles/2):

        motors_rpm[motor - 1] = ((decoded_value & 0x1FF0) >> 4) << (decoded_value >> 13);      // cut off CRC and add shifting - this is period in [us]
        motors_rpm[motor - 1] = 60 * 1000000 / motors_rpm[motor - 1] * 2 / _motorPoleCount; // convert to RPM
        motors_error[motor - 1] = 0.9F * motors_error[motor - 1];                               // reduce motor error
    } else {
        motors_error[motor - 1] = 0.9F * motors_error[motor - 1] + 10; // increase motor error
    }
}
#endif

void ESC_DShotBitbang::update_motors_rpm()
{
    // BDshot bit banging reads whole GPIO register.
    // Now it's time to create BDshot responses from all motors (made of individual bits).
    const std::array<uint32_t, 4> motorGCR21s  = {
        samples_to_GCR21(&_dmaInputBuffer_1_4[0], 1 << MOTOR_1),
        samples_to_GCR21(&_dmaInputBuffer_2_3[0], 1 << MOTOR_2),
        samples_to_GCR21(&_dmaInputBuffer_2_3[0], 1 << MOTOR_3),
        samples_to_GCR21(&_dmaInputBuffer_1_4[0], 1 << MOTOR_4)
    };
    for (size_t ii = 0; ii < motorGCR21s.size(); ++ii) {
        const uint32_t motorGCR20 = DShotCodec::GCR21_to_GCR20(motorGCR21s[ii]);
        const uint16_t eRPM = DShotCodec::GCR20_to_eRPM(motorGCR20);
        if (DShotCodec::checksumBidirectionalIsOK(eRPM)) {
            DShotCodec::telemetry_type_e telemetryType {};
            const auto eRPM_periodMicroSeconds = DShotCodec::decodeTelemetryFrame(eRPM >> 4, telemetryType);
            enum { ONE_MINUTE_IN_MICROSECONDS = 60000000 };
            // value is eRPM period in microseconds
            _eRPMs[ii] = static_cast<int32_t>(ONE_MINUTE_IN_MICROSECONDS / eRPM_periodMicroSeconds);
        }
    }

}

// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)
