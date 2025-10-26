#include <ESC_DShotBitbang.h>


ESC_DShotBitbang* ESC_DShotBitbang::self; // alias of `this` to be used in ISR

#if (defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)) && defined(FRAMEWORK_STM32_CUBE_F4)

#if true
#define CONCAT(a, b) a##b

#define DMA_STREAM_IRQ_HANDLER(d, s, h, p) \
void DMA##d##_Stream##s##_IRQHandler(); \
void DMA##d##_Stream##s##_IRQHandler() { \
    if ( CONCAT(DMA,d)->CONCAT(h,ISR) & CONCAT(DMA_##h##ISR_TCIF,s) ) { \
        CONCAT(DMA,d)->CONCAT(h,IFCR) |= CONCAT(DMA_##h##IFCR_CTCIF,s); \
        ESC_DShotBitbang::port_t& port = ESC_DShotBitbang::CONCAT(self->getPort,p)(); \
        if (port.reception) { \
            ESC_DShotBitbang::IRQ_Handler(port); \
        } \
        if (CONCAT(DMA,d)->CONCAT(h,ISR) & CONCAT(DMA_##h##ISR_HTIF,s)) { \
            CONCAT(DMA,d)->CONCAT(h,IFCR) |= CONCAT(DMA_##h##IFCR_CHTIF,s); \
        } \
        if (CONCAT(DMA,d)->CONCAT(h,ISR) & CONCAT(DMA_##h##ISR_DMEIF,s)) { \
            CONCAT(DMA,d)->CONCAT(h,IFCR) |= CONCAT(DMA_##h##IFCR_CDMEIF,s); \
        } \
        if (CONCAT(DMA,d)->CONCAT(h,ISR) & CONCAT(DMA_##h##ISR_TEIF,s)) { \
            CONCAT(DMA,d)->CONCAT(h,IFCR) |= CONCAT(DMA_##h##IFCR_CTEIF,s); \
        } \
    } \
}


DMA_STREAM_IRQ_HANDLER(2,6,H,A)

DMA_STREAM_IRQ_HANDLER(2,2,L,B)

#else
void DMA2_Stream6_IRQHandler()
{
#if defined(FRAMEWORK_STM32_CUBE_F4)
    // HISR:  DMA high interrupt status register
    // HIFCR: DMA high interrupt flag clear register
    if (DMA2->HISR & DMA_HISR_TCIF6) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF6;
        ESC_DShotBitbang::port_t& port = ESC_DShotBitbang::self->getPortA();
        if (port.reception) {
            ESC_DShotBitbang::IRQ_Handler(port);
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
#endif
}

void DMA2_Stream2_IRQHandler()
{
#if defined(FRAMEWORK_STM32_CUBE_F4)
    // LISR:  DMA low interrupt status register
    // LIFCR: DMA low interrupt flag clear register
    if (DMA2->LISR & DMA_LISR_TCIF2) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
        ESC_DShotBitbang::port_t& port = ESC_DShotBitbang::self->getPortB();
        if (port.reception) {
            ESC_DShotBitbang::IRQ_Handler(port);
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
#endif
}
#endif
#endif
