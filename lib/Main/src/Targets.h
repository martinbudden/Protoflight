#pragma once

/*!
Targets
*/


#if defined(TARGET_CODECELL)
    #define BOARD_IDENTIFIER    "CodeCell_ESP32C3"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_BNO085
    #define IMU_I2C_PINS        pins_t{.sda=8,.scl=9,.irq=BUS_I2C::IRQ_NOT_SET}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}
#endif

#if defined(TARGET_M5STACK_STAMPS3)
    #define BOARD_IDENTIFIER    "M5Stack_StampS3"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_MPU6886
    #define IMU_I2C_PINS        pins_t{.sda=38,.scl=39,.irq=16}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}
#endif

#if defined(TARGET_M5STACK_ATOMS3R)
    #define BOARD_IDENTIFIER    "M5Stack_AtomS3"

    //#define USE_AHRS_TASK_INTERRUPT_DRIVEN_SCHEDULING
    //#define AHRS_TASK_INTERVAL_MICROSECONDS 312  // 3200Hz, exact value is 312.5
    #define AHRS_TASK_INTERVAL_MICROSECONDS 1000  // 1000Hz

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_BMI270
    #define IMU_I2C_PINS        pins_t{.sda=45,.scl=0,.irq=16}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_M5STACK_CORE2)
    #define BOARD_IDENTIFIER    "M5Stack_Core2"

    #define IMU_AXIS_ORDER      IMU_Base::YNEG_XPOS_ZPOS
    #define AHRS_TASK_INTERVAL_MICROSECONDS     5000
    #define RECEIVER_TASK_INTERVAL_MICROSECONDS 0
    //#define USE_IMU_MPU6886
    #define USE_IMU_M5_UNIFIED
    #define IMU_I2C_PINS        pins_t{.sda=21,.scl=22,.irq=BUS_I2C::IRQ_NOT_SET}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}

    #define SDCARD_SPI_PINS     pins_t{.cs=4,.sck=18,.cipo=38,.copi=23,.irq=0xFF}
    //#define USE_BLACKBOX
    //#define USE_BLACKBOX_DEBUG
    //#define BLACKBOX_IS_EVENT_DRIVEN
    //#define USE_MSP

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_PICO)
    #define BOARD_IDENTIFIER "RPI_Pico"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    #define IMU_SPI_INDEX       BUS_INDEX_0
    #define IMU_SPI_PINS        pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
    //#define IMU_SPI_PINS        pins_t{.cs=13,.sck=14,.cipo=12,.copi=15,.irq=20}
#else
    #define IMU_I2C_PINS        pins_t{.sda=07,.scl=27,.irq=BUS_I2C::IRQ_NOT_SET}
#endif

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}
#endif

#if defined(TARGET_PICO2)
    #define BOARD_IDENTIFIER    "RPI_Pico2"

    #define FC_TASK_DENOMINATOR 1
    //#define USE_AHRS_TASK_INTERRUPT_DRIVEN_SCHEDULING
    #define AHRS_TASK_INTERVAL_MICROSECONDS 1000  // 1000Hz

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    #define IMU_SPI_INDEX       BUS_INDEX_0
    #define IMU_SPI_PINS        pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
    //#define IMU_SPI_PINS        pins_t{.cs=13,.sck=14,.cipo=12,.copi=15,.irq=20}
#else
    #define IMU_I2C_PINS        pins_t{.sda=07,.scl=27,.irq=BUS_I2C::IRQ_NOT_SET}
#endif

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       pins_t{.rx=0,.tx=0}

    //#define USE_MOTOR_MIXER_QUAD_X_PWM
    #define USE_MOTOR_MIXER_QUAD_X_DSHOT
    #define USE_DSHOT_RPI_PICO_PIO
    #define MOTOR_PINS          pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}
    //#define MOTOR_PINS          pins_t{.m0=2,.m1=3,.m2=4,.m3=5}
#endif

#if defined(TARGET_SEED_XIAO_NRF52840_SENSE)
    #define BOARD_IDENTIFIER    "NRF52840_Sense"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    #define IMU_I2C_PINS        pins_t{.sda=07,.scl=27,.irq=BUS_I2C::IRQ_NOT_SET}

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       pins_t{.rx=0,.tx=0}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}
#endif

#if defined(TARGET_ADAFRUIT_FEATHER_F405)
    #define BOARD_IDENTIFIER    "Feather_F405"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    //#define USE_IMU_ICM426XX
    //#define USE_IMU_MPU6000
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    #define IMU_SPI_INDEX       BUS_INDEX_1
    #define IMU_SPI_PINS        pins_t{.cs=11,.sck=14,.cipo=14,.copi=15,.irq=10}
    //#define IMU_SPI_PINS        port_pins_t{.cs={PB,11},.sck={PB,14},.cipo={PB,14},.copi={PB,15},.irq={PB,10}}
#else
    //#define IMU_I2C_PINS        port_pins_t{.sda={PB,7},.scl={PB,6},.irq={0,0xFF}}
    #define IMU_I2C_PINS        pins_t{.sda=14,.scl=15,.irq=0xFF}
#endif

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS     port_pins_t{.rx={PB,7},.tx={PB,6}}
    //#define RECEIVER_PINS       pins_t{.rx=0,.tx=0}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          stm32_motor_pins4_t{.m0={PC,7,0,0},.m1={PC,6,0,0},.m2={PB,8,0,0},.m3={PB,9,0,0}}

    // NOTE this board uses SDIO for the SD card, so pins below are just to test the build
    #define SDCARD_SPI_PINS     port_pins_t{.cs={PC,8},.sck={PC,9},.cipo={PC,10},.copi={PC,11},.irq={0,0xFF}}
    #define USE_BLACKBOX
    //#define USE_MSP
#endif

#if defined(TARGET_AFROFLIGHT_F301CB)
    #define BOARD_IDENTIFIER    "AfroFlight_F301CB"

    #define USART_1_PINS        port_pins_t{.rx={PA,10},.tx={PA,9}} // TX output is always inverted (for FrSky). Internally connected to USB port via CP2102 IC
    #define USART_2_PINS        port_pins_t{.rx={PA,3},.tx={PA,2}}
    #define SOFT_SERIAL_1_PINS  port_pins_t{.rx={PA,6},.tx={PA,7}}
    #define SOFT_SERIAL_2_PINS  port_pins_t{.rx={PB,0},.tx={PB,1}}
    #define I2C_1_PINS          port_pins_t{.sda={PB,7},.scl={PB,6}}

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    // On afroflight Rev 5 MPU6050 is connected to IC2 index 2
    #define USE_IMU_MPU6000
    //#define IMU_I2C_PINS        port_pins_t{.sda={PB,7},.scl={PB,6},.irq={PB,13}}
    //!!TODO: add port_pins to IMU_MPU6000
    #define IMU_I2C_PINS        pins_t{.sda=7,.scl=6,.irq=13}

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       port_pins_t{.rx={PB,10},.tx={PB,9}}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          stm32_motor_pins4_t{.m0={PC,7,0,0},.m1={PC,6,0,0},.m2={PB,8,0,0},.m3={PB,9,0,0}}

    #define USE_MSP
    // LED0 PB4, LED1 PB3
#endif
