#pragma once

/*!
Targets
*/


#if defined(TARGET_M5STACK_STAMPS3_FLY)

    #define BOARD_IDENTIFIER    "M5Stack_StampS3_Fly"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS_NED
    #define USE_IMU_BMI270
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
    #define IMU_SPI_INDEX       BUS_INDEX_0
    #define IMU_SPI_PINS        spi_pins_t{.cs=46,.sck=44,.cipo=43,.copi=14,.irq=0xFF}
    #define USE_BAROMETER_BMP280
    #define BAROMETER_I2C_PINS  i2c_pins_t{.sda=3,.scl=4,.irq=BUS_I2C::IRQ_NOT_SET}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    //                                       BR     TR     BL     TL
    #define MOTOR_PINS          motor_pins_t{.m0=23,.m1=25,.m2=10,.m3=5}

    #define USE_BACKCHANNEL
    #define USE_YAW_SPIN_RECOVERY
    #define USE_FLIGHT_CONTROLLER_TIME_CHECKS

#elif defined(TARGET_M5STACK_ATOMS3R)

    #define BOARD_IDENTIFIER    "M5Stack_AtomS3"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_BMI270
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 3200
    #define IMU_I2C_PINS        i2c_pins_t{.sda=45,.scl=0,.irq=16}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          motor_pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}

    #define USE_BACKCHANNEL

    #define USE_CMS
    //#define USE_DASHBOARD

    #define USE_FLIGHT_CONTROLLER_TIME_CHECKS

#elif defined(TARGET_M5STACK_CORE2)

    #define BOARD_IDENTIFIER    "M5Stack_Core2"

    #define USE_FLIGHT_CONTROLLER_TIME_CHECKS

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define RECEIVER_TASK_INTERVAL_MICROSECONDS 0
#if defined(LIBRARY_SENSORS_IMU_USE_M5_UNIFIED)
    #define USE_IMU_M5_UNIFIED
#else
    #define USE_IMU_MPU6886
#endif
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 200 // 5000us looptime
    #define IMU_I2C_PINS        i2c_pins_t{.sda=21,.scl=22,.irq=BUS_I2C::IRQ_NOT_SET}

    //#define USE_BAROMETER_BMP280
    //#define BAROMETER_I2C_PINS  i2c_pins_t{.sda=21,.scl=22,.irq=BUS_I2C::IRQ_NOT_SET}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          motor_pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}

    //#define SDCARD_SPI_PINS     spi_pins_t{.cs=4,.sck=18,.cipo=38,.copi=23,.irq=0xFF}
    //#define USE_BLACKBOX
    //#define USE_BLACKBOX_TEST
    //#define USE_MSP
    #define USE_DASHBOARD
    //#define USE_OSD
    //#define USE_CMS
    #define USE_VTX

    #define USE_BACKCHANNEL


    //#define USE_D_MAX
    //#define USE_ITERM_RELAX
    //#define USE_YAW_SPIN_RECOVERY
    //#define USE_CRASH_RECOVERY
    //#define USE_DYNAMIC_NOTCH_FILTER
    //#define USE_ALTITUDE_HOLD

#elif defined(TARGET_NATIVE)

    #define BOARD_IDENTIFIER    "BOARD_NONE"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS_NED
    #define RECEIVER_TASK_INTERVAL_MICROSECONDS 0
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 200 // 5000us looptime

    #define USE_BAROMETER_BMP280
    #define BAROMETER_I2C_PINS  i2c_pins_t{.sda=0,.scl=0,.irq=BUS_I2C::IRQ_NOT_SET}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          motor_pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}

    #define SDCARD_SPI_PINS     spi_pins_t{.cs=0,.sck=0,.cipo=0,.copi=0,.irq=0xFF}
    #define USE_BLACKBOX

    #define USE_MSP
    #define USE_CMS
    #define USE_OSD
    #define USE_VTX

    #define USE_BACKCHANNEL

    #define USE_D_MAX
    #define USE_ITERM_RELAX
    #define USE_YAW_SPIN_RECOVERY
    #define USE_CRASH_RECOVERY
    #define USE_DYNAMIC_NOTCH_FILTER
    #define USE_RPM_FILTERS
    #define USE_ALTITUDE_HOLD

#elif defined(TARGET_PICO)

    #define BOARD_IDENTIFIER "RPI_Pico"

    #define USE_ITERM_RELAX
    #define USE_RPM_FILTERS

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    #define IMU_SPI_INDEX       BUS_INDEX_0
    #define IMU_SPI_PINS        spi_pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
    //#define IMU_SPI_PINS        spi_pins_t{.cs=13,.sck=14,.cipo=12,.copi=15,.irq=20}
#else
    #define IMU_I2C_PINS        i2c_pins_t{.sda=07,.scl=27,.irq=BUS_I2C::IRQ_NOT_SET}
#endif

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          motor_pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}

#elif defined(TARGET_PICO2)

    #define BOARD_IDENTIFIER    "RPI_Pico2"

    #define USE_D_MAX
    #define USE_ITERM_RELAX
    #define USE_YAW_SPIN_RECOVERY
    #define USE_CRASH_RECOVERY
    #define USE_DYNAMIC_IDLE
    #define USE_RPM_FILTERS
    #define USE_DYNAMIC_NOTCH_FILTER
    #define USE_ALTITUDE_HOLD

    #define OUTPUT_TO_MOTORS_DENOMINATOR 1

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    #define IMU_SPI_INDEX       BUS_INDEX_0
    #define IMU_SPI_PINS        spi_pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
    //#define IMU_SPI_PINS        spi_pins_t{.cs=13,.sck=14,.cipo=12,.copi=15,.irq=20}
#else
    #define IMU_I2C_PINS        i2c_pins_t{.sda=07,.scl=27,.irq=BUS_I2C::IRQ_NOT_SET}
#endif

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       rx_pins_t{.rx=0,.tx=0}

    //#define USE_MOTOR_MIXER_QUAD_X_PWM
    #define USE_MOTOR_MIXER_QUAD_X_DSHOT
    #define USE_DSHOT_RPI_PICO_PIO
    #define MOTOR_PINS          motor_pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}
    //#define MOTOR_PINS          motor_pins_t{.m0=2,.m1=3,.m2=4,.m3=5}

    #define USE_MAX7456
    #define MAX7456_SPI_PINS    spi_pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
    #define MAX7456_SPI_INDEX   BUS_INDEX_1

#elif defined(TARGET_MADFLIGHT_FC2)

    #define BOARD_IDENTIFIER    "Madflight_FC2"
    // ESP32 S3
    // see https://madflight.com/Board-ESP-FC2/
    // pins: https://github.com/qqqlab/madflight/blob/main/src/brd/madflight_FC3.h
    // schematic: https://madflight.com/img/madflight-ESP-FC2.pdf

    #define OUTPUT_TO_MOTORS_DENOMINATOR 1

    #define SPI_0_PINS                  spi_pins_t{.cs=17,.sck=16,.cipo=14,.copi=15,.irq=13}
    #define UART_0_PINS                 rx_pins_t{.rx=7,.tx=21}
    #define UART_1_PINS                 rx_pins_t{.rx=6,.tx=9}
    #define I2C_0_PINS                  i2c_pins_t{.sda=11,.scl=10,.irq=BUS_I2C::IRQ_NOT_SET}
    #define I2C_1_PINS                  i2c_pins_t{.sda=33,.scl=39,.irq=BUS_I2C::IRQ_NOT_SET}
    #define SD_MMC_PINS                 mmc_pins_t{.dat=37,.clk=36,.cmd=35}
    #define MOTOR_PINS                  motor_pins_t{.m0=1,.m1=2,.m2=3,.m3=4}
    // other output pins are: 45, 42, 41, 5

    #define IMU_AXIS_ORDER              IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_ICM426XX
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
    #define IMU_SPI_INDEX               BUS_INDEX_0
    #define IMU_SPI_PINS                SPI_0_PINS

    // LIBRARY_RECEIVER_USE_ESPNOW is defined in platformio.ini
    //#define USE_RECEIVER_SBUS
    //#define RECEIVER_UART_INDEX         0
    //#define RECEIVER_PINS               UART_0_PINS

    #define USE_BACKCHANNEL

    #define USE_MOTOR_MIXER_QUAD_X_PWM

    #define USE_BAROMETER_HP203B
    #define BAROMETER_I2C_INDEX         0
    #define BAROMETER_I2C_ADDRESS       0x76

    #define USE_MAGNETOMETER_QMC6309
    #define MAGNETOMETER_I2C_INDEX      0
    #define MAGNETOMETER_I2C_ADDRESS    0x7C

    #define USE_BATTERY_MONITOR_INA226
    #define BATTERY_MONITOR_I2C_INDEX   0
    #define BATTERY_MONITOR_I2C_ADDRESS 0x40

    #define USE_GPS_NONE
    #define GPS_UART_INDEX              1

    #define NEO_PIXEL_PIN               12

#elif defined(TARGET_MADFLIGHT_FC3)
    #define BOARD_IDENTIFIER    "Madflight_FC3"
    // RPI PICO RP2350
    // see https://madflight.com/Board-FC3/
    // pins: https://github.com/qqqlab/madflight/blob/main/src/brd/madflight_FC3.h
    // schematic: https://madflight.com/img/madflight-FC3.pdf

    #define USE_ITERM_RELAX
    #define USE_YAW_SPIN_RECOVERY
    #define USE_DYNAMIC_IDLE
    #define USE_RPM_FILTERS

    #define OUTPUT_TO_MOTORS_DENOMINATOR 2
    //#define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ         8000

    #define SPI_1_PINS                  spi_pins_t{.cs=29,.sck=30,.cipo=28,.copi=31,.irq=27}
    #define UART_0_PINS                 rx_pins_t{.rx=1,.tx=0}
    #define UART_1_PINS                 rx_pins_t{.rx=5,.tx=4}
    #define I2C_0_PINS                  i2c_pins_t{.sda=32,.scl=33,.irq=BUS_I2C::IRQ_NOT_SET} // for barometer, battery, and magnetometer
    #define I2C_1_PINS                  i2c_pins_t{.sda=2,.scl=3,.irq=BUS_I2C::IRQ_NOT_SET} // for GPS
    #define SD_MMC_PINS                 mmc_pins_t{.dat=36,.clk=34,.cmd=35}
    #define MOTOR_PINS                  motor_pins_t{.m0=6,.m1=7,.m2=8,.m3=9}

    #define IMU_AXIS_ORDER              IMU_Base::XNEG_YNEG_ZPOS
    #define USE_IMU_ICM426XX
    #define IMU_SPI_INDEX               BUS_INDEX_1
    #define IMU_SPI_PINS                SPI_1_PINS

    //#define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX         0
    #define RECEIVER_PINS               UART_0_PINS

    //#define USE_MOTOR_MIXER_QUAD_X_PWM
    #define USE_MOTOR_MIXER_QUAD_X_DSHOT
    #define USE_DSHOT_RPI_PICO_PIO
   
    //#define USE_BLACKBOX

    #define USE_BAROMETER_BMP580
    #define BAROMETER_I2C_INDEX         0
    #define BAROMETER_I2C_ADDRESS       0x47

    #define USE_MAGNETOMETER_QMC5883
    #define MAGNETOMETER_I2C_INDEX      0
    #define MAGNETOMETER_I2C_ADDRESS    0x2C

    #define USE_BATTERY_MONITOR_INA226
    #define BATTERY_MONITOR_I2C_INDEX   0
    #define BATTERY_MONITOR_I2C_ADDRESS 0x40

    #define USE_GPS_NONE
    #define GPS_UART_INDEX              1

    #define NEO_PIXEL_PIN               46

#elif defined(TARGET_CODECELL)

    #define BOARD_IDENTIFIER    "CodeCell_ESP32C3"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_BNO085
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
    #define IMU_I2C_PINS        i2c_pins_t{.sda=8,.scl=9,.irq=BUS_I2C::IRQ_NOT_SET}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          motor_pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}

    #define USE_BACKCHANNEL

#elif defined(TARGET_AFROFLIGHT_F301CB)

    #define BOARD_IDENTIFIER    "AfroFlight_F301CB"

    //#define USE_D_MAX
    //#define USE_ITERM_RELAX
    //#define USE_YAW_SPIN_RECOVERY

    #define USART_1_PINS        stm32_rx_pins_t{.rx={PA,10},.tx={PA,9}} // TX output is always inverted (for FrSky). Internally connected to USB port via CP2102 IC
    #define USART_2_PINS        stm32_rx_pins_t{.rx={PA,3},.tx={PA,2}}
    #define SOFT_SERIAL_1_PINS  stm32_rx_pins_t{.rx={PA,6},.tx={PA,7}}
    #define SOFT_SERIAL_2_PINS  stm32_rx_pins_t{.rx={PB,0},.tx={PB,1}}
    #define I2C_1_PINS          stm32_i2c_pins_t{.sda={PB,7},.scl={PB,6}}

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    // On afroflight Rev 5 MPU6050 is connected to IC2 index 2
    #define USE_IMU_MPU6000
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
    #define IMU_I2C_PINS        stm32_i2c_pins_t{.sda={PB,7},.scl={PB,6},.irq={PB,13}}

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       stm32_rx_pins_t{.rx={PB,10},.tx={PB,9}}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          stm32_motor_pins_t{.m0={PC,7,0,0},.m1={PC,6,0,0},.m2={PB,8,0,0},.m3={PB,9,0,0}}

    //#define USE_MSP
    // LED0 PB4, LED1 PB3

#elif defined(TARGET_SEED_XIAO_NRF52840_SENSE)

    #define BOARD_IDENTIFIER    "NRF52840_Sense"

    #define USE_YAW_SPIN_RECOVERY

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
    #define IMU_I2C_PINS        i2c_pins_t{.sda=07,.scl=27,.irq=BUS_I2C::IRQ_NOT_SET}

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       rx_pins_t{.rx=0,.tx=0}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          motor_pins_t{.m0=0xFF,.m1=0xFF,.m2=0xFF,.m3=0xFF}

#elif defined(TARGET_ADAFRUIT_FEATHER_F405)

    #define BOARD_IDENTIFIER    "Feather_F405"

    #define USE_D_MAX
    #define USE_ITERM_RELAX
    #define USE_YAW_SPIN_RECOVERY
    #define USE_CRASH_RECOVERY
    #define USE_RPM_FILTERS
    #define USE_DYNAMIC_IDLE

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
    //#define USE_IMU_ICM426XX
    //#define USE_IMU_MPU6000
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    #define IMU_SPI_INDEX       BUS_INDEX_1
    #define IMU_SPI_PINS        stm32_spi_pins_t{.cs={PB,11},.sck={PB,14},.cipo={PB,14},.copi={PB,15},.irq={PB,10}}
#else
    #define IMU_I2C_PINS        stm32_i2c_pins_t{.sda={PB,7},.scl={PB,6},.irq={0,0xFF}}
#endif

    #define USE_RECEIVER_CRSF
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       stm32_rx_pins_t{.rx={PB,7},.tx={PB,6}}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          stm32_motor_pins_t{.m0={PC,7,0,0},.m1={PC,6,0,0},.m2={PB,8,0,0},.m3={PB,9,0,0}}

    // NOTE this board uses SDIO for the SD card, so pins below are just to test the build
    #define SDCARD_SPI_PINS     stm32_spi_pins_t{.cs={PC,8},.sck={PC,9},.cipo={PC,10},.copi={PC,11},.irq={0,0xFF}}
    #define USE_BLACKBOX
    //#define USE_MSP

#elif defined(TARGET_STM32F3_DISCOVERY)

    #define BOARD_IDENTIFIER    "STM32F3_Discovery"
    // https://www.st.com/resource/en/user_manual/um1570-discovery-kit-with-stm32f303vc-mcu-stmicroelectronics.pdf
    // https://www.st.com/resource/en/schematic_pack/mb1035-f303c-e02_schematic.pdf
    // https://www.st.com/resource/en/datasheet/stm32f303vc.pdf
    #define USE_RPM_FILTERS

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS

    //#define USE_IMU_LSM303AGR
    #define USE_IMU_MPU6000 // !!TODO:temporary to fix to get to build
    #define AHRS_TASK_IS_TIMER_DRIVEN
    #define GYRO_SAMPLE_RATE_HZ 1000
    #define IMU_SPI_INDEX       BUS_INDEX_1
    #define IMU_SPI_PINS        stm32_spi_pins_t{.cs={PE,3},.sck={PA,5},.cipo={PA,6},.copi={PA,7},.irq={PE,0}}

    #define USE_RECEIVER_IBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       stm32_rx_pins_t{.rx={PA,0},.tx={PA,0}}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          stm32_motor_pins_t{.m0={PA,0,0,0},.m1={PA,0,0,0},.m2={PA,0,0,0},.m3={PA,0,0,0}}

    // LED3 PE9 // red
    // LED4 PE8 // blue
#endif
