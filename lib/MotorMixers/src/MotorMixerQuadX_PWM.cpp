#include "MotorMixerQuadX_PWM.h"
#include <cmath>

#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#elif defined(FRAMEWORK_ESPIDF)
#include <driver/ledc.h>
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal-ledc.h>
#endif
#endif // FRAMEWORK


MotorMixerQuadX_PWM::MotorMixerQuadX_PWM(Debug& debug, const stm32_motor_pins4_t& pins) :
    MotorMixerQuadX_Base(debug)
{
#if defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_ARDUINO_STM32)
    if (pins.m0.pin != 0xFF) {
        _pins[M0].htim = &_htims[M0];
        _pins[M0].channel = pins.m0.channel;
        _pins[M0].pin = pins.m0.pin;
        HAL_TIM_PWM_Start(_pins[M0].htim, _pins[M0].channel);
    }
    if (pins.m1.pin != 0xFF) {
        _pins[M1].htim = &_htims[M1];
        _pins[M1].channel = pins.m1.channel;
        _pins[M1].pin = pins.m1.pin;
        HAL_TIM_PWM_Start(_pins[M1].htim, _pins[M1].channel);
    }
    if (pins.m2.pin != 0xFF) {
        _pins[M2].htim = &_htims[M2];
        _pins[M2].channel = pins.m2.channel;
        _pins[M2].pin = pins.m2.pin;
        HAL_TIM_PWM_Start(_pins[M2].htim, _pins[M2].channel);
    }
    if (pins.m3.pin != 0xFF) {
        _pins[M3].htim = &_htims[M3];
        _pins[M3].channel = pins.m3.channel;
        _pins[M3].pin = pins.m3.pin;
        HAL_TIM_PWM_Start(_pins[M3].htim, _pins[M3].channel);
    }
#endif
}

MotorMixerQuadX_PWM::MotorMixerQuadX_PWM(Debug& debug, const pins_t& pins) :
    MotorMixerQuadX_Base(debug)
#if !defined(FRAMEWORK_STM32_CUBE)
    ,_pins({pins.m0,pins.m1,pins.m2,pins.m3})
#endif
{
#if defined(FRAMEWORK_RPI_PICO)

    _pwmScale = 65535.0F;
    if (pins.m0 != 0xFF) {
        gpio_set_function(pins.m0, GPIO_FUNC_PWM);
    }
    if (pins.m1 != 0xFF) {
        gpio_set_function(pins.m1, GPIO_FUNC_PWM);
    }
    if (pins.m2 != 0xFF) {
        gpio_set_function(pins.m2, GPIO_FUNC_PWM);
    }
    if (pins.m3 != 0xFF) {
        gpio_set_function(pins.m3, GPIO_FUNC_PWM);
    }

#elif defined(FRAMEWORK_ESPIDF)

#elif defined(FRAMEWORK_STM32_CUBE)

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    // Motor PWM Frequency
    constexpr int frequency = 150000;
    // PWM Resolution
    constexpr int resolution = 8;
    if (pins.m0 != 0xFF) {
        ledcSetup(M0, frequency, resolution);
        ledcAttachPin(pins.m0, M0);
    }
    if (pins.m1 != 0xFF) {
        ledcSetup(M1, frequency, resolution);
        ledcAttachPin(pins.m1, M1);
    }
    if (pins.m2 != 0xFF) {
        ledcSetup(M2, frequency, resolution);
        ledcAttachPin(pins.m2, M2);
    }
    if (pins.m3 != 0xFF) {
        ledcSetup(M3, frequency, resolution);
        ledcAttachPin(pins.m3, M3);
    }
#else
    if (pins.m0 != 0xFF) {
        pinMode(pins.m0, OUTPUT);
    }
    if (pins.m1 != 0xFF) {
        pinMode(pins.m1, OUTPUT);
    }
    if (pins.m2 != 0xFF) {
        pinMode(pins.m2, OUTPUT);
    }
    if (pins.m3 != 0xFF) {
        pinMode(pins.m3, OUTPUT);
    }
#endif

#endif // FRAMEWORK
}

void MotorMixerQuadX_PWM::writeMotorPWM(const pwm_pin_t& pin, uint8_t motorIndex)
{
#if defined(FRAMEWORK_RPI_PICO)
    // scale motor output to GPIO range [0, 65535] and write
    if (pin.pin != 0xFF) {
        const uint16_t motorOutput = static_cast<uint16_t>(roundf(_pwmScale*clip(_motorOutputs[motorIndex], 0.0F, 1.0F)));
        pwm_set_gpio_level(pin.pin, motorOutput);
    }
#elif defined(FRAMEWORK_ESPIDF)
    (void)pin;
    (void)motorIndex;
#elif defined(FRAMEWORK_STM32_CUBE)
    if (pin.pin != 0xFF) {
        const uint32_t motorOutput = static_cast<uint32_t>(roundf(_pwmScale*clip(_motorOutputs[motorIndex], 0.0F, 1.0F)));
        __HAL_TIM_SET_COMPARE(pin.htim, pin.channel, motorOutput);
    }
#elif defined(FRAMEWORK_TEST)
    (void)pin;
    (void)motorIndex;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    // scale motor output to GPIO range [0, 255] and write
    if (pin.pin != 0xFF) {
        const uint32_t motorOutput = static_cast<uint32_t>(roundf(_pwmScale*clip(_motorOutputs[motorIndex], 0.0F, 1.0F)));
        ledcWrite(motorIndex, motorOutput);
    }
#else
    // scale motor output to GPIO range [0, 255] and write
    if (pin.pin != 0xFF) {
        const uint32_t motorOutput = static_cast<uint32_t>(roundf(_pwmScale*clip(_motorOutputs[motorIndex], 0.0F, 1.0F)));
        analogWrite(pin.pin, motorOutput);
    }
#endif
#endif // FRAMEWORK
}

void MotorMixerQuadX_PWM::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)deltaT;
    (void)tickCount;

    _throttleCommand = commands.throttle;

    if (motorsIsOn()) {
        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[M0] = -commands.roll - commands.pitch - commands.yaw + commands.throttle;
        _motorOutputs[M1] = -commands.roll + commands.pitch + commands.yaw + commands.throttle;
        _motorOutputs[M2] =  commands.roll - commands.pitch + commands.yaw + commands.throttle;
        _motorOutputs[M3] =  commands.roll + commands.pitch - commands.yaw + commands.throttle;
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

    writeMotorPWM(_pins[M0], M0);
    writeMotorPWM(_pins[M1], M1);
    writeMotorPWM(_pins[M2], M2);
    writeMotorPWM(_pins[M3], M3);
}
