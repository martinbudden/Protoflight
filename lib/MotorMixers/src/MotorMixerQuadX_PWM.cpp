#include "Mixers.h"
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


MotorMixerQuadX_PWM::MotorMixerQuadX_PWM(Debug& debug, const stm32_motor_pins_t& pins) :
    MotorMixerQuadBase(debug)
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
#else
    (void)pins;
#endif
}

MotorMixerQuadX_PWM::MotorMixerQuadX_PWM(Debug& debug, const motor_pins_t& pins) :
    MotorMixerQuadBase(debug)
#if !defined(FRAMEWORK_STM32_CUBE)
    ,_pins({pins.m0,pins.m1,pins.m2,pins.m3})
#endif
{
#if defined(FRAMEWORK_RPI_PICO)

    _pwmScale = 65535.0F; // NOLINT(cppcoreguidelines-prefer-member-initializer)
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

    (void)pins;

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    // Motor PWM Frequency
    static constexpr int frequency = 150000;
    // PWM Resolution
    static constexpr int resolution = 8;
#if defined(ESPRESSIF32_6_11_0)
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
        ledcAttach(pins.m0, frequency, resolution);
    }
    if (pins.m1 != 0xFF) {
        ledcAttach(pins.m1, frequency, resolution);
    }
    if (pins.m2 != 0xFF) {
        ledcAttach(pins.m2, frequency, resolution);
    }
    if (pins.m3 != 0xFF) {
        ledcAttach(pins.m3, frequency, resolution);
    }
#endif
#else // defaults to FRAMEWORK_ARDUINO
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

void MotorMixerQuadX_PWM::writeMotor(uint8_t motorIndex, float motorOutput) // NOLINT(readability-make-member-function-const_
{
    const pwm_pin_t& pin = _pins[motorIndex];
    // scale motor output to GPIO range (normally [0,255] or [0, 65535])
    const auto output = static_cast<uint16_t>(roundf(_pwmScale*clip(motorOutput, 0.0F, 1.0F)));
#if defined(FRAMEWORK_RPI_PICO)
    if (pin.pin != 0xFF) {
        pwm_set_gpio_level(pin.pin, output);
    }
#elif defined(FRAMEWORK_ESPIDF)
    (void)pin;
    (void)output;
#elif defined(FRAMEWORK_STM32_CUBE)
    if (pin.pin != 0xFF) {
        __HAL_TIM_SET_COMPARE(pin.htim, pin.channel, output);
    }
#elif defined(FRAMEWORK_TEST)
    (void)pin;
    (void)output;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    if (pin.pin != 0xFF) {
#if defined(ESPRESSIF32_6_11_0)
        ledcWrite(motorIndex, output);
#else
        ledcWrite(pin.pin, output);
#endif
    }
#else
    if (pin.pin != 0xFF) {
        analogWrite(pin.pin, output);
    }
#endif
#endif // FRAMEWORK
}

/*!
Calculate and output motor mix.
*/
void MotorMixerQuadX_PWM::outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)deltaT;
    (void)tickCount;

    if (motorsIsOn()) {
        // set the throttle to value returned by the mixer
        commands.throttle = mixQuadX(_motorOutputs, commands, _motorOutputMin);
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

    writeMotor(M0, _motorOutputs[M0]);
    writeMotor(M1, _motorOutputs[M1]);
    writeMotor(M2, _motorOutputs[M2]);
    writeMotor(M3, _motorOutputs[M3]);
}
