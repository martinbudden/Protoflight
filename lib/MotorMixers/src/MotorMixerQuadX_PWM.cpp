#include "MotorMixerQuadX_PWM.h"
#include <cmath>

#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#elif defined(FRAMEWORK_ESPIDF)
#include <driver/ledc.h>
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#if defined(USE_ARDUINO_ESP32)
#include <esp32-hal-ledc.h>
#endif
#endif // FRAMEWORK


MotorMixerQuadX_PWM::MotorMixerQuadX_PWM(Debug& debug, const pins_t& pins) :
    MotorMixerQuadX_Base(debug),
    _pins(pins)
{
#if defined(FRAMEWORK_RPI_PICO)

    if (pins.fl != 0xFF) {
        gpio_set_function(pins.fl, GPIO_FUNC_PWM);
    }
    if (pins.fr != 0xFF) {
        gpio_set_function(pins.fr, GPIO_FUNC_PWM);
    }
    if (pins.bl != 0xFF) {
        gpio_set_function(pins.bl, GPIO_FUNC_PWM);
    }
    if (pins.br != 0xFF) {
        gpio_set_function(pins.br, GPIO_FUNC_PWM);
    }

#elif defined(FRAMEWORK_ESPIDF)

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if defined(USE_ARDUINO_ESP32)
    // Motor PWM Frequency
    constexpr int frequency = 150000;

    // PWM Resolution
    constexpr int resolution = 8;

    if (pins.br != 0xFF) {
        ledcSetup(MOTOR_BR, frequency, resolution);
        ledcAttachPin(pins.br, MOTOR_BR);
    }
    if (pins.fr != 0xFF) {
        ledcSetup(MOTOR_FR, frequency, resolution);
        ledcAttachPin(pins.fr, MOTOR_FR);
    }

    if (pins.bl != 0xFF) {
        ledcSetup(MOTOR_BL, frequency, resolution);
        ledcAttachPin(pins.bl, MOTOR_BL);
    }
    if (pins.fl != 0xFF) {
        ledcSetup(MOTOR_FL, frequency, resolution);
        ledcAttachPin(pins.fl, MOTOR_FL);
    }
#else
    if (pins.br != 0xFF) {
        pinMode(pins.br, OUTPUT);
    }
    if (pins.fr != 0xFF) {
        pinMode(pins.fr, OUTPUT);
    }
    if (pins.bl != 0xFF) {
        pinMode(pins.bl, OUTPUT);
    }
    if (pins.fl != 0xFF) {
        pinMode(pins.fl, OUTPUT);
    }
#endif

#endif // FRAMEWORK
}

void MotorMixerQuadX_PWM::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)deltaT;
    (void)tickCount;

    if (motorsIsOn()) {
        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[MOTOR_BR] = -commands.roll - commands.pitch - commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FR] = -commands.roll + commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_BL] =  commands.roll - commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FL] =  commands.roll + commands.pitch - commands.yaw + commands.speed;
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

#if defined(FRAMEWORK_RPI_PICO)
    // scale motor output to GPIO range ([0, 65535]
    constexpr float scale = 65535.0F;
    if (_pins.br != 0xFF) {
        const uint16_t motorOutput = static_cast<uint16_t>(roundf(scale*clip(_motorOutputs[MOTOR_BR], 0.0F, 1.0F)));
        pwm_set_gpio_level(_pins.br, motorOutput);
    }
    if (_pins.fr != 0xFF) {
        const uint16_t motorOutput = static_cast<uint16_t>(roundf(scale*clip(_motorOutputs[MOTOR_FR], 0.0F, 1.0F)));
        pwm_set_gpio_level(_pins.fr, motorOutput);
    }
    if (_pins.bl != 0xFF) {
        const uint16_t motorOutput = static_cast<uint16_t>(roundf(scale*clip(_motorOutputs[MOTOR_BL], 0.0F, 1.0F)));
        pwm_set_gpio_level(_pins.bl, motorOutput);
    }
    if (_pins.fl != 0xFF) {
        const uint16_t motorOutput = static_cast<uint16_t>(roundf(scale*clip(_motorOutputs[MOTOR_FL], 0.0F, 1.0F)));
        pwm_set_gpio_level(_pins.fl, motorOutput);
    }
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if defined(USE_ARDUINO_ESP32)
    // scale motor output to GPIO range [0, 255]
    constexpr float scale = 255.0F;
    if (_pins.br != 0xFF) {
        const uint32_t motorOutput = static_cast<uint32_t>(roundf(scale*clip(_motorOutputs[MOTOR_BR], 0.0F, 1.0F)));
        ledcWrite(MOTOR_BR, motorOutput);
    }
    if (_pins.fr != 0xFF) {
        const uint32_t motorOutput = static_cast<uint32_t>(roundf(scale*clip(_motorOutputs[MOTOR_FR], 0.0F, 1.0F)));
        ledcWrite(MOTOR_FR, motorOutput);
    }
    if (_pins.bl != 0xFF) {
        const uint32_t motorOutput = static_cast<uint32_t>(roundf(scale*clip(_motorOutputs[MOTOR_BL], 0.0F, 1.0F)));
        ledcWrite(MOTOR_BL, motorOutput);
    }
    if (_pins.fl != 0xFF) {
        const uint32_t motorOutput = static_cast<uint32_t>(roundf(scale*clip(_motorOutputs[MOTOR_FL], 0.0F, 1.0F)));
        ledcWrite(MOTOR_FL, motorOutput);
    }
#else // defaults to FRAMEWORK_ARDUINO
    // scale motor output to GPIO range [0, 255]
    constexpr float scale = 255.0F;
    if (_pins.br != 0xFF) {
        const int motorOutput = static_cast<int>(roundf(scale*clip(_motorOutputs[MOTOR_BR], 0.0F, 1.0F)));
        analogWrite(_pins.br, motorOutput);
    }
    if (_pins.fr != 0xFF) {
        const int motorOutput = static_cast<int>(roundf(scale*clip(_motorOutputs[MOTOR_FR], 0.0F, 1.0F)));
        analogWrite(_pins.fr, motorOutput);
    }
    if (_pins.bl != 0xFF) {
        const int motorOutput = static_cast<int>(roundf(scale*clip(_motorOutputs[MOTOR_BL], 0.0F, 1.0F)));
        analogWrite(_pins.bl, motorOutput);
    }
    if (_pins.fl != 0xFF) {
        const int motorOutput = static_cast<int>(roundf(scale*clip(_motorOutputs[MOTOR_FL], 0.0F, 1.0F)));
        analogWrite(_pins.fl, motorOutput);
    }
#endif
#endif // FRAMEWORK

}
