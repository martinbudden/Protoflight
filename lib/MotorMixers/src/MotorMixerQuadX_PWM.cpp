#if defined(USE_MOTOR_MIXER_QUAD_X_PWM)

#include "MotorMixerQuadX_PWM.h"

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
//framework-arduinoespressif32/tools/sdk/esp32s3/include/xtensa/esp32s3/include/xtensa/config/specreg.h:
//has #define BR  4
#if defined(BR)
#undef BR
#endif
#endif
#endif



MotorMixerQuadX_PWM::MotorMixerQuadX_PWM(const pins_t& pins, float deltaT)
    : _pins(pins)
{
    (void)deltaT;

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

    for (auto& filter : _motorFilters) {
        filter.setCutoffFrequency(100, deltaT);
    }
}

void MotorMixerQuadX_PWM::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)tickCount;

    if (motorsIsOn()) {
        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[MOTOR_BR] = -commands.roll - commands.pitch - commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FR] = -commands.roll + commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_BL] =  commands.roll - commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FL] =  commands.roll + commands.pitch - commands.yaw + commands.speed;

        // filter the motor output and scale to GPIO range ([0.0F, 65535.0F] for RPI Pico or [0.0F, 255.0F] for Arduino)
#if defined(FRAMEWORK_RPI_PICO)
        constexpr float scale = 65535.0F;
#else
        constexpr float scale = 255.0F;
#endif
        _motorOutputs[MOTOR_BR] =  _motorFilters[MOTOR_BR].filter(_motorOutputs[MOTOR_BR], deltaT);
        _motorOutputs[MOTOR_BR] =  std::roundf(scale*clip(_motorOutputs[MOTOR_BR], 0.0F, 1.0F));

        _motorOutputs[MOTOR_FR] =  _motorFilters[MOTOR_FR].filter(_motorOutputs[MOTOR_FR], deltaT);
        _motorOutputs[MOTOR_FR] =  std::roundf(scale*clip(_motorOutputs[MOTOR_FR], 0.0F, 1.0F));

        _motorOutputs[MOTOR_BL] =  _motorFilters[MOTOR_BL].filter(_motorOutputs[MOTOR_BL], deltaT);
        _motorOutputs[MOTOR_BL] =  std::roundf(scale*clip(_motorOutputs[MOTOR_BL], 0.0F, 1.0F));

        _motorOutputs[MOTOR_FL] =  _motorFilters[MOTOR_FL].filter(_motorOutputs[MOTOR_FL], deltaT);
        _motorOutputs[MOTOR_FL] =  std::roundf(scale*clip(_motorOutputs[MOTOR_FL], 0.0F, 1.0F));

    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

#if defined(FRAMEWORK_RPI_PICO)
    if (_pins.br != 0xFF) {
        pwm_set_gpio_level(_pins.br, static_cast<uint16_t>(_motorOutputs[MOTOR_BR]));
    }
    if (_pins.fr != 0xFF) {
        pwm_set_gpio_level(_pins.fr, static_cast<uint16_t>(_motorOutputs[MOTOR_FR]));
    }
    if (_pins.bl != 0xFF) {
        pwm_set_gpio_level(_pins.bl, static_cast<uint16_t>(_motorOutputs[MOTOR_BL]));
    }
    if (_pins.fl != 0xFF) {
        pwm_set_gpio_level(_pins.fl, static_cast<uint16_t>(_motorOutputs[MOTOR_FL]));
    }
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if defined(USE_ARDUINO_ESP32)
    if (_pins.br != 0xFF) {
        ledcWrite(MOTOR_BR, static_cast<uint32_t>(_motorOutputs[MOTOR_BR]));
    }
    if (_pins.fr != 0xFF) {
        ledcWrite(MOTOR_FR, static_cast<uint32_t>(_motorOutputs[MOTOR_FR]));
    }
    if (_pins.bl != 0xFF) {
        ledcWrite(MOTOR_BL, static_cast<uint32_t>(_motorOutputs[MOTOR_BL]));
    }
    if (_pins.fl != 0xFF) {
        ledcWrite(MOTOR_FL, static_cast<uint32_t>(_motorOutputs[MOTOR_FL]));
    }
#else // defaults to FRAMEWORK_ARDUINO
    if (_pins.br != 0xFF) {
        analogWrite(_pins.br, static_cast<int>(_motorOutputs[MOTOR_BR]));
    }
    if (_pins.fr != 0xFF) {
        analogWrite(_pins.fr, static_cast<int>(_motorOutputs[MOTOR_FR]));
    }
    if (_pins.bl != 0xFF) {
        analogWrite(_pins.bl, static_cast<int>(_motorOutputs[MOTOR_BL]));
    }
    if (_pins.fl != 0xFF) {
        analogWrite(_pins.fl, static_cast<int>(_motorOutputs[MOTOR_FL]));
    }
#endif
#endif // FRAMEWORK

}

#endif // USE_MOTOR_MIXER_QUAD_X_PWM
