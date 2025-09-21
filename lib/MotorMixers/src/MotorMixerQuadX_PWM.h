#pragma once

#include <MotorMixerQuadX_Base.h>

#if defined(FRAMEWORK_STM32_CUBE)

#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_tim.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_tim.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_tim.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_tim.h>
#endif

#endif


class MotorMixerQuadX_PWM : public MotorMixerQuadX_Base {
public:
#if defined(FRAMEWORK_STM32_CUBE)
    struct pwm_pin_t {
        TIM_HandleTypeDef* htim;
        uint8_t pin;
        uint8_t channel;
    };
#else
    struct pwm_pin_t {
        uint8_t pin;
    };
#endif
public:
    MotorMixerQuadX_PWM(Debug& debug, const pins_t& pins);
    MotorMixerQuadX_PWM(Debug& debug, const stm32_motor_pins4_t& pins);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    void writeMotorPWM(const pwm_pin_t& pin, uint8_t motorIndex);
protected:
    float _pwmScale {255.0F};
    std::array<pwm_pin_t, MOTOR_COUNT> _pins {};
#if defined(FRAMEWORK_STM32_CUBE)
    std::array<TIM_HandleTypeDef, MOTOR_COUNT> _htims {};
#endif
};
