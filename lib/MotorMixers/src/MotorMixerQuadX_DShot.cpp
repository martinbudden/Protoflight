#include "MotorMixerQuadX_DShot.h"


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(const pins_t& pins,float deltaT)
{
    for (auto& filter : _motorFilters) {
        filter.setCutoffFrequency(100, deltaT);
    }
#if defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)
    _motorBR.init(pins.br);
    _motorFR.init(pins.fr);
    _motorBL.init(pins.bl);
    _motorFL.init(pins.fl);
#else
    (void)pins;
#endif
}

void MotorMixerQuadX_DShot::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)tickCount;

    if (motorsIsOn()) {
        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[MOTOR_BR] = -commands.roll - commands.pitch - commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FR] = -commands.roll + commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_BL] =  commands.roll - commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FL] =  commands.roll + commands.pitch - commands.yaw + commands.speed;

        // filter the motor output and scale to [0.0F, 1000.0F], which is the range required for DShot
        _motorOutputs[MOTOR_BR] =  _motorFilters[MOTOR_BR].filter(_motorOutputs[MOTOR_BR], deltaT);
        _motorOutputs[MOTOR_BR] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_BR], 0.0F, 1.0F));

        _motorOutputs[MOTOR_FR] =  _motorFilters[MOTOR_FR].filter(_motorOutputs[MOTOR_FR], deltaT);
        _motorOutputs[MOTOR_FR] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_FR], 0.0F, 1.0F));

        _motorOutputs[MOTOR_BL] =  _motorFilters[MOTOR_BL].filter(_motorOutputs[MOTOR_BL], deltaT);
        _motorOutputs[MOTOR_BL] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_BL], 0.0F, 1.0F));

        _motorOutputs[MOTOR_FL] =  _motorFilters[MOTOR_FL].filter(_motorOutputs[MOTOR_FL], deltaT);
        _motorOutputs[MOTOR_FL] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_FL], 0.0F, 1.0F));

    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }
#if defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)
    // and finally output to the motors
    _motorBR.write(static_cast<uint16_t>(_motorOutputs[MOTOR_BR]));
    _motorFR.write(static_cast<uint16_t>(_motorOutputs[MOTOR_FR]));
    _motorBL.write(static_cast<uint16_t>(_motorOutputs[MOTOR_BL]));
    _motorFL.write(static_cast<uint16_t>(_motorOutputs[MOTOR_FL]));
#endif
}
