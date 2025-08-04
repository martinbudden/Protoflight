#include "MotorMixerQuadX_DShot.h"
//#include <RPM_Filter.h>
#include <cmath>


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(const pins_t& pins, RPM_Filter& rpmFilter) :
    _rpmFilter(rpmFilter)
{
    _motorBR.init(pins.br);
    _motorFR.init(pins.fr);
    _motorBL.init(pins.bl);
    _motorFL.init(pins.fl);
}

void MotorMixerQuadX_DShot::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)deltaT;
    (void)tickCount;

    if (motorsIsOn()) {
        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[MOTOR_BR] = -commands.roll - commands.pitch - commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FR] = -commands.roll + commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_BL] =  commands.roll - commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FL] =  commands.roll + commands.pitch - commands.yaw + commands.speed;

        // scale motor output to [0.0F, 1000.0F], which is the range required for DShot
        _motorOutputs[MOTOR_BR] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_BR], 0.0F, 1.0F));
        _motorOutputs[MOTOR_FR] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_FR], 0.0F, 1.0F));
        _motorOutputs[MOTOR_BL] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_BL], 0.0F, 1.0F));
        _motorOutputs[MOTOR_FL] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_FL], 0.0F, 1.0F));

    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }
    // and finally output to the motors and read to get the motor RPM from bidirectional dshot
    _motorBR.write(static_cast<uint16_t>(_motorOutputs[MOTOR_BR]));
    _motorBR.read();
    //_rpmFilter.setFrequency(MOTOR_BR, _motorBR.getMotorHz());

    _motorFR.write(static_cast<uint16_t>(_motorOutputs[MOTOR_FR]));
    _motorFR.read();
    //_rpmFilter.setFrequency(MOTOR_FR, _motorBR.getMotorHz());

    _motorBL.write(static_cast<uint16_t>(_motorOutputs[MOTOR_BL]));
    _motorBL.read();
    //_rpmFilter.setFrequency(MOTOR_BL, _motorBR.getMotorHz());

    _motorFL.write(static_cast<uint16_t>(_motorOutputs[MOTOR_FL]));
    _motorFL.read();
    //_rpmFilter.setFrequency(MOTOR_FL, _motorBR.getMotorHz());
}
