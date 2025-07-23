#pragma once


#if defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)
#include <ESC_DShot_RPI_Pico.h>
#endif
#include <Filters.h>
#include <MotorMixerQuadX_Base.h>
#include <array>


class MotorMixerQuadX_DShot : public MotorMixerQuadX_Base {
public:
    MotorMixerQuadX_DShot (const pins_t& pins, float deltaT);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
protected:
    std::array<IIR_filter, MOTOR_COUNT> _motorFilters;
#if defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)
    ESC_DShot _motorBR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorBL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
#endif
};

