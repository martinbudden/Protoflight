#include "Main.h"
#include "NonVolatileStorage.h"
#include "RC_Modes.h"


RcModes& Main::createRcModes(NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
    static RcModes rc_modes; // !!for now, to be owned by receiver task
    rc_modes.set_mode_activation_conditions(nvs.load_rc_mode_activation_conditions());

#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && false
    const rc_modes_activation_condition_t macAngle = {
        .mode_id = MspBox::BOX_ANGLE,
        .aux_channel_index = ReceiverBase::AUX2,
        .range_start = RcModes::RANGE_STEP_MID,
        .range_end = RcModes::RANGE_STEP_MAX
        .mode_logic {},
        .linked_to {}
    };
    rcModes.set_mode_activation_condition(0, macAngle);

    const rc_modes_activation_condition_t macAltitudeHold = {
        .mode_id = MspBox::BOX_ALTHOLD,
        .aux_channel_index = ReceiverBase::AUX3,
        .range_start = RcModes::RANGE_STEP_MID,
        .range_end = RcModes::RANGE_STEP_MAX
        .mode_logic {},
        .linked_to {}
    };
    rcModes.set_mode_activation_condition(1, macAltitudeHold);
#endif // LIBRARY_RECEIVER_USE_ESPNOW

    return rc_modes;
}
