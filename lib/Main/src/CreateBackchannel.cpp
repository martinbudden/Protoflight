#include "Main.h"

#include <BackchannelFlightController.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <BackchannelTransceiverESPNOW.h>
#endif
#include <ReceiverAtomJoyStick.h>


#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
BackchannelBase& Main::createBackchannel(FlightController& flightController, AHRS& ahrs, ReceiverBase& receiver, const TaskBase* dashboardTask, NonVolatileStorage& nvs)
{
    // Statically allocate the backchannel.
    static constexpr uint8_t backchannelMacAddress[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    auto& receiverAtomJoyStick = static_cast<ReceiverAtomJoyStick&>(receiver); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    static BackchannelTransceiverESPNOW backchannelTransceiverESPNOW(receiverAtomJoyStick.getESPNOW_Transceiver(), &backchannelMacAddress[0]);

    static BackchannelFlightController backchannel(
        backchannelTransceiverESPNOW,
        &backchannelMacAddress[0],
        //&myMacAddress[0],
        &receiver.getMyEUI().octets[0],
        flightController,
        ahrs,
        receiver,
        dashboardTask,
        nvs
    );

    return backchannel;
}
#endif
