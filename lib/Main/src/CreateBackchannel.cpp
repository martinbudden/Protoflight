#include "Main.h"

#include <BackchannelFlightController.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <BackchannelTransceiverESPNOW.h>
#endif
#include <ReceiverAtomJoyStick.h>


#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
BackchannelBase& Main::createBackchannel(FlightController& flightController, AHRS& ahrs, ReceiverBase& receiver, NonVolatileStorage& nvs, const TaskBase* dashboardTask)
{
    // Statically allocate the backchannel.
    static constexpr uint8_t backchannelMacAddress[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    auto& receiverAtomJoyStick = static_cast<ReceiverAtomJoyStick&>(receiver); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    static BackchannelTransceiverESPNOW backchannelTransceiverESPNOW(receiverAtomJoyStick.get_espnow_transceiver(), &backchannelMacAddress[0]);


    const ReceiverBase::EUI_48_t myEUI = receiver.get_my_eui();
    const std::array<uint8_t, 6> myMacAddress = {
        myEUI.octets[0], myEUI.octets[1], myEUI.octets[2], myEUI.octets[3], myEUI.octets[4], myEUI.octets[5]
    };

    static BackchannelFlightController backchannel(
        backchannelTransceiverESPNOW,
        &backchannelMacAddress[0],
        &myMacAddress[0],
        flightController,
        ahrs,
        receiver,
        dashboardTask,
        nvs
    );

    return backchannel;
}
#endif
