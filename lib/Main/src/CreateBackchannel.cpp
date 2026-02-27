#include "Main.h"

#include <BackchannelFlightController.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <backchannel_transceiver_espnow.h>
#endif
#include <receiver_atom_joystick.h>


#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
BackchannelBase& Main::createBackchannel(ReceiverBase& receiver)
{
    // Statically allocate the backchannel.
    static constexpr uint8_t backchannel_mac_address[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    auto& receiver_atom_joystick = static_cast<ReceiverAtomJoystick&>(receiver); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    static BackchannelTransceiverEspnow backchannel_transceiver_espnow(receiver_atom_joystick.get_espnow_transceiver(), &backchannel_mac_address[0]);

    const ReceiverBase::EUI_48_t my_eui = receiver.get_my_eui();
    const std::array<uint8_t, 6> my_mac_address = {
        my_eui.octets[0], my_eui.octets[1], my_eui.octets[2], my_eui.octets[3], my_eui.octets[4], my_eui.octets[5]
    };

    static BackchannelFlightController backchannel(
        backchannel_transceiver_espnow,
        &backchannel_mac_address[0],
        &my_mac_address[0]
    );

    return backchannel;
}
#endif
