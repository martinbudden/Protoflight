#include "BackchannelESPNOW.h"


/*!
Setup the backchannel for the SBR using the ESPNOW protocol.

Specifically:

The BackchannelESPNOW class is derived from the BackchannelFlightController class
which handles packing and unpacking the packets.

The BackchannelESPNOW object owns a BackchannelTransceiverESPNOW object
which implements the ESPNOW protocol using the ESPNOW_Transceiver.

The BackchannelESPNOW and BackchannelTransceiverESPNOW classes are implemented in
the StabilizedVehicle library.a64l

The ESPNOW_Transceiver class is implemented in the Receiver library.
*/
BackchannelESPNOW::BackchannelESPNOW(
        ESPNOW_Transceiver& espnowTransceiver,
        const uint8_t* backchannelMacAddress,
        const uint8_t* myMacAddress,
        FlightController& flightController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        const TaskBase* mainTask
    ) :
    BackchannelFlightController(
        base_init_t {
            .backchannelTransceiverPtr = &_backchannelTransceiver,
            .transmitDataBufferPtr = &_transmitDataBuffer[0],
            .transmitDataBufferSize = sizeof(_transmitDataBuffer),
            .receivedDataBufferPtr = &_receivedDataBuffer[0],
            .receivedDataBufferSize = sizeof(_receivedDataBuffer)
        },
        idFromMacAddress(backchannelMacAddress), // backchannelID
        idFromMacAddress(myMacAddress), // telemetryID
        flightController,
        ahrs,
        receiver,
        preferences,
        mainTask
    ),
    _backchannelTransceiver(espnowTransceiver, backchannelMacAddress, &_receivedDataBuffer[0], sizeof(_receivedDataBuffer))
{
#if defined(USE_ESPNOW)
    static_assert(sizeof(_transmitDataBuffer) >= ESP_NOW_MAX_DATA_LEN && "transmit buffer too small");
    static_assert(sizeof(_receivedDataBuffer) >= ESP_NOW_MAX_DATA_LEN && "receive buffer too small");
#endif
}
