## Backchannel

This needs refactoring. In particular `BackchannelESPNOW` should not be derived from `BackchannelFlightController`.
```mermaid
classDiagram
    class BackchannelTransceiverBase {
        <<abstract>>
        sendData() const int *
        WAIT_FOR_DATA_RECEIVED() *
        getReceivedDataLength() const size_t *
        setReceivedDataLengthToZero() *
        getTickCountDeltaAndReset() uint32_t *
    }

    BackchannelTransceiverBase <|-- BackchannelTransceiverESPNOW
    BackchannelTransceiverESPNOW o-- ESPNOW_Transceiver

    class BackchannelBase {
        <<abstract>>
        WAIT_FOR_DATA_RECEIVED()
        sendData() const int
        processedReceivedPacket() bool *
        sendPacket() bool *
    }
    BackchannelBase *-- BackchannelTransceiverBase : calls WAIT_FOR_DATA_RECEIVED

    class ReceiverBase {
        <<abstract>>
        WAIT_FOR_DATA_RECEIVED() int32_t *
        update() bool *
        getStickValues() *
        getAuxiliaryChannel() uint32_t *
    }
    ReceiverBase <|-- ReceiverAtomJoyStick
    class ReceiverAtomJoyStick
    ReceiverAtomJoyStick *-- ESPNOW_Transceiver

    BackchannelBase <|-- BackchannelStabilizedVehicle
    class BackchannelStabilizedVehicle {
        +sendPacket(uint8_t subCommand) bool override;
        #processedReceivedPacket() bool override;
        #virtual packetRequestData() bool
        #virtual packetSetOffset() bool
        #virtual packetControl() bool
        #virtual packetSetPID() bool
    }
    BackchannelStabilizedVehicle o-- DashboardTask
    BackchannelStabilizedVehicle *-- AHRS
    BackchannelStabilizedVehicle *-- SV_Preferences
    BackchannelStabilizedVehicle *-- ReceiverBase
    BackchannelStabilizedVehicle *-- VehicleControllerBase

    BackchannelStabilizedVehicle <|--BackchannelFlightController
    class BackchannelFlightController {
        +sendPacket() bool override
        #packetControl() bool override;
        #bool packetSetPID() bool override;
    }
    VehicleControllerBase <|-- FlightController
    BackchannelFlightController o-- FlightController

    BackchannelFlightController <|-- BackchannelESPNOW
    BackchannelESPNOW *-- BackchannelTransceiverESPNOW




    class TaskBase {
        uint32_t _taskIntervalMicroseconds
    }
    TaskBase <|-- BackchannelTask
    class BackchannelTask {
        +loop()
        -task() [[noreturn]]
    }
    BackchannelTask o-- BackchannelBase : calls processedReceivedPacket sendPacket
```

