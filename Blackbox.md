# Blackbox

`BlackboxProtoFlight` encodes system information (ie the header of the blackbox file) when blackbox is started.

`BlackboxCallbacksProtoFlight` encodes blackbox data during flight

All writing to the serial device is done via the `BlackboxEncoder`

```mermaid
classDiagram
    TaskBase <|-- BlackboxTask
    class BlackboxTask {
        +loop()
        -task() [[noreturn]]
    }
    BlackboxTask o-- MessageQueueBase : calls WAIT
    BlackboxTask o-- Blackbox : calls update
    class Blackbox {
        <<abstract>>
        virtual writeSystemInformation() write_e *
        virtual update() uint32_t
    }

    class MessageQueueBase {
        <<abstract>>
        virtual WAIT() const * int32_t
    }
    class AHRS_MessageQueue {
        ahrs_data_t ahrsData
        WAIT() override
        SEND(const ahr_data_t&)
        PEEK_COPY(const ahr_data_t&)
        getAHRS_Data()
    }
    class RadioControllerBase {
        <<abstract>>
    }
    RadioControllerBase <|-- RadioController
    class RadioController {
        getRates() rates_t  const
    }

    %%Blackbox <|-- BlackboxProtoFlight
    class BlackboxProtoFlight {
        write_e writeSystemInformation() override
    }

    class BlackboxCallbacksBase {
        <<abstract>>
        _queueItem queue_item_t
        virtual void loadSlowState() *
        virtual void loadMainState() *
    }
    class BlackboxCallbacksProtoFlight {
        loadSlowState() override
        loadMainState() override
    }

    class ReceiverBase {
        <<abstract>>
    }

    Blackbox o-- BlackboxCallbacksBase : calls loadState
    Blackbox <|-- BlackboxProtoFlight
    Blackbox *-- BlackboxEncoder : calls write
    Blackbox o-- BlackboxSerialDevice : calls open close
    %%BlackboxEncoder --* Blackbox : calls write
    %%BlackboxSerialDevice --o Blackbox : calls open close
    BlackboxEncoder o-- BlackboxSerialDevice : calls write


    FlightController o-- AHRS_MessageQueue : calls SEND
    BlackboxCallbacksProtoFlight o-- AHRS_MessageQueue : calls getQueueItem
    MessageQueueBase <|-- AHRS_MessageQueue
    BlackboxCallbacksProtoFlight o-- FlightController : calls getPID
    BlackboxCallbacksProtoFlight o-- ReceiverBase : calls getControls
    BlackboxCallbacksProtoFlight o-- RadioControllerBase : calls getFailSafePhase
    %%FlightController --o BlackboxCallbacksProtoFlight
    %%FlightController o-- Blackbox : calls start finish
    %%Blackbox --o FlightController : calls start finish
    BlackboxCallbacksBase <|-- BlackboxCallbacksProtoFlight
    %%BlackboxCallbacksProtoFlight --|> BlackboxCallbacksBase

    %%FlightController --o BlackboxProtoFlight
    %%Blackbox --o FlightController
    BlackboxProtoFlight o-- FlightController
    RadioController --o BlackboxProtoFlight : calls getRates
    %%BlackboxProtoFlight o-- RadioController : calls getRates

    class BlackboxSerialDevice {
        <<abstract>>
    }
    class BlackboxSerialDeviceSDCard["BlackboxSerialDeviceSDCard(eg)"]
    BlackboxSerialDevice <|-- BlackboxSerialDeviceSDCard

```

