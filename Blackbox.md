# Blackbox

`BlackboxProtoFlight` encodes system information (ie the header of the blackbox file) when blackbox is started.

`BlackboxCallbacksProtoFlight` encodes blackbox data during flight

All writing to the serial device is done via the `BlackboxEncoder`

```mermaid
classDiagram
    class Blackbox {
        <<abstract>>
        virtual writeSystemInformation() write_e *
        virtual update() uint32_t
    }

    class BlackboxMessageQueue {
        RECEIVE(queue_item_t& queueItem) int32_t
        SEND(const queue_item_t& queueItem)
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
        setQueueItem()
        virtual void loadSlowState() *
        virtual void loadMainState() *
    }
    class BlackboxCallbacksProtoFlight {
        void loadSlowState() override
        void loadMainState() override
    }

    class ReceiverBase {
        <<abstract>>
    }

    Blackbox o-- BlackboxCallbacksBase : calls loadState
    Blackbox <|-- BlackboxProtoFlight
    BlackboxEncoder --* Blackbox : calls write
    BlackboxSerialDevice --o Blackbox : calls open close
    BlackboxEncoder o-- BlackboxSerialDevice : calls write


    BlackboxCallbacksBase o-- BlackboxMessageQueue
    BlackboxCallbacksProtoFlight o-- ReceiverBase : calls getControls
    BlackboxCallbacksProtoFlight o-- RadioControllerBase : calls getFailSafePhase
    BlackboxCallbacksProtoFlight o-- FlightController : calls getPID
    %%FlightController --o BlackboxCallbacksProtoFlight 
    %%FlightController o-- Blackbox : calls start finish
    Blackbox --o FlightController : calls start finish
    BlackboxCallbacksBase <|-- BlackboxCallbacksProtoFlight
    %%BlackboxCallbacksProtoFlight --|> BlackboxCallbacksBase

    FlightController --o BlackboxProtoFlight
    %%Blackbox --o FlightController
    %%BlackboxProtoFlight o-- FlightController
    RadioController --o BlackboxProtoFlight : calls getRates

    class BlackboxSerialDevice {
        <<abstract>>
    }
    class BlackboxSerialDeviceSDCard["BlackboxSerialDeviceSDCard(eg)"]
    BlackboxSerialDevice <|-- BlackboxSerialDeviceSDCard

    TaskBase <|-- BlackboxTask
    class BlackboxTask {
        +loop()
        -task() [[noreturn]]
    }
    BlackboxTask o-- BlackboxMessageQueue : calls RECEIVE
    BlackboxTask o-- BlackboxCallbacksBase : calls setQueueItem
    BlackboxTask o-- Blackbox : calls update
```

