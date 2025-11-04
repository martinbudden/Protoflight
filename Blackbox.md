# Blackbox

`BlackboxProtoFlight` encodes system information (ie the header of the blackbox file) when blackbox is started.

`BlackboxCallbacksProtoFlight` encodes blackbox data during flight

All writing to the serial device is done via the `BlackboxEncoder`

```mermaid
classDiagram
    class TaskBase:::taskClass {
    }
    link TaskBase "https://github.com/martinbudden/Library-TaskBase/blob/main/src/TaskBase.h"
    TaskBase <|-- BlackboxTask
    class BlackboxTask {
        +loop()
        -task() [[noreturn]]
    }
    link BlackboxTask "https://github.com/martinbudden/Library-Blackbox/blob/main/src/BlackboxTask.h"
    BlackboxTask o-- MessageQueueBase : calls WAIT
    BlackboxTask o-- Blackbox : calls update
    class Blackbox {
        <<abstract>>
        virtual writeSystemInformation() *
        update()
    }
    link Blackbox "https://github.com/martinbudden/Library-Blackbox/blob/main/src/Blackbox.h"

    class MessageQueueBase {
        <<abstract>>
        virtual WAIT() *
    }
    link MessageQueueBase "https://github.com/martinbudden/Library-TaskBase/blob/main/src/MessageQueueBase.h"
    class AHRS_MessageQueue {
        ahrs_data_t ahrsData
        WAIT() override
        SEND(const ahr_data_t&)
        PEEK_TELEMETRY(const ahr_data_t&)
        getReceivedAHRS_Data()
    }
    link AHRS_MessageQueue "https://github.com/martinbudden/protoflight/blob/main/lib/Blackbox/src/AHRS_MessageQueue.h"
    class RadioControllerBase {
        <<abstract>>
    }
    link RadioControllerBase "https://github.com/martinbudden/Library-Receiver/blob/main/src/RadioControllerBase.h"
    RadioControllerBase <|-- RadioController
    class RadioController {
        getRates() rates_t  const
    }
    link RadioController "https://github.com/martinbudden/protoflight/blob/main/lib/Helm/src/RadioController.h"

    %%Blackbox <|-- BlackboxProtoFlight
    class BlackboxProtoFlight {
        writeSystemInformation() override
    }
    link BlackboxProtoFlight "https://github.com/martinbudden/protoflight/blob/main/lib/Blackbox/src/BlackboxProtoFlight.h"

    class BlackboxCallbacksBase {
        <<abstract>>
        virtual loadSlowState() *
        virtual loadMainState() *
    }
    link BlackboxCallbacksBase "https://github.com/martinbudden/Library-Blackbox/blob/main/src/BlackboxCallbacksBase.h"
    class BlackboxCallbacksProtoFlight {
        loadSlowState() override
        loadMainState() override
    }

    class ReceiverBase {
        <<abstract>>
    }
    link ReceiverBase "https://github.com/martinbudden/Library-Receiver/blob/main/src/ReceiverBase.h"

    Blackbox o-- BlackboxCallbacksBase : calls loadSlow/MainState
    Blackbox <|-- BlackboxProtoFlight : overrides writeSystemInformation
    Blackbox *-- BlackboxEncoder : calls write
    Blackbox o-- BlackboxSerialDevice : calls open close
    %%BlackboxEncoder --* Blackbox : calls write
    %%BlackboxSerialDevice --o Blackbox : calls open close
    BlackboxEncoder o-- BlackboxSerialDevice : calls write


    class FlightController {
    }
    link FlightController "https://github.com/martinbudden/protoflight/blob/main/lib/FlightController/src/FlightController.h"
    FlightController o-- AHRS_MessageQueue : calls SEND
    BlackboxCallbacksProtoFlight o-- AHRS_MessageQueue : calls getReceivedAHRS_Data
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
    link BlackboxSerialDevice "https://github.com/martinbudden/Library-Blackbox/blob/main/src/BlackboxSerialDevice.h"
    class BlackboxSerialDeviceSDCard["BlackboxSerialDeviceSDCard(eg)"]
    BlackboxSerialDevice <|-- BlackboxSerialDeviceSDCard

    classDef taskClass fill:#f96
```
