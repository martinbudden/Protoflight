```mermaid
---
  config:
    class:
      hideEmptyMembersBox: true
---
classDiagram
    class TaskBase:::taskClass {
    }
    link TaskBase "https://github.com/martinbudden/Library-TaskBase/blob/main/src/task_base.h"
    TaskBase <|-- AHRS_Task
    TaskBase <|-- VehicleControllerTask
    TaskBase <|-- ReceiverTask
    TaskBase <|-- BlackboxTask

    class AHRS_Task:::taskClass {
    }
    link AHRS_Task "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/AHRS_Task.h"
    AHRS_Task o-- IMU_Base : calls WAIT_IMU_DATA_READY
    AHRS_Task --o IMU_Base : calls SIGNAL
    AHRS_Task o-- AHRS : calls readIMUandUpdateOrientation
    class ImuBase {
        <<abstract>>
        WAIT_IMU_DATA_READY()
        SIGNAL_IMU_DATA_READY_FROM_ISR()
        virtual readAccGyroRPS() accGyroRPS_t
    }
    link IMU_Base "https://github.com/martinbudden/Library-Sensors/blob/main/src/IMU_Base.h"

    IMU_Base <|-- IMU_BMI270 : overrides readAccGyroRPS
    class IMU_BMI270["IMU_BMI270(eg)"]
    link IMU_BMI270 "https://github.com/martinbudden/Library-Sensors/blob/main/src/IMU_BMI270.h"

    class ImuFiltersBase {
        <<abstract>>
        filter() *
    }
    link ImuFiltersBase "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/ImuFiltersBase.h"

    class DynamicNotchFilter {
        array~BiquadFilter~ notchFilters
        filter()
    }
    link DynamicNotchFilter "https://github.com/martinbudden/protoflight/blob/main/lib/FlightController/src/DynamicNotchFilter.h"

    class RPM_Filters {
        array~BiquadFilterT~xyz_t~~ _filters[MOTORS][HARMONICS]
        setFrequencyHzIterationStep()
        filter()
    }
    link RPM_Filters "https://github.com/martinbudden/protoflight/blob/main/lib/FlightController/src/RPM_Filters.h"

    class IMU_Filters {
        FilterT~xyz_t~  _gyroLPF
        BiquadFilterT~xyz_t~ _gyroNotch
        filter() override
    }
    link IMU_Filters "https://github.com/martinbudden/protoflight/blob/main/lib/FlightController/src/IMU_Filters.h"
    IMU_Filters o-- DynamicNotchFilter : calls filter
    %%DynamicNotchFilter --o IMU_Filters : calls filter
    IMU_Filters o-- RPM_Filters : calls filter
    ImuFiltersBase <|-- IMU_Filters : overrides filter
    %%IMU_Filters o-- MotorMixerBase

    class SensorFusionFilterBase {
        Quaternion orientation
        <<abstract>>
        updateOrientation() Quaternion *
    }
    link SensorFusionFilterBase "https://github.com/martinbudden/Library-SensorFusion/blob/main/src/sensor_fusion.h"

    SensorFusionFilterBase  <|-- MadgwickFilter : overrides updateOrientation
    class MadgwickFilter["MadgwickFilter(eg)"] {
        updateOrientation() override
    }
    link MadgwickFilter "https://github.com/martinbudden/Library-SensorFusion/blob/main/src/sensor_fusion.h"

    class AHRS {
        bool readIMUandUpdateOrientation()
    }
    link AHRS "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/ahrs.h"
    AHRS o-- IMU_Base : calls readAccGyroRPS
    AHRS o-- ImuFiltersBase : calls filter
    AHRS o-- SensorFusionFilterBase : calls updateOrientation
    AHRS o-- VehicleControllerBase : calls updateOutputsUsingPIDs / SIGNAL
    class VehicleControllerBase {
        <<abstract>>
        WAIT()
        SIGNAL()
        outputToMixer() *
        updateOutputsUsingPIDs() *
    }
    link VehicleControllerBase "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/vehicle_controller_base.h"

    class VehicleControllerTask:::taskClass {
    }
    link VehicleControllerTask "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/VehicleControllerTask.h"
    %%VehicleControllerBase --o VehicleControllerTask : calls WAIT / outputToMixer
    VehicleControllerTask o-- VehicleControllerBase : calls WAIT / outputToMixer

    class VehicleControllerMessageQueue {
        WAIT()
        SIGNAL()
    }
    link VehicleControllerMessageQueue "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/VehicleControllerMessageQueue.h"
    class MotorMixerBase {
        <<abstract>>
        outputToMotors() *
    }
    link MotorMixerBase "https://github.com/martinbudden/protoflight/blob/main/lib/MotorMixers/src/motor_mixer_base.h"

    class DynamicIdleController {
        calculateSpeedIncrease()
    }
    link DynamicIdleController "https://github.com/martinbudden/protoflight/blob/main/lib/MotorMixers/src/dynamic_idle_controller.h"

    class MotorMixerQuadX_DShot["MotorMixerQuadX_DShot(eg)"]
    link MotorMixerQuadX_DShot "https://github.com/martinbudden/protoflight/blob/main/lib/MotorMixers/src/MotorMixerQuadX_DShot.h"
    MotorMixerQuadX_DShot *-- RPM_Filters : calls setFrequencyHzIterationStep
    %%RPM_Filters --* MotorMixerQuadX_DShot : calls setFrequencyHzIterationStep
    MotorMixerQuadX_DShot *-- DynamicIdleController : calls calculateSpeedIncrease

    MotorMixerBase <|-- MotorMixerQuadX_Base : overrides outputToMotors
    MotorMixerQuadX_Base <|-- MotorMixerQuadX_DShot : overrides getMotorFrequencyHz
    class MotorMixerQuadX_Base {
        array~float,4~ _motorOutputs
    }
    link MotorMixerQuadX_Base "https://github.com/martinbudden/protoflight/blob/main/lib/MotorMixers/src/MotorMixerQuadX_Base.h"

    class FlightController {
        array~PIDF~ _pids
        array~Filter~ _dTermFilters
        array~Filter~ _stickSetpointFilters
        outputToMixer() override
        updateOutputsUsingPIDs() override
        updateSetpoints()
    }
    link FlightController "https://github.com/martinbudden/protoflight/blob/main/lib/FlightController/src/FlightController.h"
    VehicleControllerBase *-- VehicleControllerMessageQueue : calls WAIT / SIGNAL
    VehicleControllerBase <|-- FlightController : overrides outputToMixer updateOutputsUsingPIDs
    FlightController o-- MotorMixerBase : calls outputToMotors

    class CockpitBase {
        <<abstract>>
        updateControls() *
        checkFailsafe() *
    }
    link CockpitBase "https://github.com/martinbudden/Library-Receiver/blob/main/src/CockpitBase.h"
    CockpitBase <|-- Cockpit : overrides updateControls
    CockpitBase o--ReceiverBase : calls getAuxiliaryChannel

    class Cockpit {
        updateControls() override
        checkFailsafe() override
        getFailsafePhase()
    }
    link Cockpit "https://github.com/martinbudden/protoflight/blob/main/lib/Helm/src/Cockpit.h"
    Cockpit o-- FlightController : calls updateSetpoints
    Cockpit o-- Autopilot : calls calculateFlightControls

    class ReceiverTask:::taskClass {
    }
    link ReceiverTask "https://github.com/martinbudden/Library-Receiver/blob/main/src/ReceiverTask.h"

    ReceiverTask o-- CockpitBase : calls updateControls
    ReceiverTask o-- ReceiverBase : calls update / getStickValues
    %%CockpitBase --o ReceiverTask : calls updateControls

    class Autopilot {
        array~geographic_coordinate_t~ _waypoints
        altitudeHoldCalculateThrottle()
        calculateFlightControls()
    }
    link Autopilot "https://github.com/martinbudden/protoflight/blob/main/lib/Helm/src/Autopilot.h"
    Autopilot o-- AhrsMessageQueue : calls PEEK_AHRS_DATA

    class AhrsMessageQueue {
        ahrs_data_t ahrsData
        WAIT() override
        SIGNAL(const ahr_data_t&)
        SEND_AHRS_DATA(const ahr_data_t&)
        PEEK_AHRS_DATA(const ahr_data_t&)
        getReceivedAHRS_Data()
    }
    link AhrsMessageQueue "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/AhrsMessageQueue.h"
    FlightController o-- AhrsMessageQueue : calls SIGNAL
    %%AhrsMessageQueue --o FlightController : calls SIGNAL

    class ReceiverBase {
        <<abstract>>
        WAIT_FOR_DATA_RECEIVED() *
        update() *
        getStickValues() *
        getAuxiliaryChannel() *
    }
    link ReceiverBase "https://github.com/martinbudden/Library-Receiver/blob/main/src/ReceiverBase.h"
    class ReceiverAtomJoyStick {
        update() override
    }
    ReceiverBase <|-- ReceiverAtomJoyStick
    class ReceiverAtomJoyStick["ReceiverAtomJoyStick(eg)"]
    link ReceiverAtomJoyStick "https://github.com/martinbudden/Library-Receiver/blob/main/src/ReceiverAtomJoyStick.h"

    class BlackboxTask:::taskClass {
    }
    link BlackboxTask "https://github.com/martinbudden/Library-Blackbox/blob/main/src/BlackboxTask.h"
    BlackboxTask o-- AhrsMessageQueue : calls WAIT
    BlackboxTask o-- Blackbox : calls update
    class Blackbox {
        <<abstract>>
        write_system_information() *
        start()
        finish()
        update() uint32_t
    }
    link Blackbox "https://github.com/martinbudden/Library-Blackbox/blob/main/src/blackbox.h"
    Blackbox <|-- BlackboxProtoFlight
    class BlackboxProtoFlight {
        write_system_information() override
    }
    link BlackboxProtoFlight "https://github.com/martinbudden/protoflight/blob/main/lib/Blackbox/src/BlackboxProtoFlight.h"

    classDef taskClass fill:#f96
```
