# ProtoFlight

ProtoFlight is flight control software.

It has the following design goals (in no particular order)

1. A modular design that is built up from components in separate libraries (see below).
2. Produce libraries that are usable in their own right.
3. Be peformant. Support 8kHz Gyro/PID loop time.
4. Support multi-core processors, in particular allow the Gyro/PID loop to have an entire core to itself.
5. Run on bare metal or under [FREERTOS](https://www.freertos.org/).
6. Be (relatively) easy to learn and modify.
7. Give users the ability implement their own code or modifications.
8. Modular architecture to make it easier to identify which bit of code to modify, without impacting other code.
9. Be useful to people who want to experiment with and customize a flight controller.
10. Be useful to someone who wants to understand how flight control software works.

## Libraries used to build ProtoFlight

1. [VectorQuaternionMatrix](https://github.com/martinbudden/Library-VectorQuaternionMatrix.git) - 3D Vectors, Quaternions, and 3x3 Matrices.
2. [TaskBase](https://github.com/martinbudden/Library-TaskBase.git) - base class for all tasks.
3. [Filters](https://github.com/martinbudden/Library-Filters.git) - various filters.
4. [PIDF](https://github.com/martinbudden/Library-PIDF.git) - PID (Proportional Integral Derivative) controller with optional feedforward and open loop control.
5. [IMU](https://github.com/martinbudden/Library-IMU.git) (Inertial Measurement Unit) - gyroscopes and accelerometers.
6. [SensorFusion](https://github.com/martinbudden/Library-SensorFusion.git) - sensor fusion, including: Complementary Filter,
   Mahony Filter, Madgwick Filter, and Versatile Quaternion Filter (VQF).
7. [Stabilized Vehicle](https://github.com/martinbudden/Library-StabilizedVehicle.git) - AHRS (Attitude and Heading Reference System)
   and VehicleControllerBase (base class for stabilized vehicles).
8. [Receiver](https://github.com/martinbudden/Library-Receiver.git) - receiver base class and implementations, including implementation using [ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html).
9. [Backchannel](https://github.com/martinbudden/Library-Backchannel.git) - backchannel over ESP-NOW for telemetry, PID tuning, and benchmarking.
10. [StreamBuf](https://github.com/martinbudden/Library-Filters.git) - simple serializer/deserializer with no bounds checking
11. [Blackbox](https://github.com/martinbudden/Library-Blackbox.git) - ported from Betaflight Blackbox.
12. [MultiWiiSerialProtocol](https://github.com/martinbudden/Library-MultiWiiSerialProtocol.git) (MSP) - ported from Betaflight MultiWiiSerialProtocol.

## Flight Controller design/software framework

The design/software framework has undergone a number of iterations and I don't expect to make any major changes to it. However it
is still somewhat in flux and I expect there will continue to be changes. In particular:

1. The Receiver class does not yet handle telemetry. Adding this may involve structural changes to the Receiver class and/or Receiver task.
2. I'm not really happy with the name of the `RadioController` class.
  This class forms the interface between the receiver and the flight controller, and "RadioController" doesn't really describe its function properly.
  For example it is the place where functions like altitude hold, return to home, and waypointing would be implemented.
  I've thought about other names, including "FlightCommander" and "Pilot".
  I've also thought of renaming the current "FlightController" to "FlightStabilizer", and renaming "RadioController" to "FlightController".
  But none of these options seem satisfactory. Currently I'm leaning towards "Helm" or "Cockpit".
3. The `SV_Preferences` class was sufficient for storing settings for a Self Balancing Robot, however it's not really adequate for an Aircraft.
  I will need to look at alternative ways of storing settings. This may also involve removing the `SV_Preferences` and associated classes from
  the Stabilized Vehicle library.

## ProtoFlight Project

The ProtoFlight project is a "spare time" project: updates will be made when I have the time and inclination. The following are areas
that I, at some point, would like to tackle. In no particular order:

1. Implement DMA on ESP32 (DMA currently only working on RPI Pico).
2. Port to STM32 family of processors (this would enable use of many commercially available flight controllers).
3. Support additional IMUs.
4. Implement Backchannel over UDP. This would allow Backchannel to be used on RPI Pico2W.
5. Implement additional receiver protocols, including SBus and ExpressLRS.
6. Implement DShot on ESP32 (currently only implemented for RPI Pico).
7. Implement bi-directional DShot.
8. Implement saving preferences to flash on RPI Pico (currently only works on ESP32)
9. Implement saving Blackbox to flash (currently only saves to SD-card)
10. Investigate filtering the output from the PIDs and/or the inputs to the motors.
11. Investigate new ways of handling PID integral windup.

## History of the Project

The core of this code was originally developed for a [Self Balancing Robot](https://github.com/martinbudden/SelfBalancingRobot.git) (SBR).

### Self Balancing Robot design decisions

One of the early design decisions for the SBR was to split the functionality into libraries, and make each of these libraries usable
in their own right. Two of the early libraries were the vector/quaternion library and the sensor fusion library. I named the 3D vector
class `xyz_t` rather than `Vector` to avoid any confusion with the C++ `vector` container class.

An SBR runs in "Angle Mode" all the time: what is being stabilized is the pitch angle. This means the the sensor fusion algorithm
runs every iteration of the "Gyro/PID loop". So the performance of the sensor fusion is critical. There are a number of Arduino
Madgwick Sensor fusion libraries, but they all seem to be copies of each other, presumably all copied from some original implementation.
This implementation was not particularly efficient, so I wrote my own optimized version.

It turns out that for yaw rate control on an SBR you don't need a PID controller - simple open loop control (ie setting the output
proportional to the setpoint) works fine. So rather than implement a separate open loop controller for yaw, I added what I then called
a feed forward component to the PID controller - that is a term that gave output proportional to the setpoint. (It turns out that
this is not the same as Betaflight feed forward, so this was later renamed).

My original SBR implementation was on an ESP32. So ESPNOW was used to implement the receiver. ESPNOW supports several what it calls "peers"
over a radio channel, so I decided to implement a "Backchannel" that could be used for telemetry and PID tuning.

The backchannel turned out to be incredibly useful for debugging, benchmarking, and seeing the PID values in realtime, so I decided
to run the "Gyro/PID loop" all the time, even when the motors are switched off. Of course you get terrible integral windup if the motors
are off so I added functions to the PID controller to turn PID integration on and off.

### Generalization of SBR code for use in a Flight Controller

While I was writing the code for the SBR  I realized that I had written virtually everything required for a Flight Controller.
The main new requirements were:

1. a MotorMixer that could handle output to the 4 propeller motors on a quadcopter rather than the 2 wheel motors on an SBR
2. a flight controller that could stabilize the 3 axes of roll, pitch, and yaw, rather than the single axis of pitch on an SBR
3. a wider range of filters, since the vibrations on an SBR are not that severe and don't require too much filtering

So I split the `MotorPairController` I had written for the SBR into `VehicleControllerBase` and `MotorPairController` and used
`VehicleControllerBase` as a base class for a new `FlightController` class.

I originally developed the SBR on ESP32 hardware, and ESP32 has [FREERTOS](https://www.freertos.org/) built into its development software,
so it was natural to continue using FREERTOS.

It soon became clear that this rabbit hole was a bit deeper than I thought it would be.

A flight controller has many more parameters that need tuning that an SBR. The Backchannel I had developed for the SBR was very useful
for developing the FlightController, but it was not sufficient. Rather than spend a lot of effort in adding features to the Backchannel
I decided it would be better (and less effort) to port MultiWiiSerialProtocol and Blackbox from Betaflight. That way I could (in principle)
use the Betaflight configurator and Betaflight blackbox viewer. These ports are still in progress.

While porting the Blackbox and MSP I renamed and rearranged some of the ProtoFlight parameters -  this is ongoing.

While doing this port I also realized that the FeedForward in Betaflight was not the same as FeedForward in ProtoFlight.
Betaflight's FeedForward is proportional to the rate of change of the setpoint, whereas ProtoFlight's FeedForward was proportional
to the setpoint. So I changed the meaning of ProtoFlight's FeedForward, and added a new PID component (called 'S' for setpoint) which
is proportional to the setpoint. (Betaflight also has an 'S' component for PIDs, as far as I can tell this is only used for fixed wing
aircraft and is proportional to the setpoint.)

The project is still ongoing and has not yet achieved "first flight". The Self Balancing Robot has achieved "First Stabilized Drive"
so I am confident in all the lower layers of the software stack (that is layers 1-10 listed above).

I currently have ports of the software to ESP32 and Raspberry Pi Pico, but not yet to STM32.

One of the reasons for concentrating on ESP32 and RPI Pico was that they both have dual-core versions available, and one of my
primary interests is that of having the `Gyro/PID` loop
(what I snappily call the `readIMUandUpdateOrientation/updateOutputsUsingPIDs` loop) having a whole core entirely to itself.
ProtoFlight will run on a single core, but I find the dual core setup more interesting.

### Running the PIDs in *Quaternion Space*

I have on occasion toyed with the idea of running the PIDs in *Quaternion Space*. That is, instead of having 3 PIDs (one for each of
roll, pitch, and yaw) a transformation would be applied to these PIDs, mapping them onto 4 PIDs in *Quaternion Space*, one for each of
w, x, y, and z.

The idea of operating in *Quaternion Space* is not without precedent: the Madgwick Sensor Fusion Filter can in some ways be regarded as
a Complementary Filter that operates in *Quaternion Space*.

However operating PIDs in *Quaternion Space* is problematic:

1. There is little advantage for controlling roll, pitch, and yaw rates.
2. There is some advantage in using it for controlling roll and pitch angles, but here we would then replace two PIDs by four PIDs
   negating some of that advantage.
3. The mapping of PIDs in Euler Angle Space to PIDs in Quaternion Space is by no means obvious.
4. PID tuning in Quaternion Space is likely to be complicated.

I realized that there is an intermediate space between *Euler Angle Space* and *Quaternion Space*: the space defined by the sine of the
Euler Angles. So the measurement input to the rollAngle PID would be not the *rollAngle*, but would be *sin(rollAngle)*. And the setpoint
of the rollAngle PID would not be the *desiredRollAngle*, but *sin(desiredRollAngle)* (and likewise for pitchAngle). This would avoid using the
computationally expensive use of `Quaternion::calculateRollDegrees()` and `Quaternion::calculateRollDegrees()`, instead you could
use `Quaternion::sinRoll()` and `Quaternion::sinPitch()` which are relatively cheap computationally.

But, I hear you say, you've just replaced the computationally expensive `Quaternion::calculateRollDegrees()` with `sinf(desiredRollAngle)`
so there is little benefit. This is true, but `sinf(desiredRollAngle)` only changes when a new stick value is received, and so can be
calculated in the Receiver Task.

What about the fact that *sin(rollAngle)* is a non-linear function of *rollAngle*? Well, PIDs deal with non-linear systems all the time, so
I don't think that will be a problem. There also remains the possibility of changing the "rates" to compensate for this, if necessary.

Since the "*intermediate between Euler Angle and Quaternion space*" is a bit of a mouthful, and this intermediate space is closer to
*Quaternion Space than Euler Angle Space*, I'm takings some programmer's license and using the term *Quaternion Space* for this space.

Preliminary benchmarks (on an ESP32 S3 running at 240MHz) that the angle mode PID calculations take 80 microseconds with running in
*Euler Angle Space* and 50 microseconds in *Quaternion Space*. By alternatively calculating the roll angle values and pitch angle values
in each iteration of the control loop, the *Quaternion Space* calculation is reduced to 25 microseconds. This means it is feasible
to do the angle mode stabilization in the main control loop, even when it is running at 8kHz. Feasible doesn't mean possible, but this
is certainly worth investigating. For a 4kHz control loop this certainly seems possible.

## Description of Main Control Loop

This is a description of the Main Control Loop, often called the "Gyro/PID loop".
The there are variations in this loop depending on the hardware configuration, this describes operation for "ideal" hardware, namely:

1. IMU connected via SPI, with interrupt pin connected
2. IMU capable of 8kHz update frequency
3. IMU that can be read in little-endian format
4. Dual-core processor with floating point unit (FPU)

The heart of the loop is the `AHRS::readIMUandUpdateOrientation` function, invocation is as follows:

1. The IMU generates a new reading and sets its interrupt pin.
2. The IMU device driver ISR (interrupt service routine) is called and it initiates a read of the IMU via DMA (direct memory access).
3. The DMA completes. The IMU device driver converts the raw IMU data into real world units, taking into account the IMU orientation
   within the aircraft frame.
4. The IMU device driver signals the AHRS task that there is a new IMU value available.
5. The AHRS task receives the signal and calls the `AHRS::readIMUandUpdateOrientation` function.
6. **THE CLOCK STARTS NOW**. For an **8kHz** update frequency we have **125 microseconds** (see note) to complete all the following steps,
   *(Timings for each step in microseconds on an 240MHz ESP32 S3 are given where available)*.
7. The AHRS gets the IMU reading from the IMU device.
8. The AHRS sets the RPM filter for one motor using the current motor RPM values, so over 4 loops all RPM filter values are set *(1st harmonic only:16us(max25),1st+3rd:26us)*.
9. The AHRS filters the IMU reading, using the RPM filters and all other filters *(1st harmonic only:15us(max19),1st+3rd:21us)*.
10. The AHRS calculates the current orientation using sensor fusion of the gyro and accelerometers values *(20us)*.
11. The AHRS calls `FlightController::updateOutputsUsingPIDs` passing the gyro, accelerometer and orientation values *(acroMode:20us,angleModeQuaternionSpace:50us,angleModeEulerAngleSpace:80us)*.
12. If the blackbox is active, data is copied to the blackbox for logging by the Blackbox task.

Note: strictly speaking we don't have the full 125 microseconds - step 3 also eats into this time.

## Class structure

Simplified outline of the main classes.

Classes with *"(eg)"* suffix give examples of specific instances.

All objects are statically allocated in `Main::setup()`.

The `RadioController` class forms the interface between the receiver and the flight controller.
It handles failsafe.
It is the place where any intelligence (ie waypointing, return to home, crash detection etc) should be added.
RadioController is not really a good name and I am trying to think of a better one.

```mermaid
classDiagram
    class SensorFusionFilterBase {
        <<abstract>>
        update() Quaternion *
        getOrientation() Quaternion const
    }

    class IMU_Base {
        virtual readAccGyroRPS() accGyroRPS_t
    }

    class IMU_FiltersBase {
        <<abstract>>
        setFilters() *
        filter() *
    }

    class VehicleControllerBase {
        <<abstract>>
        loop() *
        updateOutputsUsingPIDs() *
        updateBlackbox() uint32_t  *
    }

    class MotorMixerBase {
        <<abstract>>
        outputToMotors() *
        getMotorRPM() int32_t *
    }
    MotorMixerBase <|-- MotorMixerQuadX_Base

    VehicleControllerBase <|-- FlightController
    class FlightController {
        array~PIDF~ _pids
        updateOutputsUsingPIDs() override
        updateBlackbox() uint32_t override
        updateSetpoints()
        updateMotorSpeedEstimates()
    }
    FlightController *-- MotorMixerBase
    FlightController o-- RadioControllerBase
    FlightController o-- Blackbox

    class AHRS {
        bool readIMUandUpdateOrientation()
    }
    AHRS *-- IMU_Base
    AHRS *-- SensorFusionFilterBase
    AHRS *-- IMU_FiltersBase
    AHRS o-- VehicleControllerBase
    VehicleControllerBase o-- AHRS

    class ReceiverBase {
        <<abstract>>
        WAIT_FOR_DATA_RECEIVED() int32_t *
        update() bool *
        getStickValues() *
        getAuxiliaryChannel() uint32_t *
    }

    class RadioControllerBase {
        <<abstract>>
        updateControls() *
        checkFailsafe() *
        getFailsafePhase() uint32_t const *
    }

    RadioController o-- FlightController
    RadioControllerBase o--ReceiverBase
    RadioControllerBase <|-- RadioController
    class RadioController {
        updateControls() override
        checkFailsafe() override
        getFailsafePhase() uint32_t const override
    }

    class RPM_Filter {
        setFrequency(size_t motorIndex, float frequencyHz)
        filter(xyz_t& input, size_t motorIndex)
    }
    IMU_FiltersBase <|-- IMU_Filters

    IMU_Filters *-- RPM_Filter

    MotorMixerQuadX_Base <|-- MotorMixerQuadX_DShot
    class MotorMixerQuadX_DShot["MotorMixerQuadX_DShot(eg)"]

    IMU_Base <|-- IMU_BMI270
    class IMU_BMI270["IMU_BMI270(eg)"]

    SensorFusionFilterBase  <|-- MadgwickFilter
    class MadgwickFilter["MadgwickFilter(eg)"] {
        update() Quaternion override
    }

    ReceiverBase <|-- ReceiverAtomJoyStick
    class ReceiverAtomJoyStick["ReceiverAtomJoyStick(eg)"]
    ReceiverAtomJoyStick *-- ESPNOW_Transceiver
```

## Task structure

Simplified outline of the tasks.

On a dual-core processor `AHRS_Task` has the second core all to itself.

The `AHRS_Task` and the `ReceiverTask` may be either interrupt driven or timer driven.<br>
All other tasks are timer driven.

Tasks are statically (build-time) polymorphic, not dynamically (run-time) polymorphic. 
They all have `task` and `loop` functions, but these functions are not virtual.
This is deliberate.

Despite its name, `MainTask` is only responsible for checking the buttons and updating the screen.
It is called such because it implements the Arduino main `loop()` function.

`BackchannelTask`, `BlackboxTask`, and `MSP_Task` are optional tasks and are not required for flight.

```mermaid
classDiagram
    class TaskBase {
        uint32_t _taskIntervalMicroSeconds
    }

    TaskBase <|-- MainTask
    class MainTask {
        +loop()
        -task() [[noreturn]]
    }
    MainTask o-- ButtonsBase : calls update
    MainTask o-- ScreenBase : calls update

    class RadioControllerBase {
        <<abstract>>
        updateControls() *
        checkFailsafe() *
        getFailsafePhase() uint32_t const *
    }
    class ReceiverBase {
        <<abstract>>
        WAIT_FOR_DATA_RECEIVED() int32_t *
        update() bool *
        getStickValues() *
        getAuxiliaryChannel() uint32_t *
    }
    class FlightController {
        array~PIDF~ _pids
        updateOutputsUsingPIDs() override
        updateBlackbox() uint32_t override
        updateSetpoints()
        updateMotorSpeedEstimates()
    }
    FlightController o-- RadioControllerBase : calls getFailsafePhase
    RadioControllerBase o--ReceiverBase
    RadioControllerBase <|-- RadioController
    RadioController o-- FlightController : calls updateSetpoints

    TaskBase <|-- ReceiverTask
    class ReceiverTask {
        +loop()
        -task() [[noreturn]]
    }
    class ReceiverWatcher {
        <<abstract>>
        newReceiverPacketAvailable() *
    }
    ReceiverTask o-- ReceiverWatcher : calls newReceiverPacketAvailable
    ReceiverTask o-- ReceiverBase : calls WAIT_FOR_DATA_RECEIVED update getStickValues
    ReceiverWatcher <|-- ScreenBase
    ReceiverTask o-- RadioControllerBase : calls updateControls checkFailsafe


    class VehicleControllerBase {
        <<abstract>>
        loop() *
        updateOutputsUsingPIDs() *
        updateBlackbox() uint32_t  *
    }
    TaskBase <|-- VehicleControllerTask
    class VehicleControllerTask {
        +loop()
        -task() [[noreturn]]
    }
    VehicleControllerTask o-- VehicleControllerBase : calls loop
    VehicleControllerBase <|-- FlightController

    TaskBase <|-- AHRS_Task
    class AHRS_Task {
        +loop()
        -task() [[noreturn]]
    }
    AHRS_Task o-- AHRS : calls readIMUandUpdateOrientation

    class AHRS {
        bool readIMUandUpdateOrientation()
    }
    AHRS o-- VehicleControllerBase : calls updateOutputsUsingPIDs
    VehicleControllerBase o-- AHRS

    class Backchannel {
        processedReceivedPacket() bool
    }
    TaskBase <|-- BackchannelTask
    class BackchannelTask {
        +loop()
        -task() [[noreturn]]
    }
    BackchannelTask o-- Backchannel : calls processedReceivedPacket

    TaskBase <|-- BlackboxTask
    class BlackboxTask {
        +loop()
        -task() [[noreturn]]
    }
    BlackboxTask o-- Blackbox : calls update
    Blackbox <|-- BlackboxProtoFlight
    class Blackbox {
        update() uint32_t
    }

    TaskBase <|-- MSP_Task
    class MSP_Task {
        +loop()
        -task() [[noreturn]]
    }
    MSP_Task o-- MSP_SerialBase : calls processInput
    MSP_SerialBase <|-- MSP_Serial
    class MSP_SerialBase {
        <<abstract>>
        sendFrame() int *
        processInput() *
    }
```

## Blackbox

`BlackboxProtoFlight` writes system information (ie the header of the blackbox file) when blackbox is started.

`BlackboxCallbacksProtoFlight` writes blackbox data during flight

```mermaid
classDiagram
    class Blackbox {
        virtual writeSystemInformation() write_e *
        virtual update() uint32_t
    }
    Blackbox *-- BlackboxEncoder
    Blackbox o-- BlackboxSerialDevice
    Blackbox o-- BlackboxCallbacksBase

    RadioControllerBase <|-- RadioController
    class RadioController {
        getRates() rates_t  const
    }

    Blackbox <|-- BlackboxProtoFlight
    class BlackboxProtoFlight {
        write_e writeSystemInformation() override
    }
    BlackboxProtoFlight o-- FlightController
    FlightController o-- BlackboxProtoFlight
    BlackboxProtoFlight o-- RadioController

    class BlackboxCallbacksBase {
        virtual void loadSlowStateFromFlightController() *
        virtual void loadMainStateFromFlightController() *
    }

    BlackboxCallbacksBase <|-- BlackboxCallbacksProtoFlight
    class BlackboxCallbacksProtoFlight {
        void loadSlowStateFromFlightController() override
        void loadMainStateFromFlightController() override
    }
    BlackboxCallbacksProtoFlight o-- AHRS
    BlackboxCallbacksProtoFlight o-- ReceiverBase
    BlackboxCallbacksProtoFlight o-- RadioControllerBase
    BlackboxCallbacksProtoFlight o-- FlightController

    BlackboxSerialDevice <|-- BlackboxSerialDeviceSDCard~eg~

    TaskBase <|-- BlackboxTask
    class BlackboxTask {
        +loop()
        -task() [[noreturn]]
    }
    BlackboxTask o-- Blackbox : calls update
```

## MSP

`MSP_Base` has the virtual functions `processOutCommand` and `processInCommand` which are overridden in `MSP_ProtoFlight`.

`MSP_ProtoFlight` overrides these functions to set and get values in the main flight objects (ie `FlightController`, `AHRS` etc)
when MSP commands are received.

```mermaid
classDiagram
    class MSP_SerialBase {
        <<abstract>>
        sendFrame() int *
        processInput() *
    }
    class MSP_Base {
        virtual processOutCommand() result_e
        virtual processInCommand() result_e
    }
    class MSP_Stream {
    }
    MSP_Stream *-- MSP_Base
    MSP_Stream o-- MSP_SerialBase

    MSP_SerialBase <|-- MSP_Serial
    class MSP_Serial {
        sendFrame() int override
        processInput() override
    }
    MSP_Serial o-- MSP_Stream

    MSP_Base <|-- MSP_ProtoFlight
    class MSP_ProtoFlight {
        processOutCommand() result_e override
        processInCommand() result_e override
    }
    MSP_ProtoFlight *-- MSP_ProtoBox
    MSP_ProtoFlight o-- Features
    MSP_ProtoFlight o-- AHRS
    MSP_ProtoFlight o-- FlightController
    MSP_ProtoFlight o-- RadioController
    MSP_ProtoFlight o-- ReceiverBase

    MSP_Box <|-- MSP_ProtoBox

    TaskBase <|-- MSP_Task
    class MSP_Task {
        +loop()
        -task() [[noreturn]]
    }
    MSP_Task o-- MSP_SerialBase : calls processInput
```
