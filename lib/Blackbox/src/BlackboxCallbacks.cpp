#include "BlackboxCallbacks.h"
#include "BlackboxMessageQueue.h"

#include <AHRS.h>
#include <Debug.h>
#include <FlightController.h>
#include <RadioController.h>
#include <ReceiverBase.h>
#include <cmath>


bool BlackboxCallbacks::isArmed() const
{
    // ARMING_FLAG(ARMED)
    return _flightController.motorsIsOn();
}

bool BlackboxCallbacks::areMotorsRunning() const
{
    return _flightController.motorsIsOn();
}

bool BlackboxCallbacks::isBlackboxRcModeActive() const
{
    // IS_RC_MODE_ACTIVE(BOX_BLACKBOX)
    return true;
};

bool BlackboxCallbacks::isBlackboxModeActivationConditionPresent() const
{
    //isModeActivationConditionPresent(BOX_BLACKBOX);
    return true;
}

uint32_t BlackboxCallbacks::getArmingBeepTimeMicroSeconds() const
{
    return 0;
}

uint32_t BlackboxCallbacks::rcModeActivationMask() const
{
    return 0;
}

void BlackboxCallbacks::loadSlowState(blackboxSlowState_t& slowState)
{
    //memcpy(&slowState->flightModeFlags, &_rcModeActivationMask, sizeof(slowState->flightModeFlags)); //was flightModeFlags;
    slowState.flightModeFlags = _flightController.getFlightModeFlags();
    slowState.stateFlags = 0; // this is GPS state
    slowState.failsafePhase = _radioController.getFailsafePhase();
    //slowState.rxSignalReceived = _receiver.isRxReceivingSignal();
    slowState.rxSignalReceived = (slowState.failsafePhase == RadioController::FAILSAFE_IDLE);
    slowState.rxFlightChannelsValid = (slowState.failsafePhase == RadioController::FAILSAFE_IDLE);
}

void BlackboxCallbacks::loadMainState(blackboxMainState_t& mainState, uint32_t currentTimeUs)
{

    xyz_t gyroRPS;
    xyz_t gyroRPS_unfiltered;
    xyz_t acc;
    if (_useMessageQueue) {
        (void)currentTimeUs;
        BlackboxMessageQueue::queue_item_t queueItem;
        _messageQueue.RECEIVE(queueItem);
        mainState.time = queueItem.timeMicroSeconds;
        gyroRPS = queueItem.gyroRPS;
        gyroRPS_unfiltered = queueItem.gyroRPS_unfiltered;
        acc = queueItem.acc;
    } else {
        mainState.time = currentTimeUs;
        const AHRS::data_t ahrsData = _ahrs.getAhrsDataForInstrumentationUsingLock();
        gyroRPS = ahrsData.gyroRPS;
        gyroRPS_unfiltered = ahrsData.gyroRPS_unfiltered;
        acc = ahrsData.acc;
    }

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

    constexpr float radiansToDegrees {180.0F / static_cast<float>(M_PI)};
    constexpr float gyroScale {radiansToDegrees * 10.0F};

    mainState.gyroADC[0] = static_cast<int16_t>(std::lroundf(gyroRPS.x * gyroScale));
    mainState.gyroADC[1] = static_cast<int16_t>(std::lroundf(gyroRPS.y * gyroScale));
    mainState.gyroADC[2] = static_cast<int16_t>(std::lroundf(gyroRPS.z * gyroScale));
    mainState.gyroUnfiltered[0] = static_cast<int16_t>(std::lroundf(gyroRPS_unfiltered.x * gyroScale));
    mainState.gyroUnfiltered[1] = static_cast<int16_t>(std::lroundf(gyroRPS_unfiltered.y * gyroScale));
    mainState.gyroUnfiltered[2] = static_cast<int16_t>(std::lroundf(gyroRPS_unfiltered.z * gyroScale));
    // just truncate for gyro
    mainState.accADC[0] = static_cast<int16_t>(acc.x * 4096);
    mainState.accADC[1] = static_cast<int16_t>(acc.y * 4096);
    mainState.accADC[2] = static_cast<int16_t>(acc.z * 4096);

    // iterate through roll, pitch, and yaw PIDs
    for (int ii = 0; ii < blackboxMainState_t::XYZ_AXIS_COUNT; ++ii) {
        const auto pidIndex = static_cast<FlightController::pid_index_e>(ii);
        const PIDF& pid = _flightController.getPID(pidIndex);
        const PIDF::error_t pidError = pid.getError();
        mainState.axisPID_P[ii] = std::lroundf(pidError.P);
        mainState.axisPID_I[ii] = std::lroundf(pidError.I);
        mainState.axisPID_D[ii] = std::lroundf(pidError.D);
        mainState.axisPID_F[ii] = std::lroundf(pidError.F);
        //mainState.axisPID_S[ii] = std::lroundf(pidError.S);
        mainState.setpoint[ii] = static_cast<int16_t>(std::lroundf(pid.getSetpoint()));
#if defined(USE_MAG)
        mainState.magADC[ii] = static_cast<int16_t>(mag.magADC.v[ii]);
#endif
    }
    // log the final throttle value used in the mixer
    mainState.setpoint[3] = static_cast<int16_t>(std::lroundf(_flightController.getMixer().getThrottleCommand() * 1000.0F));

    // interval [1000,2000] for THROTTLE and [-500,+500] for ROLL/PITCH/YAW
    const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM(); // returns controls in range [1000, 2000]
    mainState.rcCommand[0] = static_cast<int16_t>(controls.rollStick - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[1] = static_cast<int16_t>(controls.pitchStick - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[2] = static_cast<int16_t>(controls.yawStick - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[3] = controls.throttleStick;

    static_assert(static_cast<int>(blackboxMainState_t::DEBUG_VALUE_COUNT) == static_cast<int>(Debug::VALUE_COUNT));
    for (int ii = 0; ii < blackboxMainState_t::DEBUG_VALUE_COUNT; ++ii) {
        mainState.debug[ii] = _debug.get(ii);
    }

    const MotorMixerBase& mixer = _flightController.getMixer();
    for (size_t ii = 0; ii < mixer.getMotorCount(); ++ ii) {
        mainState.motor[ii] = static_cast<int16_t>(std::lroundf(mixer.getMotorOutput(ii)));
#if defined(USE_DSHOT_TELEMETRY)
        mainState.erpm[ii] = static_cast<int16_t>(mixer.getMotorRPM(ii));
#endif
    }
    mainState.vbatLatest = static_cast<uint16_t>(_flightController.getBatteryVoltage()*10.0F);
    mainState.amperageLatest = static_cast<uint16_t>(_flightController.getAmperage()*10.0F);

#if defined(USE_BARO)
    mainState.baroAlt = baro.altitude;
#endif

#if defined(USE_RANGEFINDER)
    // Store the raw sonar value without applying tilt correction
    //mainState.surfaceRaw = rangefinderGetLatestAltitude();
#endif

    mainState.rssi = 0;//getRssi();

#if defined(USE_SERVOS)
    mainState.servo[0] = 1527;
    mainState.servo[1] = 1473;
    mainState.servo[2] = 1620;
    mainState.servo[4] = 1750;
    //for (size_t ii = 0; ii < blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
    //    mainState.servo[ii] = 1527;
    //}
#endif
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}
