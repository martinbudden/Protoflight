#include "BlackboxCallbacks.h"
#include "Cockpit.h"
#include "FlightController.h"

#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#include <Debug.h>
#include <GPS.h>
#include <GPS_MessageQueue.h>
#include <MotorMixerBase.h>
#include <ReceiverBase.h>
#include <cmath>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif


BlackboxCallbacks::BlackboxCallbacks(const AHRS_MessageQueue& messageQueue, const FlightController& flightController, const Cockpit& cockpit, const ReceiverBase& receiver, const Debug& debug, GPS* gps) :
    _messageQueue(messageQueue),
    _flightController(flightController),
    _cockpit(cockpit),
    _rcModes(cockpit.getRC_Modes()),
    _receiver(receiver),
    _debug(debug),
    _gps(gps)
    {}

bool BlackboxCallbacks::isArmed() const
{
    return _cockpit.isArmed();
}

bool BlackboxCallbacks::areMotorsRunning() const
{
    return _flightController.motorsIsOn();
}

bool BlackboxCallbacks::isBlackboxModeActive() const
{
    return _cockpit.getRC_Modes().isModeActive(MSP_Box::BOX_BLACKBOX);
}

bool BlackboxCallbacks::isBlackboxEraseModeActive() const
{
    return _rcModes.isModeActive(MSP_Box::BOX_BLACKBOX_ERASE);
}

bool BlackboxCallbacks::isBlackboxModeActivationConditionPresent() const
{
    return _rcModes.isModeActivationConditionPresent(MSP_Box::BOX_BLACKBOX);
}

uint32_t BlackboxCallbacks::getArmingBeepTimeMicroseconds() const
{
    return 0;
}

void BlackboxCallbacks::beep() const
{
}

uint32_t BlackboxCallbacks::rcModeActivationMask() const
{
    return _cockpit.getFlightModeFlags();
}

void BlackboxCallbacks::loadSlowState(blackbox_slow_state_t& slowState)
{
    //memcpy(&slowState->flightModeFlags, &_rcModeActivationBitset, sizeof(slowState->flightModeFlags)); //was flightModeFlags;
    slowState.flightModeFlags = _cockpit.getFlightModeFlags();
    slowState.stateFlags = 0; // this is GPS state
    slowState.failsafePhase = _cockpit.getFailsafePhase();
    //slowState.rxSignalReceived = _receiver.isRxReceivingSignal();
    slowState.rxSignalReceived = (slowState.failsafePhase == Cockpit::FAILSAFE_IDLE);
    slowState.rxFlightChannelsValid = (slowState.failsafePhase == Cockpit::FAILSAFE_IDLE);
}

/*!
Called from within Blackbox::logIteration().
*/
void BlackboxCallbacks::loadMainState(blackbox_main_state_t& mainState, uint32_t currentTimeUs)
{
    (void)currentTimeUs;

    const ahrs_data_t ahrsData = _messageQueue.getReceivedAHRS_Data();
    Quaternion orientation = ahrsData.orientation;

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

    static constexpr float RADIANS_TO_DEGREES {180.0F / 3.14159265358979323846F};
    static constexpr float gyroScale {RADIANS_TO_DEGREES * 10.0F};

    mainState.gyroADC[0] = static_cast<int16_t>(std::lroundf(ahrsData.accGyroRPS.gyroRPS.x * gyroScale));
    mainState.gyroADC[1] = static_cast<int16_t>(std::lroundf(ahrsData.accGyroRPS.gyroRPS.y * gyroScale));
    mainState.gyroADC[2] = static_cast<int16_t>(std::lroundf(ahrsData.accGyroRPS.gyroRPS.z * gyroScale));
    mainState.gyroUnfiltered[0] = static_cast<int16_t>(std::lroundf(ahrsData.gyroRPS_unfiltered.x * gyroScale));
    mainState.gyroUnfiltered[1] = static_cast<int16_t>(std::lroundf(ahrsData.gyroRPS_unfiltered.y * gyroScale));
    mainState.gyroUnfiltered[2] = static_cast<int16_t>(std::lroundf(ahrsData.gyroRPS_unfiltered.z * gyroScale));
    // just truncate for acc
    mainState.accADC[0] = static_cast<int16_t>(ahrsData.accGyroRPS.acc.x * 4096);
    mainState.accADC[1] = static_cast<int16_t>(ahrsData.accGyroRPS.acc.y * 4096);
    mainState.accADC[2] = static_cast<int16_t>(ahrsData.accGyroRPS.acc.z * 4096);

    if (orientation.getW() < 0.0F) {
        // negate orientation if W is negative
        orientation = -orientation;
    }
    (void)orientation;
    //mainState.orientation[0] = static_cast<int16_t>(orientation.getX() * 0x7FFF);
    //mainState.orientation[1] = static_cast<int16_t>(orientation.getY() * 0x7FFF);
    //mainState.orientation[2] = static_cast<int16_t>(orientation.getZ() * 0x7FFF);

    // iterate through roll, pitch, and yaw PIDs
#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{blackbox_main_state_t::XYZ_AXIS_COUNT})) {
#else
    for (size_t ii = 0; ii < blackbox_main_state_t::XYZ_AXIS_COUNT; ++ii) {
#endif
        const auto pidIndex = static_cast<FlightController::pid_index_e>(ii);
        const PIDF& pid = _flightController.getPID(pidIndex);
        const PIDF::error_t pidError = pid.getError();
        mainState.axisPID_P[ii] = static_cast<int32_t>(std::lroundf(pidError.P));
        mainState.axisPID_I[ii] = static_cast<int32_t>(std::lroundf(pidError.I));
        mainState.axisPID_D[ii] = static_cast<int32_t>(std::lroundf(pidError.D));
        mainState.axisPID_S[ii] = static_cast<int32_t>(std::lroundf(pidError.S));
        mainState.axisPID_K[ii] = static_cast<int32_t>(std::lroundf(pidError.K));
        mainState.setpoint[ii] = static_cast<int16_t>(std::lroundf(pid.getSetpoint()));
#if defined(USE_MAGNETOMETER)
        mainState.magADC[ii] = static_cast<int16_t>(mag.magADC.v[ii]);
#endif
    }
    // log the final throttle value used in the mixer
    mainState.setpoint[3] = static_cast<int16_t>(std::lroundf(_flightController.getMotorMixer().get_throttle_command() * 1000.0F));

    // interval [1000,2000] for THROTTLE and [-500,+500] for ROLL/PITCH/YAW
    const receiver_controls_pwm_t controls = _receiver.get_controls_pwm(); // returns controls in range [1000, 2000]
    mainState.rcCommand[0] = static_cast<int16_t>(controls.roll - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[1] = static_cast<int16_t>(controls.pitch - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[2] = static_cast<int16_t>(controls.yaw - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[3] = static_cast<int16_t>(controls.throttle);

    static_assert(static_cast<int>(blackbox_main_state_t::DEBUG_VALUE_COUNT) == static_cast<int>(Debug::VALUE_COUNT));
    mainState.debug = _debug.getValues();

    const MotorMixerBase& motorMixer = _flightController.getMotorMixer();
#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{motorMixer.get_motor_count()})) {
#else
    for (size_t ii = 0; ii < motorMixer.get_motor_count(); ++ ii) {
#endif
        mainState.motor[ii] = static_cast<int16_t>(std::lroundf(motorMixer.get_motor_output(ii)));
#if defined(USE_DSHOT_TELEMETRY)
        mainState.erpm[ii] = static_cast<int16_t>(mixer.getMotorRPM(ii));
#endif
    }
    mainState.vbatLatest = static_cast<uint16_t>(11.6F * 10.0F);
    mainState.amperageLatest = static_cast<uint16_t>(0.67F * 10.0F);

#if defined(USE_BAROMETER)
    //mainState.baroAlt = baro.altitude;
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

void BlackboxCallbacks::loadGPS_State(blackbox_gps_state_t& gpsState)
{
    if (!_gps) {
        return;
    }
#if defined(USE_GPS)
    gps_message_data_t gpsMessageData {};
    _gps->getGPS_MessageQueue().PEEK_GPS_DATA(gpsMessageData);

    gpsState.timeOfWeek_ms = gpsMessageData.timeOfWeek_ms;
    //gpsState.interval_ms;
    //gpsState.homeLongitude_degrees1E7;
    //gpsState.homeLatitude_degrees1E7;
    //gpsState.homeAltitude_cm;
    gpsState.longitude_degrees1E7 = gpsMessageData.longitude_degrees1E7;
    gpsState.latitude_degrees1E7 = gpsMessageData.latitude_degrees1E7;
    gpsState.altitude_cm = gpsMessageData.altitude_cm;
    gpsState.velocityNorth_cmps = gpsMessageData.velocityNorth_cmps;
    gpsState.velocityEast_cmps = gpsMessageData.velocityEast_cmps;
    gpsState.velocityDown_cmps = gpsMessageData.velocityDown_cmps;
    gpsState.speed3d_cmps = gpsMessageData.speed3d_cmps;
    gpsState.groundSpeed_cmps = gpsMessageData.groundSpeed_cmps;
    gpsState.groundCourse_deciDegrees= gpsMessageData.heading_deciDegrees;
    gpsState.satelliteCount = gpsMessageData.satelliteCount;
#else
    (void)gpsState;
#endif
}
