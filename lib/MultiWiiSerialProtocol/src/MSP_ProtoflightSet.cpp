#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"

#include <AHRS.h>
#include <MSP_Protocol.h>
#include <ReceiverBase.h>


MSP_Base::result_e MSP_Protoflight::processSetCommand(int16_t cmdMSP, StreamBufReader& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) // NOLINT(readability-function-cognitive-complexity)
{
    (void)srcDesc;
    (void)postProcessFn;
    // const size_t dataSize = src.bytesRemaining();

    switch (cmdMSP) {
    case MSP_SELECT_SETTING:
        src.readU8(); // PID profile and Rate profile
        break;
    case MSP_COPY_PROFILE:
        return RESULT_ERROR;
    case MSP_SET_RAW_RC:
        return RESULT_ERROR;
    case MSP_SET_ARMING_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_PID_CONTROLLER:
        return RESULT_ERROR;
    case MSP_SET_PID: {
        for (size_t ii = 0; ii <= FlightController::YAW_RATE_DPS; ++ii) {
            const auto pidIndex = static_cast<FlightController::pid_index_e>(ii);
            _flightController.setPID_P_MSP(pidIndex, src.readU8());
            _flightController.setPID_I_MSP(pidIndex, src.readU8());
            _flightController.setPID_D_MSP(pidIndex, src.readU8());
        }
        const uint8_t kp = src.readU8();
        const uint8_t ki = src.readU8();
        const uint8_t kd = src.readU8();
        _flightController.setPID_P_MSP(FlightController::ROLL_ANGLE_DEGREES, kp);
        _flightController.setPID_I_MSP(FlightController::ROLL_ANGLE_DEGREES, ki);
        _flightController.setPID_D_MSP(FlightController::ROLL_ANGLE_DEGREES, kd);
        _flightController.setPID_P_MSP(FlightController::PITCH_ANGLE_DEGREES, kp);
        _flightController.setPID_I_MSP(FlightController::PITCH_ANGLE_DEGREES, ki);
        _flightController.setPID_D_MSP(FlightController::PITCH_ANGLE_DEGREES, kd);
        // skip over PID_MAG
        src.readU8();
        src.readU8();
        src.readU8();
        break;
    }
    case MSP_SET_MODE_RANGE:
        return RESULT_ERROR;
    case MSP_SET_ADJUSTMENT_RANGE:
        return RESULT_ERROR;
    case MSP_SET_RC_TUNING: {
        if (src.bytesRemaining() < 10) {
            return RESULT_ERROR;
        }
        Cockpit::rates_t rates = _cockpit.getRates();
        uint8_t value = src.readU8();
        if (rates.rcRates[Cockpit::PITCH] == rates.rcRates[Cockpit::ROLL]) {
            rates.rcRates[Cockpit::PITCH] = value;
        }
        rates.rcRates[Cockpit::ROLL] = value;

        value = src.readU8();
        if (rates.rcExpos[Cockpit::PITCH] == rates.rcExpos[Cockpit::ROLL]) {
            rates.rcExpos[Cockpit::PITCH] = value;
        }
        rates.rcExpos[Cockpit::ROLL] = value;

        rates.rcRates[Cockpit::ROLL] = src.readU8();
        rates.rcRates[Cockpit::PITCH] = src.readU8();
        rates.rcRates[Cockpit::YAW] = src.readU8();
        src.readU8(); // skip tpa_rate
        rates.throttleMidpoint = src.readU8();
        rates.throttleExpo = src.readU8();
        src.readU16(); // skip tpa_breakpoint

        if (src.bytesRemaining() >= 1) {
            rates.rcExpos[Cockpit::YAW] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcRates[Cockpit::YAW] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcRates[Cockpit::PITCH] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcExpos[Cockpit::PITCH] = src.readU8();
        }
        // version 1.41
        if (src.bytesRemaining() >= 2) {
            rates.throttleLimitType = src.readU8();
            rates.throttleLimitPercent = src.readU8();
        }
        // version 1.42
        if (src.bytesRemaining() >= 6) {
            rates.rateLimits[Cockpit::ROLL] = src.readU16();
            rates.rateLimits[Cockpit::PITCH] = src.readU16();
            rates.rateLimits[Cockpit::YAW] = src.readU16();
        }
        // version 1.43
        if (src.bytesRemaining() >= 1) {
            src.readU8(); // hardcoded to RATES_TYPE_ACTUAL
        }
        _cockpit.setRates(rates);
        break;
    }
    case MSP_SET_MOTOR_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_GPS_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_COMPASS_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_GPS_RESCUE:
        return RESULT_ERROR;
    case MSP_SET_MOTOR:
        return RESULT_ERROR;
    case MSP_SET_SERVO_CONFIGURATION:
        return RESULT_ERROR;
    case MSP_SET_SERVO_MIX_RULE:
        return RESULT_ERROR;
    case MSP_SET_MOTOR_3D_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_RC_DEADBAND:
        return RESULT_ERROR;
    case MSP_SET_RESET_CURR_PID:
        return RESULT_ERROR;
    case MSP_SET_SENSOR_ALIGNMENT: {
        // maintain backwards compatibility for API < 1.41
        //const uint8_t gyroAlignment = src.readU8();
        //const uint8_t axisOrder = src.readU8();
        src.readU8();
        //_ahrs.getIMU().setAxisOrder(static_cast<IMU_Base::axis_order_e>(axisOrder));
        src.readU8();  // discard deprecated acc_align
        // MAG
        src.readU8();
        break;
    }
    case MSP_SET_ADVANCED_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_FILTER_CONFIG: {
        auto& imuFilters = static_cast<IMU_Filters&>(_ahrs.getIMU_Filters()); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
        IMU_Filters::config_t imuFiltersConfig = imuFilters.getConfig();
        RPM_Filters* rpmFilters = imuFilters.getRPM_Filters();
        RPM_Filters::config_t rpmFiltersConfig = rpmFilters ? rpmFilters->getConfig() : RPM_Filters::config_t {}; // cppcheck-suppress knownConditionTrueFalse
        FlightController::filters_config_t fcFilters = _flightController.getFiltersConfig();

        imuFiltersConfig.gyro_lpf1_hz = src.readU8();
        fcFilters.dterm_lpf1_hz = src.readU16();
        fcFilters.yaw_lpf_hz = src.readU16();
        if (src.bytesRemaining() >= 8) {
            imuFiltersConfig.gyro_notch1_hz = src.readU16();
            imuFiltersConfig.gyro_notch1_cutoff = src.readU16();
#if defined(USE_DTERM_FILTERS_EXTENDED)
            fcFilters.dterm_notch_hz = src.readU16();
            fcFilters.dterm_notch_cutoff = src.readU16();
#else
            src.readU16();
            src.readU16();
#endif
        }
        if (src.bytesRemaining() >= 4) {
            imuFiltersConfig.gyro_notch2_hz = src.readU16();
            imuFiltersConfig.gyro_notch2_cutoff = src.readU16();
        }
        if (src.bytesRemaining() >= 1) {
#if defined(USE_DTERM_FILTERS_EXTENDED)
            fcFilters.dterm_lpf1_type = src.readU8();
#else
            src.readU8();
#endif
        }
        if (src.bytesRemaining() >= 10) {
            src.readU8(); // ignored gyro_hardware_lpf set in driver
            src.readU8(); // was gyro_32khz_hardware_lpf
            imuFiltersConfig.gyro_lpf1_hz = src.readU16();
            imuFiltersConfig.gyro_lpf2_hz = src.readU16();
            imuFiltersConfig.gyro_lpf1_type = src.readU8();
            imuFiltersConfig.gyro_lpf2_type = src.readU8();
            fcFilters.dterm_lpf2_hz = src.readU16();
        }

        if (src.bytesRemaining() >= 9) {
            // Added in MSP API 1.41
#if defined(USE_DTERM_FILTERS_EXTENDED)
            fcFilters.dterm_lpf2_type = src.readU8();
#else
            src.readU8();
#endif
            /*imuFiltersConfig.gyro_dynamic_lpf1_min_hz = */src.readU16();
            /*imuFiltersConfig.gyro_dynamic_lpf1_max_hz = */src.readU16();
#if defined(USE_DTERM_FILTERS_EXTENDED)
            fcFilters.dterm_dynamic_lpf1_min_hz = src.readU16();
            fcFilters.dterm_dynamic_lpf1_max_hz = src.readU16();
#else
            src.readU16();
            src.readU16();
#endif
        }

        if (src.bytesRemaining() >= 8) {
            // Added in MSP API 1.42
            src.readU8(); // DEPRECATED 1.43: dyn_notch_range
            src.readU8(); // DEPRECATED 1.44: dyn_notch_width_percent
            //dynamic_notch_q =
            src.readU16();
            //dynamic_notch_min_hz =
            src.readU16();
            rpmFiltersConfig.rpm_filter_harmonics = src.readU8();
            rpmFiltersConfig.rpm_filter_min_hz = src.readU8();
        }
        if (src.bytesRemaining() >= 2) {
            // Added in MSP API 1.43
            //dynamic_notch_max_hz =
            src.readU16();
        }
        if (src.bytesRemaining() >= 2) {
            // Added in MSP API 1.44
            // dterm_lpf1_dyn_expo =
            src.readU8();
            // dynamic_notch_count =
            src.readU8();
        }
        imuFilters.setConfig(imuFiltersConfig);
        if (rpmFilters) { // cppcheck-suppress knownConditionTrueFalse
            rpmFilters->setConfig(rpmFiltersConfig);
        }
        _flightController.setFiltersConfig(fcFilters);
        break;
    }
    case MSP_SET_PID_ADVANCED: {
        src.readU16();
        src.readU16();
        src.readU16(); // was yaw_p_limit
        src.readU8(); // reserved
        src.readU8(); // was vbatPidCompensation
        src.readU8(); // !!TODO::feedforward_transition
        src.readU8(); // was low byte of dtermSetpointWeight
        src.readU8(); // reserved
        src.readU8(); // reserved
        src.readU8(); // reserved
        src.readU16(); // !!TODO: rateAccelLimit
        src.readU16(); // !!TODO: yawRateAccelLimit
        if (src.bytesRemaining() >= 2) {
            src.readU8(); // !!TODO: angle_limit
            src.readU8(); // was levelSensitivity
        }
        if (src.bytesRemaining() >= 4) {
            src.readU16(); // was currentPidProfile->itermThrottleThreshold
            src.readU16(); // !!TODO: anti_gravity_gain
        }
        if (src.bytesRemaining() >= 2) {
            src.readU16(); // was currentPidProfile->dtermSetpointWeight
        }
        if (src.bytesRemaining() >= 14) {
            // Added in MSP API 1.40
            src.readU8(); // !!TODO: iterm_rotation
            src.readU8(); // was currentPidProfile->smart_feedforward
            src.readU8(); // !!TODO: iterm_relax
            src.readU8(); // !!TODO: iterm_relax_type
            src.readU8(); // !!TODO: abs_control_gain
            src.readU8(); // !!TODO: throttle_boost
            src.readU8(); // !!TODO: acro_trainer_angle_limit
            // PID controller kick terms
            _flightController.setPID_K_MSP(FlightController::ROLL_RATE_DPS, src.readU16());
            _flightController.setPID_K_MSP(FlightController::PITCH_RATE_DPS, src.readU16());
            _flightController.setPID_K_MSP(FlightController::YAW_RATE_DPS, src.readU16());
            src.readU8(); // was currentPidProfile->antiGravityMode
        }
        break;
    }
    case MSP_SET_SENSOR_CONFIG:
        src.readU8(); // acc
        src.readU8(); // baro
        src.readU8(); // map
        break;
    case MSP_SET_FEATURE_CONFIG:
        _cockpit.setFeatures(src.readU32());
        break;
    case MSP_SET_BEEPER_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
        //rollDegrees = src.readU16();
        //pitchDegrees = src.readU16();
        //yawDegrees = src.readU16();
        break;
    case MSP_SET_MIXER_CONFIG:
        src.readU8(); // mixer mode, eg QUAD, etc
        src.readU8(); // yaw_motors_reversed
        break;
    case MSP_SET_RX_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_FAILSAFE_CONFIG: {
        Cockpit::failsafe_config_t failsafeConfig{};
        failsafeConfig.delay_deciseconds = src.readU8();
        failsafeConfig.landing_time_seconds = src.readU8();
        failsafeConfig.throttle_pwm = src.readU16();
        failsafeConfig.switch_mode = src.readU8();
        failsafeConfig.throttle_low_delay_deciseconds = src.readU16();
        failsafeConfig.procedure = src.readU8();
        _cockpit.setFailsafeConfig(failsafeConfig);
        break;
    }
    case MSP_SET_RXFAIL_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_RSSI_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_RX_MAP:
        return RESULT_ERROR;
    case MSP_SET_CF_SERIAL_CONFIG:
        return RESULT_ERROR;
    case MSP2_COMMON_SET_SERIAL_CONFIG:
        return RESULT_ERROR;
    case MSP_SET_ACC_TRIM:
        return RESULT_ERROR;
    case MSP_ACC_CALIBRATION:
        return RESULT_ERROR;
    case MSP_MAG_CALIBRATION:
        return RESULT_ERROR;
    case MSP_EEPROM_WRITE:
        if (_flightController.motorsIsOn()) {
            // can't save to non volatile storage if the motors are on
            return RESULT_ERROR;
        }
        _nonVolatileStorage.storeAll(_imuFilters, _flightController, _cockpit, _autopilot, _pidProfileIndex, _ratesProfileIndex);
        break;

    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return RESULT_ERROR;
    }
    return RESULT_ACK;
}
