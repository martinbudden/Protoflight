#include "MSP_ProtoFlight.h"

#include <AHRS.h>
#include <Features.h>
#include <FlightController.h>
#include <IMU_Filters.h>
#include <MSP_Protocol.h>
#include <NonVolatileStorage.h>
#include <RadioController.h>
#include <ReceiverBase.h>


MSP_Base::result_e MSP_ProtoFlight::processInCommand(int16_t cmdMSP, StreamBuf& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) // NOLINT(readability-function-cognitive-complexity)
{
    (void)srcDesc;
    (void)postProcessFn;
    // const size_t dataSize = src.bytesRemaining();

    switch (cmdMSP) {
    case MSP_SET_FEATURE_CONFIG:
        _features.setFeatures(src.readU32());
        break;

    case MSP_SET_FAILSAFE_CONFIG: {
        RadioController::failsafe_t failsafe{};
        failsafe.delay = src.readU8();
        failsafe.landing_time = src.readU8();
        failsafe.throttle = src.readU16();
        failsafe.switch_mode = src.readU8();
        failsafe.throttle_low_delay = src.readU16();
        failsafe.procedure = src.readU8();
        _radioController.setFailsafe(failsafe);
        break;
    }
    case MSP_SET_MIXER_CONFIG:
        src.readU8(); // mixer mode, eg QUAD, etc
        src.readU8(); // yaw_motors_reversed
        break;
    case MSP_SELECT_SETTING:
        break;

    case MSP_COPY_PROFILE:
        break;

    case MSP_SET_RAW_RC:
        break;

    case MSP_SET_ACC_TRIM:
        break;

    case MSP_SET_ARMING_CONFIG:
        break;

    case MSP_SET_PID_CONTROLLER:
        break;

    case MSP_SET_PID:
        for (size_t ii = 0; ii <= FlightController::YAW_RATE_DPS; ++ii) {
            const auto pidIndex = static_cast<FlightController::pid_index_e>(ii);
            _flightController.setPID_P_MSP(pidIndex, src.readU8());
            _flightController.setPID_I_MSP(pidIndex, src.readU8());
            _flightController.setPID_D_MSP(pidIndex, src.readU8());
        }
        // skip over PID_LEVEL and PID_MAG
        for (size_t ii = 0; ii < 2; ++ii) {
            src.readU8();
            src.readU8();
            src.readU8();
        }
        break;

    case MSP_SET_PID_ADVANCED:
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
            // PID controller feedforward terms
            _flightController.setPID_F_MSP(FlightController::ROLL_RATE_DPS, src.readU16());
            _flightController.setPID_F_MSP(FlightController::PITCH_RATE_DPS, src.readU16());
            _flightController.setPID_F_MSP(FlightController::YAW_RATE_DPS, src.readU16());
            src.readU8(); // was currentPidProfile->antiGravityMode
        }
        break;

    case MSP_SET_MODE_RANGE:
        break;

    case MSP_SET_ADJUSTMENT_RANGE:
        break;

    case MSP_SET_RC_TUNING: {
        if (src.bytesRemaining() < 10) {
            return RESULT_ERROR;
        }
        RadioController::rates_t rates = _radioController.getRates();
        uint8_t value = src.readU8();
        if (rates.rcRates[RadioController::PITCH] == rates.rcRates[RadioController::ROLL]) {
            rates.rcRates[RadioController::PITCH] = value;
        }
        rates.rcRates[RadioController::ROLL] = value;

        value = src.readU8();
        if (rates.rcExpos[RadioController::PITCH] == rates.rcExpos[RadioController::ROLL]) {
            rates.rcExpos[RadioController::PITCH] = value;
        }
        rates.rcExpos[RadioController::ROLL] = value;

        rates.rcRates[RadioController::ROLL] = src.readU8();
        rates.rcRates[RadioController::PITCH] = src.readU8();
        rates.rcRates[RadioController::YAW] = src.readU8();
        src.readU8(); // skip tpa_rate
        rates.throttleMidpoint = src.readU8();
        rates.throttleExpo = src.readU8();
        src.readU16(); // skip tpa_breakpoint

        if (src.bytesRemaining() >= 1) {
            rates.rcExpos[RadioController::YAW] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcRates[RadioController::YAW] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcRates[RadioController::PITCH] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcExpos[RadioController::PITCH] = src.readU8();
        }
        // version 1.41
        if (src.bytesRemaining() >= 2) {
            rates.throttleLimitType = src.readU8();
            rates.throttleLimitPercent = src.readU8();
        }
        // version 1.42
        if (src.bytesRemaining() >= 6) {
            rates.rateLimits[RadioController::ROLL] = src.readU16();
            rates.rateLimits[RadioController::PITCH] = src.readU16();
            rates.rateLimits[RadioController::YAW] = src.readU16();
        }
        // version 1.43
        if (src.bytesRemaining() >= 1) {
            src.readU8(); // hardcoded to RATES_TYPE_ACTUAL
        }
        _radioController.setRates(rates);
        break;
    }

    case MSP_SET_MOTOR_CONFIG:
        break;

    case MSP_SET_MOTOR:
        //for (int i = 0; i < getMotorCount(); i++) {
        //    motor_disarmed[i] = motorConvertFromExternal(src.readU16());
        //}
        break;

    case MSP_SET_RESET_CURR_PID:
        //resetPidProfile(currentPidProfile);
        break;

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

    case MSP_SET_FILTER_CONFIG: {
        IMU_Filters::config_t imuFiltersConfig {};
        FlightController::filters_config_t fcFilters {};

        imuFiltersConfig.gyro_lpf1_hz = src.readU8();
        fcFilters.dterm_lpf1_hz = src.readU16();
        fcFilters.yaw_lpf_hz = src.readU16();
        if (src.bytesRemaining() >= 8) {
            imuFiltersConfig.gyro_notch1_hz = src.readU16();
            imuFiltersConfig.gyro_notch1_cutoff = src.readU16();
            fcFilters.dterm_notch_hz = src.readU16();
            fcFilters.dterm_notch_cutoff = src.readU16();
        }
        if (src.bytesRemaining() >= 4) {
            imuFiltersConfig.gyro_notch2_hz = src.readU16();
            imuFiltersConfig.gyro_notch2_cutoff = src.readU16();
        }
        if (src.bytesRemaining() >= 1) {
            fcFilters.dterm_lpf1_type = src.readU8();
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
            fcFilters.dterm_lpf2_type = src.readU8();
            imuFiltersConfig.gyro_dynamic_lpf1_min_hz = src.readU16();
            imuFiltersConfig.gyro_dynamic_lpf1_max_hz = src.readU16();
            fcFilters.dterm_dynamic_lpf1_min_hz = src.readU16();
            fcFilters.dterm_dynamic_lpf1_max_hz = src.readU16();
        }

        if (src.bytesRemaining() >= 8) {
            // Added in MSP API 1.42
            src.readU8(); // DEPRECATED 1.43: dyn_notch_range
            src.readU8(); // DEPRECATED 1.44: dyn_notch_width_percent
            //dynamic_notch_q =
            src.readU16();
            //dynamic_notch_min_hz =
            src.readU16();
            imuFiltersConfig.rpm_filter_harmonics = src.readU8();
            imuFiltersConfig.rpm_filter_min_hz = src.readU8();
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
        IMU_FiltersBase& imuFiltersObject = _ahrs.getIMU_Filters();
        static_cast<IMU_Filters&>(imuFiltersObject).setConfig(imuFiltersConfig); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
        _flightController.setFiltersConfig(fcFilters);
        break;
    }

    case MSP_SET_SENSOR_CONFIG:
        src.readU8(); // acc
        src.readU8(); // baro
        src.readU8(); // map
        break;

    case MSP_ACC_CALIBRATION:
        break;

#if defined(USE_MAG)
    case MSP_MAG_CALIBRATION:
        break;
#endif
    case MSP_EEPROM_WRITE:
        if (_flightController.motorsIsOn()) {
            // can't save to non volatile storage if the motors are on
            return RESULT_ERROR;
        }
        _nonVolatileStorage.storeAll(_ahrs, _flightController, _radioController, _receiver);
        break;
    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
        //rollDegrees = src.readU16();
        //pitchDegrees = src.readU16();
        //yawDegrees = src.readU16();
        break;

    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return RESULT_ERROR;
    }
    return RESULT_ACK;
}
