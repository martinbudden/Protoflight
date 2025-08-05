#include "MSP_ProtoFlight.h"

#include <AHRS.h>
#include <Features.h>
#include <FlightController.h>
#include <IMU_Filters.h>
#include <MSP_Protocol.h>
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
        for (size_t ii = 0; ii < FlightController::PID_COUNT; ++ii) {
            const auto pidIndex = static_cast<FlightController::pid_index_e>(ii);
            _flightController.setPID_P_MSP(pidIndex, src.readU8());
            _flightController.setPID_I_MSP(pidIndex, src.readU8());
            _flightController.setPID_D_MSP(pidIndex, src.readU8());
        }
        //pidInitConfig(currentPidProfile);
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
            rates.ratesType = src.readU8();
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
        IMU_Filters::filters_config_t imuFilters {};
        FlightController::filters_config_t fcFilters {};

        imuFilters.gyro_lpf1_hz = src.readU8();
        fcFilters.dterm_lpf1_hz = src.readU16();
        fcFilters.yaw_lpf_hz = src.readU16();
        if (src.bytesRemaining() >= 8) {
            imuFilters.gyro_notch1_hz = src.readU16();
            imuFilters.gyro_notch1_cutoff = src.readU16();
            fcFilters.dterm_notch_hz = src.readU16();
            fcFilters.dterm_notch_cutoff = src.readU16();
        }
        if (src.bytesRemaining() >= 4) {
            imuFilters.gyro_notch2_hz = src.readU16();
            imuFilters.gyro_notch2_cutoff = src.readU16();
        }
        if (src.bytesRemaining() >= 1) {
            fcFilters.dterm_lpf1_type = src.readU8();
        }
        if (src.bytesRemaining() >= 10) {
            imuFilters.gyro_hardware_lpf = src.readU8();
            src.readU8(); // was gyro_32khz_hardware_lpf
            imuFilters.gyro_lpf1_hz = src.readU16();
            imuFilters.gyro_lpf2_hz = src.readU16();
            imuFilters.gyro_lpf1_type = src.readU8();
            imuFilters.gyro_lpf2_type = src.readU8();
            fcFilters.dterm_lpf2_hz = src.readU16();
        }

        if (src.bytesRemaining() >= 9) {
            // Added in MSP API 1.41
            fcFilters.dterm_lpf2_type = src.readU8();
            imuFilters.gyro_dynamic_lpf1_min_hz = src.readU16();
            imuFilters.gyro_dynamic_lpf1_max_hz = src.readU16();
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
            imuFilters.rpm_filter_harmonics = src.readU8();
            imuFilters.rpm_filter_min_hz = src.readU8();
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
        static_cast<IMU_Filters&>(imuFiltersObject).setFiltersConfig(imuFilters); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
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
