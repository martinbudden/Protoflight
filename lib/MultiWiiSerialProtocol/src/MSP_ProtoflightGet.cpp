#include "Autopilot.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"
#include "Rates.h"
#include "VTX.h"
#include "version.h"

#include <GPS.h>
#include <GPS_MessageQueue.h>
#include <IMU_Base.h>
#include <MSP_Protocol.h>

#include <ahrs_message_queue.h>
#include <blackbox.h>
#include <debug.h>
#include <motor_mixer_base.h>
#include <receiver_base.h>
#include <rpm_filters.h>

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


//enum  gyroHardware_e { GYRO_NONE = 0, GYRO_DEFAULT = 1, GYRO_VIRTUAL = 20 };
//enum  accelerationSensor_e { ACC_DEFAULT = 0, ACC_NONE = 1, ACC_VIRTUAL = 21 };

enum { SIGNATURE_LENGTH = 32 };




/*!
Returns true if the command was processed, false otherwise.
May set mspPostProcessFunc to a function to be called once the command has been processed
*/
msp_result_e MSP_Protoflight::process_get_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufWriter& dst)
{
    switch (cmd_msp) {
    case MSP_API_VERSION:
        dst.write_u8(MSP_PROTOCOL_VERSION);
        dst.write_u8(MSP_API_VERSION_MAJOR);
        dst.write_u8(MSP_API_VERSION_MINOR);
        break;
    case MSP_STATUS_EX:
        [[fallthrough]];
    case MSP_STATUS: {
        dst.write_u16(static_cast<uint16_t>(pg.flightController.get_task_interval_microseconds()));
        dst.write_u16(0); // I2C error counter
        static constexpr uint16_t SENSOR_ACCELEROMETER = 0x01;
        static constexpr uint16_t SENSOR_GYROSCOPE = 0x01U << 5U;
        dst.write_u16(SENSOR_ACCELEROMETER | SENSOR_GYROSCOPE);
        MspBox::bitset_t flightModeFlags;
        const size_t flagBitCount = pg.cockpit.packFlightModeFlags(flightModeFlags, pg.rc_modes);
        dst.write_data(&flightModeFlags, 4); // unconditional part of flags, first 32 bits
        dst.write_u8(pg.nonVolatileStorage.get_current_pid_profile_index());
        dst.write_u16(10); //constrain(getAverageSystemLoadPercent(), 0, LOAD_PERCENTAGE_ONE))
        if (cmd_msp == MSP_STATUS_EX) {
            dst.write_u8(NonVolatileStorage::PID_PROFILE_COUNT);
            dst.write_u8(pg.nonVolatileStorage.get_current_rate_profile_index());
        } else { // MSP_STATUS
            dst.write_u16(0); // gyro cycle time
        }

        // write flightModeFlags header. Lowest 4 bits contain number of bytes that follow
        // header is emmitted even when all bits fit into 32 bits to allow future extension
        size_t byteCount = (flagBitCount - 32 + 7) / 8;        // 32 already stored, round up
        byteCount = static_cast<uint8_t>(byteCount);
        if (byteCount > 15) {
            byteCount = 15; // limit to 16 bytes (128 bits)
        }
        dst.write_u8(static_cast<uint8_t>(byteCount));
        dst.write_data(reinterpret_cast<uint8_t*>(&flightModeFlags) + 4, byteCount); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast)

        // Write arming disable flags
        // 1 byte, flag count
        dst.write_u8(ARMING_DISABLE_FLAGS_COUNT);
        // 4 bytes, flags
        const uint32_t armingDisableFlags = pg.cockpit.getArmingDisableFlags();
        dst.write_u32(armingDisableFlags);

        // config state flags - bits to indicate the state of the configuration, reboot required, etc.
        // other flags can be added as needed
        const bool rebootRequired = false;
        dst.write_u8(rebootRequired);

        dst.write_u16(0); // CPU temperature, added in API v1.46
        break;
    }
    case MSP_RAW_IMU: {
        ImuBase::xyz_int32_t acc {};
        pg.ahrs.read_acc_raw(acc.x, acc.y, acc.z);
        dst.write_u16(static_cast<uint16_t>(acc.x));
        dst.write_u16(static_cast<uint16_t>(acc.y));
        dst.write_u16(static_cast<uint16_t>(acc.z));
        ImuBase::xyz_int32_t gyro {};
        pg.ahrs.read_gyro_raw(gyro.x, gyro.y, gyro.z);
        dst.write_u16(static_cast<uint16_t>(gyro.x));
        dst.write_u16(static_cast<uint16_t>(gyro.y));
        dst.write_u16(static_cast<uint16_t>(gyro.z));
        // write zeros for magnetometer
        ImuBase::xyz_int32_t mag {};
        //pg.ahrs.readMagRaw(mag.x, mag.y, mag.z);
        dst.write_u16(static_cast<uint16_t>(mag.x));
        dst.write_u16(static_cast<uint16_t>(mag.y));
        dst.write_u16(static_cast<uint16_t>(mag.z));
        break;
    }
    case MSP_NAME:
        dst.write_string_with_zero_terminator("Martin Budden"); // pilot/aircraft name
        break;
    case MSP_MOTOR:
        // Range-based for over the view
        // std::ranges::for_each( std::views::iota(size_t{0}, size_t{8}), [&](size_t) { dst.write_u16(0); } );
#if (__cplusplus >= 202002L)
        for ([[maybe_unused]] auto _ : std::views::iota(size_t{0}, size_t{8})) {
#else
        for (size_t ii = 0; ii < 8; ++ii) {
#endif
            dst.write_u16(0);
        }
        break;
    case MSP_MOTOR_TELEMETRY:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP2_MOTOR_OUTPUT_REORDERING:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP2_GET_VTX_DEVICE_STATUS:
        serializeVTX(pg, dst);
        break;
    case MSP_RC: {
        const receiver_controls_pwm_t controls = pg.receiver.get_controls_pwm();
        dst.write_u16(controls.throttle);
        dst.write_u16(controls.roll);
        dst.write_u16(controls.pitch);
        dst.write_u16(controls.yaw);
#if (__cplusplus >= 202002L)
        for (auto ii : std::views::iota(size_t{0}, size_t{pg.receiver.get_auxiliary_channel_count()})) {
#else
        for (size_t ii = 0; ii < pg.receiver.get_auxiliary_channel_count(); ++ii) {
#endif
            dst.write_u16(pg.receiver.get_auxiliary_channel(ii));
        }
        break;
    }
    case MSP_ATTITUDE: {
        ahrs_data_t ahrsData;
        pg.ahrsMessageQueue.PEEK_AHRS_DATA(ahrsData);
        dst.write_u16(static_cast<uint16_t>(ahrsData.orientation.calculate_roll_degrees()*10.0F));
        dst.write_u16(static_cast<uint16_t>(ahrsData.orientation.calculate_pitch_degrees()*10.0F));
        dst.write_u16(static_cast<uint16_t>(ahrsData.orientation.calculate_yaw_degrees()));
        break;
    }
    case MSP_ALTITUDE:
        dst.write_u32(0);
        dst.write_u16(0);
        break;
    case MSP_SONAR_ALTITUDE:
        dst.write_u32(0);
        break;
    case MSP_BOARD_ALIGNMENT_CONFIG:
        //dst.write_u16(boardAlignment()->rollDegrees);
        //dst.write_u16(boardAlignment()->pitchDegrees);
        //dst.write_u16(boardAlignment()->yawDegrees);
        dst.write_u16(0);
        dst.write_u16(0);
        dst.write_u16(0);
        break;
    case MSP_ARMING_CONFIG:
        dst.write_u8(0); //armingConfig()->auto_disarm_delay);
        dst.write_u8(0);
        dst.write_u8(0); //imuConfig()->small_angle);
        dst.write_u8(0); //armingConfig()->gyro_cal_on_first_arm);
        break;
    case MSP_RC_TUNING: {
        const rates_t rates = pg.cockpit.getRates();
        dst.write_u8(static_cast<uint8_t>(rates.rcRates[rates_t::ROLL]));
        dst.write_u8(static_cast<uint8_t>(rates.rcExpos[rates_t::ROLL]));
        dst.write_u8(static_cast<uint8_t>(rates.rates[rates_t::ROLL]));
        dst.write_u8(static_cast<uint8_t>(rates.rates[rates_t::PITCH]));
        dst.write_u8(static_cast<uint8_t>(rates.rates[rates_t::YAW]));
        dst.write_u8(0); // was tpa_rate
        dst.write_u8(rates.throttleMidpoint);
        dst.write_u8(rates.throttleExpo);
        dst.write_u16(0); // was tpa_breakpoint
        dst.write_u8(static_cast<uint8_t>(rates.rcExpos[rates_t::YAW]));
        dst.write_u8(static_cast<uint8_t>(rates.rcRates[rates_t::YAW]));
        dst.write_u8(static_cast<uint8_t>(rates.rcRates[rates_t::PITCH]));
        dst.write_u8(static_cast<uint8_t>(rates.rcExpos[rates_t::PITCH]));
        // added in 1.41
        dst.write_u8(rates.throttleLimitType);
        dst.write_u8(rates.throttleLimitPercent);
        // added in 1.42
        dst.write_u8(static_cast<uint8_t>(rates.rateLimits[rates_t::ROLL]));
        dst.write_u8(static_cast<uint8_t>(rates.rateLimits[rates_t::PITCH]));
        dst.write_u8(static_cast<uint8_t>(rates.rateLimits[rates_t::YAW]));
        // added in 1.43
        dst.write_u8(rates_t::TYPE_ACTUAL); // hardcoded, since we only support RATES_TYPE_ACTUAL rates.ratesType);
        break;
    }
    case MSP_PID: {
#if (__cplusplus >= 202002L)
        for (auto ii : std::views::iota(size_t{0}, size_t{FlightController::PID_COUNT})) {
#else
        for (size_t ii = 0; ii < FlightController::PID_COUNT; ++ii) {
#endif
            const auto pid_index = static_cast<FlightController::pid_index_e>(ii);
            const FlightController::PIDF_uint16_t pid = pg.flightController.get_pid_msp(pid_index);
            dst.write_u8(static_cast<uint8_t>(pid.kp));
            dst.write_u8(static_cast<uint8_t>(pid.ki));
            dst.write_u8(static_cast<uint8_t>(pid.kd));
        }
        const FlightController::PIDF_uint16_t pid = pg.flightController.get_pid_msp(FlightController::ROLL_ANGLE_DEGREES);
        dst.write_u8(static_cast<uint8_t>(pid.kp));
        dst.write_u8(static_cast<uint8_t>(pid.ki));
        dst.write_u8(static_cast<uint8_t>(pid.kd));
        break;
    }
    case MSP_PIDNAMES:
#if (__cplusplus >= 202002L)
        for (auto ii : std::views::iota(size_t{0}, size_t{FlightController::PID_COUNT})) {
#else
        for (size_t ii = 0; ii < FlightController::PID_COUNT; ++ii) {
#endif
            const std::string& pidName = pg.flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
            dst.write_string_with_zero_terminator(pidName);
        }
        break;
    case MSP_PID_CONTROLLER: {
        enum { PID_CONTROLLER_BETAFLIGHT = 1 };
        dst.write_u8(PID_CONTROLLER_BETAFLIGHT);
        break;
    }
    case MSP_MODE_RANGES:
        for (const auto& mac : pg.rc_modes.get_mode_activation_conditions()) {
            const MspBox::box_t* box = MspBox::findBoxByBoxId(mac.mode_id);
            if (box == nullptr) {
                return MSP_RESULT_CMD_UNKNOWN;
            }
            dst.write_u8(box->permanent_id);
            dst.write_u8(mac.auxiliary_channel_index);
            dst.write_u8(mac.range_start);
            dst.write_u8(mac.range_end);
        }
        break;
    case MSP_MODE_RANGES_EXTRA:
        dst.write_u8(RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT);
        for (const auto& mac : pg.rc_modes.get_mode_activation_conditions()) {
            const MspBox::box_t* box = MspBox::findBoxByBoxId(mac.mode_id);
            const MspBox::box_t* linkedBox = MspBox::findBoxByBoxId(mac.linked_to);
            if (box == nullptr || linkedBox == nullptr) {
                return MSP_RESULT_CMD_UNKNOWN;
            }
            dst.write_u8(box->permanent_id); // each element is aligned with MODE_RANGES by the permanent_id
            dst.write_u8(mac.mode_logic);
            dst.write_u8(linkedBox->permanent_id);
        }
        break;
    case MSP_ADJUSTMENT_RANGES:
#if defined(USE_RC_ADJUSTMENTS)
        for (const auto& adjustmentRange : pg.cockpit.getRC_Adjustments().getAdjustmentRanges()) {
            dst.write_u8(0); // was adjustmentRange.adjustmentIndex
            dst.write_u8(adjustmentRange.aux_channel_index);
            dst.write_u8(adjustmentRange.range_start);
            dst.write_u8(adjustmentRange.range_end);
            dst.write_u8(adjustmentRange.adjustmentConfig);
            dst.write_u8(adjustmentRange.auxSwitchChannelIndex);
        }
        break;
#else
        return MSP_RESULT_CMD_UNKNOWN;
#endif
    case MSP_MOTOR_CONFIG:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_COMPASS_CONFIG:
        return MSP_RESULT_CMD_UNKNOWN;
#if defined(USE_GPS)
    case MSP_GPS_CONFIG: {
        if (pg.gps == nullptr) {
            return MSP_RESULT_CMD_UNKNOWN;
        }
        const gps_config_t& gpsConfig = pg.gps->getConfig();
        dst.write_u8(gpsConfig.provider);
        dst.write_u8(gpsConfig.sbasMode);
        dst.write_u8(gpsConfig.autoConfig);
        dst.write_u8(gpsConfig.autoBaud);
        // Added in API version 1.43
        dst.write_u8(gpsConfig.gps_set_home_point_once);
        dst.write_u8(gpsConfig.gps_ublox_use_galileo);
        break;
    }
    case MSP_RAW_GPS: {
        if (pg.gps == nullptr) {
            return MSP_RESULT_CMD_UNKNOWN;
        }
        gps_message_data_t gpsMessageData {};
        pg.gps->getGPS_MessageQueue().PEEK_GPS_DATA(gpsMessageData);

        dst.write_u8(gpsMessageData.fix);
        dst.write_u8(gpsMessageData.satellite_count);
        dst.write_u32(static_cast<uint32_t>(gpsMessageData.latitude_degrees1E7));
        dst.write_u32(static_cast<uint32_t>(gpsMessageData.longitude_degrees1E7));
        // altitude changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. To maintain backwards compatibility compensate to 1m per lsb in MSP again.
        dst.write_u16( std::clamp(static_cast<uint16_t>(gpsMessageData.altitude_cm / 100), uint16_t{0}, uint16_t{UINT16_MAX}) ); 
        dst.write_u16(static_cast<uint16_t>(gpsMessageData.ground_speed_cmps));
        dst.write_u16(static_cast<uint16_t>(gpsMessageData.heading_deci_degrees));
        // Added in API version 1.44
        dst.write_u16(gpsMessageData.dilution_of_precision_positional);
        break;
    }
    case MSP_COMP_GPS: {
        if (pg.gps == nullptr) {
            return MSP_RESULT_CMD_UNKNOWN;
        }
        gps_message_data_t gpsMessageData {};
        pg.gps->getGPS_MessageQueue().PEEK_GPS_DATA(gpsMessageData);

        dst.write_u16(static_cast<uint16_t>(gpsMessageData.distance_to_home_meters));
        dst.write_u16(static_cast<uint16_t>(gpsMessageData.distance_to_home_meters / 10.0F)); // resolution increased in Betaflight 4.4 by factor of 10, this maintains backwards compatibility for DJI OSD
        dst.write_u8(gpsMessageData.update & 0x01U);
        break;
    }
    case MSP_GPSSVINFO:
        return MSP_RESULT_CMD_UNKNOWN;
#if defined(USE_GPS_RESCUE)
    case MSP_GPS_RESCUE: {
        const gps_rescue_config_t& gpsRescueConfig = pg.cockpit.getAutopilot().getGPS_RescueConfig();
        dst.write_u16(gpsRescueConfig.maxRescueAngle_degrees);
        dst.write_u16(gpsRescueConfig.returnAltitude_meters);
        dst.write_u16(gpsRescueConfig.descentDistance_meters);
        dst.write_u16(gpsRescueConfig.ground_speed_cmps);
        const autopilot_config_t& autopilotConfig = pg.cockpit.getAutopilot().get_autopilot_config();
        dst.write_u16(autopilotConfig.throttle_min_pwm);
        dst.write_u16(autopilotConfig.throttle_max_pwm);
        dst.write_u16(autopilotConfig.throttle_hover_pwm);
        dst.write_u8( gpsRescueConfig.sanityChecks);
        dst.write_u8( gpsRescueConfig.minSats);

        // Added in API version 1.43
        dst.write_u16(gpsRescueConfig.ascendRate);
        dst.write_u16(gpsRescueConfig.descendRate);
        dst.write_u8(gpsRescueConfig.allowArmingWithoutFix);
        dst.write_u8(gpsRescueConfig.altitudeMode);
        // Added in API version 1.44
        dst.write_u16(gpsRescueConfig.minStartDist_meters);
        // Added in API version 1.46
        dst.write_u16(gpsRescueConfig.initialClimb_meters);
        break;
    }
    case MSP_GPS_RESCUE_PIDS: {
        const autopilot_config_t& autopilotConfig = pg.cockpit.getAutopilot().get_autopilot_config();
        dst.write_u16(autopilotConfig.altitudePID.kp);
        dst.write_u16(autopilotConfig.altitudePID.ki);
        dst.write_u16(autopilotConfig.altitudePID.kd);
        // altitude_F not implemented yet
        const gps_rescue_config_t& gpsRescueConfig = pg.cockpit.getAutopilot().getGPS_RescueConfig();
        dst.write_u16(gpsRescueConfig.velP);
        dst.write_u16(gpsRescueConfig.velI);
        dst.write_u16(gpsRescueConfig.velD);
        dst.write_u16(gpsRescueConfig.yawP);
        break;
    }
#endif // USE_GPS_RESCUE
#endif // USE_GPS
    case MSP_ACC_TRIM:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_MIXER_CONFIG: {
        const mixer_config_t& mixerConfig = pg.motorMixer.get_mixer_config();
        dst.write_u8(static_cast<uint8_t>(mixerConfig.type));
        dst.write_u8(mixerConfig.yaw_motors_reversed);
        break;
    }
    case MSP_RX_CONFIG: {
        const rx_config_t& rxConfig = pg.cockpit.getRX_Config();
        dst.write_u8(rxConfig.serial_rx_provider);
        dst.write_u16(rxConfig.max_check);
        dst.write_u16(rxConfig.mid_rc);
        dst.write_u16(rxConfig.min_check);
        dst.write_u8(rxConfig.spektrum_sat_bind);
        dst.write_u16(rxConfig.rx_min_usec);
        dst.write_u16(rxConfig.rx_max_usec);
        dst.write_u8(0); // not required in API 1.44, was rxConfig.rcInterpolation
        dst.write_u8(0); // not required in API 1.44, was rxConfig.rcInterpolationInterval
        dst.write_u16(static_cast<uint16_t>(rxConfig.airModeActivateThreshold * 10 + 1000));
        dst.write_u8(0);
        dst.write_u32(0);
        dst.write_u8(0);
        dst.write_u8(rxConfig.fpvCamAngleDegrees);
        dst.write_u8(0); // not required in API 1.44, was rxConfig.rcSmoothingChannels
        dst.write_u8(0);
        dst.write_u8(0);
        dst.write_u8(0);
        dst.write_u8(0);
        dst.write_u8(0);
        dst.write_u8(0);
        break;
    }
    case MSP_FAILSAFE_CONFIG: {
        const failsafe_config_t failsafeConfig = pg.cockpit.getFailsafeConfig();
        dst.write_u8(failsafeConfig.delay_deciseconds);
        dst.write_u8(failsafeConfig.landing_time_seconds);
        dst.write_u16(failsafeConfig.throttle_pwm);
        dst.write_u8(failsafeConfig.switch_mode);
        dst.write_u16(failsafeConfig.throttle_low_delay_deciseconds);
        dst.write_u8(failsafeConfig.procedure);
        break;
    }
    case MSP_RXFAIL_CONFIG: {
#if false
        const RX::failsafe_channel_configs_t& rxFailsafeChannelConfigs = pg.cockpit.getRX_FailsafeChannelConfigs();
        for (size_t ii = 0; ii < rxRuntimeState.channelCount; ++ii) {
            dst.write_u8(rxFailsafeChannelConfigs[ii].mode);
            dst.write_u16(RX::failStepToChannelValue(rxFailsafeChannelConfigs[ii].step));
        }
        break;
#endif
        return MSP_RESULT_CMD_UNKNOWN;
    }
    case MSP_RSSI_CONFIG: {
        const rx_config_t& rxConfig = pg.cockpit.getRX_Config();
        dst.write_u8(rxConfig.rssi_channel);
        break;
    }
    case MSP_RX_MAP:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_CF_SERIAL_CONFIG:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP2_COMMON_SERIAL_CONFIG:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(pg, dst);
        break;
    case MSP_BLACKBOX_CONFIG: {
#if defined(USE_BLACKBOX)
        if (pg.blackbox == nullptr) {
            return MSP_RESULT_CMD_UNKNOWN;
        }
        const Blackbox::config_t& blackboxConfig = pg.blackbox->getConfig();
        dst.write_u8(1); //Blackbox supported
        dst.write_u8(blackboxConfig.device);
        dst.write_u8(1); // Rate numerator, not used anymore
        dst.write_u8(static_cast<uint8_t>(pg.blackbox->getPInterval()));
        dst.write_u16(static_cast<uint16_t>(pg.blackbox->getIInterval() / pg.blackbox->getPInterval()));
        dst.write_u8(blackboxConfig.sample_rate);
        // Added in MSP API 1.45
        dst.write_u32(blackboxConfig.fieldsDisabledMask);
#else
        dst.write_u8(0); // Blackbox not supported
        dst.write_u8(0);
        dst.write_u8(0);
        dst.write_u8(0);
        dst.write_u16(0);
        dst.write_u8(0);
        // Added in MSP API 1.45
        dst.write_u32(0);
#endif
        break;
    }
    case MSP_SDCARD_SUMMARY:
        serializeSDCardSummaryReply(pg, dst);
        break;
    case MSP_MOTOR_3D_CONFIG:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_RC_DEADBAND:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_SENSOR_ALIGNMENT: {
        const uint8_t gyroAlignment = 0; //gyroDeviceConfig(0)->alignment;
        dst.write_u8(gyroAlignment);
        dst.write_u8(gyroAlignment);  // Starting with 4.0 gyro and acc alignment are the same
        dst.write_u8(0); // mag alignment

        // API 1.41 - Add multi-gyro indicator, selected gyro, and support for separate gyro 1 & 2 alignment
        dst.write_u8(0); // getGyroDetectionFlags()
        enum { GYRO_CONFIG_USE_GYRO_1 = 0 };
        dst.write_u8(GYRO_CONFIG_USE_GYRO_1);
        dst.write_u8(0); // alignment
        enum { ALIGN_DEFAULT = 0 };
        dst.write_u8(ALIGN_DEFAULT);
        break;
    }
    case MSP_ADVANCED_CONFIG: {
        const motor_config_t& motorConfig = pg.motorMixer.get_motor_config();

        dst.write_u8(1); // was gyro_sync_denom - removed in API 1.43
        dst.write_u8(1); // pid_process_denom
        dst.write_u8(motorConfig.device.use_continuous_update);
        dst.write_u8(motorConfig.device.motor_protocol);
        dst.write_u16(motorConfig.device.motor_pwm_rate);
        dst.write_u16(motorConfig.motor_idle);
        dst.write_u8(0); // was gyro_use_32kHz
        dst.write_u8(motorConfig.device.motor_inversion);
        dst.write_u8(0); // deprecated gyro_to_use
        dst.write_u8(0); // gyro_high_fsr
        dst.write_u8(0); // gyroMovementCalibrationThreshold
        dst.write_u16(0); // gyroCalibrationDuration
        dst.write_u16(0); // gyro_offset_yaw
        dst.write_u8(0); // checkOverflow
        //Added in MSP API 1.42
        dst.write_u8(pg.debug.getMode());
        dst.write_u8(DEBUG_COUNT);
        break;
    }
    case MSP_FILTER_CONFIG : {
        const imu_filters_config_t imuFiltersConfig = pg.imuFilters.getConfig();
        const flight_controller_filters_config_t fcFilters = pg.flightController.getFiltersConfig();
        const RpmFilters* rpmFilters = pg.imuFilters.getRPM_Filters();
        const rpm_filters_config_t rpmFiltersConfig = rpmFilters ? rpmFilters->get_config() : rpm_filters_config_t {}; // cppcheck-suppress knownConditionTrueFalse

        dst.write_u8(static_cast<uint8_t>(imuFiltersConfig.gyro_lpf1_hz));
        dst.write_u16(fcFilters.dterm_lpf1_hz);
        dst.write_u16(fcFilters.yaw_lpf_hz);
        dst.write_u8(static_cast<uint8_t>(imuFiltersConfig.gyro_lpf1_hz));
        dst.write_u16(fcFilters.dterm_lpf1_hz);
        dst.write_u16(fcFilters.yaw_lpf_hz);
        dst.write_u16(imuFiltersConfig.gyro_notch1_hz);
        dst.write_u16(imuFiltersConfig.gyro_notch1_cutoff);
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.write_u16(fcFilters.dterm_notch_hz);
        dst.write_u16(fcFilters.dterm_notch_cutoff);
#else
        dst.write_u16(0);
        dst.write_u16(0);
#endif
        dst.write_u16(imuFiltersConfig.gyro_notch2_hz);
        dst.write_u16(imuFiltersConfig.gyro_notch2_cutoff);
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.write_u8(fcFilters.dterm_lpf1_type);
#else
        dst.write_u8(0);
#endif
        dst.write_u8(0); // gyro_hardware_lpf set in driver
        dst.write_u8(0); // was gyro_32khz_hardware_lpf
        dst.write_u16(imuFiltersConfig.gyro_lpf1_hz);
        dst.write_u16(imuFiltersConfig.gyro_lpf2_hz);
        dst.write_u8(imuFiltersConfig.gyro_lpf1_type);
        dst.write_u8(imuFiltersConfig.gyro_lpf2_type);
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.write_u16(fcFilters.dterm_lpf2_hz);
#else
        dst.write_u16(0);
#endif
        // Added in MSP API 1.41
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.write_u8(fcFilters.dterm_lpf2_type);
#else
        dst.write_u8(0);
#endif
        dst.write_u16(0); // imuFiltersConfig.gyro_dynamic_lpf1_min_hz);
        dst.write_u16(0); //imuFiltersConfig.gyro_dynamic_lpf1_max_hz);
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.write_u16(fcFilters.dterm_dynamic_lpf1_min_hz);
        dst.write_u16(fcFilters.dterm_dynamic_lpf1_max_hz);
#else
        dst.write_u16(0);
        dst.write_u16(0);
#endif
        // Added in MSP API 1.42
        dst.write_u8(0);  // DEPRECATED 1.43: dyn_notch_range
        dst.write_u8(0);  // DEPRECATED 1.44: dyn_notch_width_percent
        dst.write_u16(0); // dynNotchConfig.dyn_notch_q
        dst.write_u16(0); // dynNotchConfig.dyn_notch_min_hz
        dst.write_u8(rpmFiltersConfig.rpm_filter_harmonics);
        dst.write_u8(rpmFiltersConfig.rpm_filter_min_hz);
        // Added in MSP API 1.43
        dst.write_u16(0); // dynNotchConfig.dyn_notch_max_hz
        break;
    }
    case MSP_PID_ADVANCED:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_SENSOR_CONFIG: {
        // use sensorIndex_e index: 0:GyroHardware, 1:AccHardware, 2:BaroHardware, 3:MagHardware, 4:RangefinderHardware
        // hardcode a value for now
        const ImuBase& imu = pg.ahrs.get_imu();
        //dst.write_u8(ACC_BMI270);
        dst.write_u8(static_cast<uint8_t>(imu.get_acc_id_msp()));
        enum barometer_e { BAROMETER_DEFAULT = 0, BAROMETER_NONE = 1, BAROMETER_VIRTUAL = 11 };
        enum magnetometer_e { MAGNETOMETER_DEFAULT = 0, MAGNETOMETER_NONE = 1 };
        enum rangefinder_e { RANGEFINDER_NONE = 0 };
        enum optical_flow_e { OPTICAL_FLOW_NONE = 0 };
        dst.write_u8(BAROMETER_NONE);
        dst.write_u8(MAGNETOMETER_NONE);
        dst.write_u8(RANGEFINDER_NONE);
        dst.write_u8(OPTICAL_FLOW_NONE);
        break;
    }
#if defined(USE_VTX)
    case MSP_VTX_CONFIG: {
        if (pg.vtx == nullptr) {
            return MSP_RESULT_CMD_UNKNOWN;
        }
        const VTX::type_e vtxType = pg.vtx->getDeviceType();
        uint32_t vtxStatus = 0;
        pg.vtx->getStatus(vtxStatus);
        const uint8_t deviceIsReady = pg.vtx->isReady() ? 1 : 0;
        const vtx_config_t vtxConfig = pg.vtx->getConfig();
        dst.write_u8(vtxType);
        dst.write_u8(vtxConfig.band);
        dst.write_u8(vtxConfig.channel);
        dst.write_u8(vtxConfig.power);
        dst.write_u8((vtxStatus & VTX::STATUS_PIT_MODE) ? 1 : 0);
        dst.write_u16(vtxConfig.frequencyMHz);
        dst.write_u8(deviceIsReady);
        dst.write_u8(vtxConfig.lowPowerDisarm);

        // API version 1.42
        dst.write_u16(vtxConfig.pitModeFrequencyMHz);
#ifdef USE_VTX_TABLE
        dst.write_u8(1); // vtxtable is available
        dst.write_u8(vtxTableConfig.bands);
        dst.write_u8(vtxTableConfig.channels);
        dst.write_u8(vtxTableConfig.powerLevels);
#else
        dst.write_u8(0);
        dst.write_u8(0);
        dst.write_u8(0);
        dst.write_u8(0);
#endif
#if defined(USE_VTX_MSP)
        setMSP_VTX_DeviceStatusReady(pg);
#endif
        break;
    }
#endif // USE_VTX
    case MSP_TX_INFO:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_RTC:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP2_SENSOR_CONFIG_ACTIVE: {
        // just hardcode some values for now
        const ImuBase& imu = pg.ahrs.get_imu();
        dst.write_u8(static_cast<uint8_t>(imu.get_gyro_id_msp()));
        dst.write_u8(static_cast<uint8_t>(imu.get_acc_id_msp()));
        //dst.write_u8(GYRO_BMI270);
        //dst.write_u8(ACC_BMI270);
        dst.write_u8(SENSOR_NOT_AVAILABLE); // barometer
        dst.write_u8(SENSOR_NOT_AVAILABLE); // magnetometer
        dst.write_u8(SENSOR_NOT_AVAILABLE); // rangefinder
        dst.write_u8(SENSOR_NOT_AVAILABLE); // optical flow
        break;
    }
    case MSP_FC_VARIANT:
        dst.write_data(flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;
    case MSP_FC_VERSION:
        dst.write_u8(FC_VERSION_MAJOR);
        dst.write_u8(FC_VERSION_MINOR);
        dst.write_u8(FC_VERSION_PATCH_LEVEL);
        break;
    case MSP_BOARD_INFO: {
        dst.write_data(boardIdentifier, BOARD_IDENTIFIER_LENGTH);
        dst.write_u16(0); //hardware revision
        dst.write_u8(0);  // 0 == FC
        const uint8_t targetCapabilities = 0;
        dst.write_u8(targetCapabilities);
        // Target name with explicit length
        dst.write_u8(static_cast<uint8_t>(strlen(targetName)));
        dst.write_data(targetName, strlen(targetName));
        // board name strings
        dst.write_u8(0);
        dst.write_u8(0);
#if defined(USE_SIGNATURE)
        // Signature
        dst.write_data(getSignature(), SIGNATURE_LENGTH);
#else
        std::array<uint8_t, SIGNATURE_LENGTH> emptySignature {};
        dst.write_data(&emptySignature, sizeof(emptySignature));
#endif
        dst.write_u8(0xFF); // unknown MCU type
        //dst.write_u8(getMcuTypeId());
        // Added in API version 1.42
        dst.write_u8(0);
        //dst.write_u8(systemConfig()->configurationState);
        // Added in API version 1.43
        //dst.write_u16(0);
        dst.write_u16(static_cast<uint16_t>(pg.ahrs.get_imu().get_gyro_sample_rate_hz())); // informational so the configurator can display the correct gyro/pid frequencies in the drop-down
        // Configuration warnings / problems (uint32_t)
        const uint32_t configurationProblems = 0;
        dst.write_u32(configurationProblems);
        // Added in MSP API 1.44
        dst.write_u8(1); // SPI registered device count
        dst.write_u8(0); // I2C registered device count
        break;
    }
    case MSP_BUILD_INFO:
        dst.write_data(buildDate, BUILD_DATE_LENGTH);
        dst.write_data(buildTime, BUILD_TIME_LENGTH);
        //dst.write_data(shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        // Added in API version 1.46
        //dst.writeBuildInfoFlags();
        break;
    case MSP_ANALOG:
        return MSP_RESULT_CMD_UNKNOWN;
    case MSP_DEBUG: {
#if (__cplusplus >= 202002L)
        for (auto ii : std::views::iota(size_t{0}, size_t{Debug::VALUE_COUNT})) {
#else
        for (size_t ii = 0; ii < Debug::VALUE_COUNT; ++ii) {
#endif
            dst.write_u16(static_cast<uint16_t>(pg.debug.get(ii)));
        }
        break;
    }
    case MSP_UID:
        dst.write_u32(U_ID_0);
        dst.write_u32(U_ID_1);
        dst.write_u32(U_ID_2);
        break;
    case MSP_FEATURE_CONFIG:
        dst.write_u32(pg.cockpit.enabledFeatures());
        break;
    case MSP_TRANSPONDER_CONFIG: {
        dst.write_u8(0); // no providers
        break;
    }
    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
    return MSP_RESULT_ACK;
}


void MSP_Protoflight::serializeVTX(msp_parameter_group_t& pg, StreamBufWriter& dst)
{
#if defined(USE_VTX)
    assert(pg.vtx != nullptr);

    const VTX::type_e vtxType = pg.vtx->getDeviceType();
    const bool deviceReady = pg.vtx->isReady();

    uint8_t band = 0;
    uint8_t channel = 0;
    const bool bandAndChannelAvailable = pg.vtx->getBandAndChannel(band, channel);

    uint8_t powerIndex = 0;
    const bool powerIndexAvailable = pg.vtx->getPowerIndex(powerIndex);

    uint16_t frequency = 0;
    const bool frequencyAvailable = pg.vtx->getFrequency(frequency);

    uint32_t vtxStatus = 0; // pit mode and/or locked
    const bool vtxStatusAvailable = pg.vtx->getStatus(vtxStatus);

    dst.write_u8(MSP_PROTOCOL_VERSION);

    dst.write_u8(vtxType);
    dst.write_u8(deviceReady);

    dst.write_u8(bandAndChannelAvailable);
    dst.write_u8(band + 1);
    dst.write_u8(channel + 1);

    dst.write_u8(powerIndexAvailable);
    dst.write_u8(powerIndex);

    dst.write_u8(frequencyAvailable);
    dst.write_u16(frequency);

    dst.write_u8(vtxStatusAvailable);
    dst.write_u32(vtxStatus);

    // serialize power levels
    const uint8_t powerLevelCount = pg.vtx->getPowerLevelCount();
    dst.write_u8(powerLevelCount);

    std::array<uint16_t, VTX::POWER_LEVEL_COUNT> levels;
    std::array<uint16_t, VTX::POWER_LEVEL_COUNT> powers;
    pg.vtx->getPowerLevels(&levels[0], &powers[0]);

    for (size_t ii = 0; ii < powerLevelCount; ++ii) {
        dst.write_u16(levels[ii]);
        dst.write_u16(powers[ii]);
    }

    // serialize custom device status
    if (vtxType == VTX::SMART_AUDIO) {
        dst.write_u8(0);
#if false
        //!!TODO custom device status for SmartAudio
        enum { SMART_AUDIO_CUSTOM_DEVICE_STATUS_SIZE = 5 };
        dst.write_u8(VTX_CUSTOM_DEVICE_STATUS_SIZE);
        dst.write_u8(saDevice.version);
        dst.write_u8(saDevice.mode);
        dst.write_u16(saDevice.orfreq); // pit frequency
        dst.write_u8(saDevice.willBootIntoPitMode);
#endif
    } else {
        dst.write_u8(0);
    }
#else
    (void)pg;
    (void)dst;
#endif
}

void MSP_Protoflight::serializeDataflashSummaryReply(msp_parameter_group_t& pg, StreamBufWriter& dst)
{
    (void)pg;
    (void)dst;
}

void MSP_Protoflight::serializeSDCardSummaryReply(msp_parameter_group_t& pg, StreamBufWriter& dst)
{
    (void)pg;
    (void)dst;
}
