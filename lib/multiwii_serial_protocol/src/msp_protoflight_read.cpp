#include "cockpit.h"
#include "flight_controller.h"
#if defined(USE_GPS)
#include "gps.h"
#endif
#include "imu_filters.h"
#include "msp_protoflight.h"
#include "non_volatile_storage.h"
#include "protoflight_serial_port.h"
#if defined(USE_VTX)
#include "vtx.h"
#endif

#include <msp_protocol.h>

#include <ahrs.h>
#include <blackbox.h>
#include <debug.h>
#include <motor_mixer_base.h>
#include <receiver_base.h>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif


msp_result_e MSP_Protoflight::process_read_command(msp_context_t& ctx, int16_t cmd_msp, StreamBufReader& src) // NOLINT(readability-function-cognitive-complexity)
{
    // const size_t dataSize = src.bytes_remaining();

    switch (cmd_msp) {
    case MSP_SELECT_SETTING:
        src.read_u8(); // PID profile and Rate profile
        break;
    case MSP_COPY_PROFILE:
        return MSP_RESULT_ERROR;
    case MSP_SET_RAW_RC:
        return MSP_RESULT_ERROR;
    case MSP_SET_ARMING_CONFIG:
        return MSP_RESULT_ERROR;
    case MSP_SET_PID_CONTROLLER:
        return MSP_RESULT_ERROR;
    case MSP_SET_PID: {
#if (__cplusplus >= 202002L)
        for (auto ii : std::views::iota(size_t{0}, size_t{FlightController::RPY_AXIS_COUNT})) {
#else
        for (size_t ii = 0; ii < FlightController::RPY_AXIS_COUNT; ++ii) {
#endif
            const auto pid_index = static_cast<FlightController::pid_index_e>(ii);
            ctx.flight_controller.set_pid_p_msp(pid_index, src.read_u8());
            ctx.flight_controller.set_pid_i_msp(pid_index, src.read_u8());
            ctx.flight_controller.set_pid_d_msp(pid_index, src.read_u8());
        }
        const uint8_t kp = src.read_u8();
        const uint8_t ki = src.read_u8();
        const uint8_t kd = src.read_u8();
        ctx.flight_controller.set_pid_p_msp(FlightController::ROLL_ANGLE_DEGREES, kp);
        ctx.flight_controller.set_pid_i_msp(FlightController::ROLL_ANGLE_DEGREES, ki);
        ctx.flight_controller.set_pid_d_msp(FlightController::ROLL_ANGLE_DEGREES, kd);
        ctx.flight_controller.set_pid_p_msp(FlightController::PITCH_ANGLE_DEGREES, kp);
        ctx.flight_controller.set_pid_i_msp(FlightController::PITCH_ANGLE_DEGREES, ki);
        ctx.flight_controller.set_pid_d_msp(FlightController::PITCH_ANGLE_DEGREES, kd);
        // skip over PID_MAG
        src.read_u8();
        src.read_u8();
        src.read_u8();
        break;
    }
    case MSP_SET_MODE_RANGE: {
        const uint8_t mac_index = src.read_u8();
        if (mac_index >= RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            return MSP_RESULT_ERROR;
        }
        const uint8_t box_id = src.read_u8();
        const MspBox::box_t* box = MspBox::find_box_by_permanent_id(box_id);
        if (box == nullptr) {
            return MSP_RESULT_ERROR;
        }
        rc_modes_activation_condition_t mac = ctx.rc_modes.get_mode_activation_condition(mac_index);
        mac.mode_id = box->id;
        mac.auxiliary_channel_index = src.read_u8();
        mac.range_start = src.read_u8();
        mac.range_end = src.read_u8();
        if (src.bytes_remaining() >= 2) {
            mac.mode_logic = src.read_u8();
            const uint8_t linked_to_index = src.read_u8();
            const MspBox::box_t* linkBox = MspBox::find_box_by_permanent_id(linked_to_index);
            if (linkBox) {
                mac.linked_to = linkBox->id;
            }
        }
        ctx.rc_modes.set_mode_activation_condition(mac_index, mac);
        ctx.rc_modes.analyze_mode_activation_conditions();
        break;
    }
    case MSP_SET_ADJUSTMENT_RANGE: {
#if defined(USE_RC_ADJUSTMENTS)
        const uint8_t adjustment_range_index = src.read_u8();
        if (adjustment_range_index >= RC_MAX_ADJUSTMENT_RANGE_COUNT) {
            return MSP_RESULT_ERROR;
        }
        RcAdjustments& rc_adjustments = ctx.cockpit.get_rc_adjustments();
        rc_adjustment_range_t adjustment_range = rc_adjustments.get_adjustment_range(adjustment_range_index);
        src.read_u8(); // was adjustment_range.adjustmentIndex
        adjustment_range.aux_channel_index = src.read_u8();
        adjustment_range.range_start = src.read_u8();
        adjustment_range.range_end = src.read_u8();
        adjustment_range.adjustment_config = src.read_u8();
        adjustment_range.aux_switch_channel_index = src.read_u8();
        rc_adjustments.set_adjustment_range(adjustment_range_index, adjustment_range);
        rc_adjustments.active_adjustment_range_reset();
        break;
#else
        return MSP_RESULT_ERROR;
#endif
    }
    case MSP_SET_RC_TUNING: {
        if (src.bytes_remaining() < 10) {
            return MSP_RESULT_ERROR;
        }
        rates_t rates = ctx.cockpit.get_rates();
        uint8_t value = src.read_u8();
        if (rates.rc_rates[rates_t::PITCH] == rates.rc_rates[rates_t::ROLL]) {
            rates.rc_rates[rates_t::PITCH] = value;
        }
        rates.rc_rates[rates_t::ROLL] = value;

        value = src.read_u8();
        if (rates.rc_expos[rates_t::PITCH] == rates.rc_expos[rates_t::ROLL]) {
            rates.rc_expos[rates_t::PITCH] = value;
        }
        rates.rc_expos[rates_t::ROLL] = value;

        rates.rc_rates[rates_t::ROLL] = src.read_u8();
        rates.rc_rates[rates_t::PITCH] = src.read_u8();
        rates.rc_rates[rates_t::YAW] = src.read_u8();
        src.read_u8(); // skip tpa_rate
        rates.throttle_midpoint = src.read_u8();
        rates.throttle_expo = src.read_u8();
        src.read_u16(); // skip tpa_breakpoint

        if (src.bytes_remaining() >= 1) {
            rates.rc_expos[rates_t::YAW] = src.read_u8();
        }
        if (src.bytes_remaining() >= 1) {
            rates.rc_rates[rates_t::YAW] = src.read_u8();
        }
        if (src.bytes_remaining() >= 1) {
            rates.rc_rates[rates_t::PITCH] = src.read_u8();
        }
        if (src.bytes_remaining() >= 1) {
            rates.rc_expos[rates_t::PITCH] = src.read_u8();
        }
        // version 1.41
        if (src.bytes_remaining() >= 2) {
            rates.throttle_limit_type = src.read_u8();
            rates.throttle_limit_percent = src.read_u8();
        }
        // version 1.42
        if (src.bytes_remaining() >= 6) {
            rates.rate_limits[rates_t::ROLL] = src.read_u16();
            rates.rate_limits[rates_t::PITCH] = src.read_u16();
            rates.rate_limits[rates_t::YAW] = src.read_u16();
        }
        // version 1.43
        if (src.bytes_remaining() >= 1) {
            src.read_u8(); // hardcoded to RATES_TYPE_ACTUAL
        }
        ctx.cockpit.set_rates(rates);
        break;
    }
    case MSP_SET_MOTOR_CONFIG:
        return MSP_RESULT_ERROR;
#if defined(USE_GPS)
    case MSP_SET_GPS_CONFIG: {
        if (ctx.gps == nullptr) {
            return MSP_RESULT_ERROR;
        }
        gps_config_t gpsConfig = ctx.gps->get_config();
        gpsConfig.provider = src.read_u8();
        gpsConfig.sbasMode = src.read_u8();
        gpsConfig.autoConfig = src.read_u8();
        gpsConfig.autoBaud = src.read_u8();
        if (src.bytes_remaining() >= 2) {
            // Added in API version 1.43
            gpsConfig.gps_set_home_point_once = src.read_u8();
            gpsConfig.gps_ublox_use_galileo = src.read_u8();
        }
        ctx.gps->set_config(gpsConfig);
        break;
    }
#endif // USE_GPS
    case MSP_SET_COMPASS_CONFIG:
        return MSP_RESULT_ERROR;
    case MSP_SET_GPS_RESCUE:
        return MSP_RESULT_ERROR;
    case MSP_SET_MOTOR:
        return MSP_RESULT_ERROR;
    case MSP_SET_SERVO_CONFIGURATION:
        return MSP_RESULT_ERROR;
    case MSP_SET_SERVO_MIX_RULE:
        return MSP_RESULT_ERROR;
    case MSP_SET_MOTOR_3D_CONFIG:
        return MSP_RESULT_ERROR;
    case MSP_SET_RC_DEADBAND:
        return MSP_RESULT_ERROR;
    case MSP_SET_RESET_CURR_PID:
        return MSP_RESULT_ERROR;
    case MSP_SET_SENSOR_ALIGNMENT: {
        // maintain backwards compatibility for API < 1.41
        //const uint8_t gyroAlignment = src.read_u8();
        //const uint8_t axisOrder = src.read_u8();
        src.read_u8();
        //_ahrs.get_imu().setAxisOrder(static_cast<ImuBase::axis_order_e>(axisOrder));
        src.read_u8();  // discard deprecated acc_align
        // MAG
        src.read_u8();
        break;
    }
    case MSP_SET_ADVANCED_CONFIG: {
        motor_config_t motorConfig = ctx.motor_mixer.get_motor_config();
        src.read_u8();  // was gyro_sync_denom - removed in API 1.43
        src.read_u8(); // pid_process_denom
        motorConfig.device.use_continuous_update = src.read_u8();
        motorConfig.device.motor_protocol = src.read_u8();
        motorConfig.device.motor_pwm_rate = src.read_u16();
        if (src.bytes_remaining() >= 2) {
            motorConfig.motor_idle = src.read_u16();
        }
        if (src.bytes_remaining() >= 1) {
            src.read_u8(); // was gyro_use_32kHz
        }
        if (src.bytes_remaining() >= 1) {
            motorConfig.device.motor_inversion = src.read_u8();
        }
        if (src.bytes_remaining() >= 8) {
            src.read_u8();  // deprecated gyro_to_use
            src.read_u8();  // gyro_high_fsr
            src.read_u8();  // gyroMovementCalibrationThreshold
            src.read_u16(); // gyroCalibrationDuration
            src.read_u16(); // gyro_offset_yaw
            src.read_u8();  // checkOverflow
        }
        if (src.bytes_remaining() >= 1) {
            //Added in MSP API 1.42
            ctx.debug.set_mode(src.read_u8());
        }
        ctx.motor_mixer.set_motor_config(motorConfig);
        break;
    }
    case MSP_SET_FILTER_CONFIG: {
        imu_filters_config_t imu_filtersConfig = ctx.imu_filters.get_config();
        RpmFilters* rpm_filters = ctx.imu_filters.get_rpm_filters_mutable();
        rpm_filters_config_t rpm_filters_config = rpm_filters ? rpm_filters->get_config() : rpm_filters_config_t {}; // cppcheck-suppress knownConditionTrueFalse
        flight_controller_filters_config_t fc_filters = ctx.flight_controller.get_filters_config();
        imu_filtersConfig.gyro_lpf1_hz = src.read_u8();
        fc_filters.dterm_lpf1_hz = src.read_u16();
        fc_filters.yaw_lpf_hz = src.read_u16();
        if (src.bytes_remaining() >= 8) {
            imu_filtersConfig.gyro_notch1_hz = src.read_u16();
            imu_filtersConfig.gyro_notch1_cutoff = src.read_u16();
#if defined(USE_DTERM_FILTERS_EXTENDED)
            fc_filters.dterm_notch_hz = src.read_u16();
            fc_filters.dterm_notch_cutoff = src.read_u16();
#else
            src.read_u16();
            src.read_u16();
#endif
        }
        if (src.bytes_remaining() >= 4) {
            imu_filtersConfig.gyro_notch2_hz = src.read_u16();
            imu_filtersConfig.gyro_notch2_cutoff = src.read_u16();
        }
        if (src.bytes_remaining() >= 1) {
#if defined(USE_DTERM_FILTERS_EXTENDED)
            fc_filters.dterm_lpf1_type = src.read_u8();
#else
            src.read_u8();
#endif
        }
        if (src.bytes_remaining() >= 10) {
            src.read_u8(); // ignored gyro_hardware_lpf set in driver
            src.read_u8(); // was gyro_32khz_hardware_lpf
            imu_filtersConfig.gyro_lpf1_hz = src.read_u16();
            imu_filtersConfig.gyro_lpf2_hz = src.read_u16();
            imu_filtersConfig.gyro_lpf1_type = src.read_u8();
            imu_filtersConfig.gyro_lpf2_type = src.read_u8();
            fc_filters.dterm_lpf2_hz = src.read_u16();
        }

        if (src.bytes_remaining() >= 9) {
            // Added in MSP API 1.41
#if defined(USE_DTERM_FILTERS_EXTENDED)
            fc_filters.dterm_lpf2_type = src.read_u8();
#else
            src.read_u8();
#endif
            /*imu_filtersConfig.gyro_dynamic_lpf1_min_hz = */src.read_u16();
            /*imu_filtersConfig.gyro_dynamic_lpf1_max_hz = */src.read_u16();
#if defined(USE_DTERM_FILTERS_EXTENDED)
            fc_filters.dterm_dynamic_lpf1_min_hz = src.read_u16();
            fc_filters.dterm_dynamic_lpf1_max_hz = src.read_u16();
#else
            src.read_u16();
            src.read_u16();
#endif
        }

        if (src.bytes_remaining() >= 8) {
            // Added in MSP API 1.42
            src.read_u8(); // DEPRECATED 1.43: dyn_notch_range
            src.read_u8(); // DEPRECATED 1.44: dyn_notch_width_percent
            //dynamic_notch_q =
            src.read_u16();
            //dynamic_notch_min_hz =
            src.read_u16();
            rpm_filters_config.rpm_filter_harmonics = src.read_u8();
            rpm_filters_config.rpm_filter_min_hz = src.read_u8();
        }
        if (src.bytes_remaining() >= 2) {
            // Added in MSP API 1.43
            //dynamic_notch_max_hz =
            src.read_u16();
        }
        if (src.bytes_remaining() >= 2) {
            // Added in MSP API 1.44
            // dterm_lpf1_dyn_expo =
            src.read_u8();
            // dynamic_notch_count =
            src.read_u8();
        }
        ctx.imu_filters.set_config(imu_filtersConfig);
        if (rpm_filters) { // cppcheck-suppress knownConditionTrueFalse
            rpm_filters->set_config(rpm_filters_config);
        }
        ctx.flight_controller.set_filters_config(fc_filters);
        break;
    }
    case MSP_SET_PID_ADVANCED: {
        src.read_u16();
        src.read_u16();
        src.read_u16(); // was yaw_p_limit
        src.read_u8(); // reserved
        src.read_u8(); // was vbatPidCompensation
        src.read_u8(); // !!TODO::feedforward_transition
        src.read_u8(); // was low byte of dtermSetpointWeight
        src.read_u8(); // reserved
        src.read_u8(); // reserved
        src.read_u8(); // reserved
        src.read_u16(); // !!TODO: rateAccelLimit
        src.read_u16(); // !!TODO: yaw_rateAccelLimit
        if (src.bytes_remaining() >= 2) {
            src.read_u8(); // !!TODO: angle_limit
            src.read_u8(); // was levelSensitivity
        }
        if (src.bytes_remaining() >= 4) {
            src.read_u16(); // was current_pid_profile->itermThrottleThreshold
            src.read_u16(); // !!TODO: anti_gravity_gain
        }
        if (src.bytes_remaining() >= 2) {
            src.read_u16(); // was current_pid_profile->dtermSetpointWeight
        }
        if (src.bytes_remaining() >= 14) {
            // Added in MSP API 1.40
            src.read_u8(); // !!TODO: iterm_rotation
            src.read_u8(); // was current_pid_profile->smart_feedforward
#if defined(USE_ITERM_RELAX)
            iterm_relax_config_t itermRelaxConfig = ctx.flight_controller.get_iterm_relax_config();
            itermRelaxConfig.iterm_relax = src.read_u8();
            itermRelaxConfig.iterm_relax_type = src.read_u8();
            ctx.flight_controller.set_iterm_relax_config(itermRelaxConfig);
#else
            src.read_u8();
            src.read_u8();
#endif
            src.read_u8(); // !!TODO: abs_control_gain
            src.read_u8(); // !!TODO: throttle_boost
            src.read_u8(); // !!TODO: acro_trainer_angle_limit
            // PID controller kick terms
            ctx.flight_controller.set_pid_k_msp(FlightController::ROLL_RATE_DPS, src.read_u16());
            ctx.flight_controller.set_pid_k_msp(FlightController::PITCH_RATE_DPS, src.read_u16());
            ctx.flight_controller.set_pid_k_msp(FlightController::YAW_RATE_DPS, src.read_u16());
            src.read_u8(); // was current_pid_profile->anti_gravityMode
        }
        break;
    }
    case MSP_SET_SENSOR_CONFIG:
        src.read_u8(); // acc
        src.read_u8(); // baro
        src.read_u8(); // map
        break;
    case MSP_SET_FEATURE_CONFIG:
        ctx.cockpit.set_features(src.read_u32());
        break;
    case MSP_SET_BEEPER_CONFIG:
        return MSP_RESULT_ERROR;
    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
        //rollDegrees = src.read_u16();
        //pitchDegrees = src.read_u16();
        //yawDegrees = src.read_u16();
        break;
    case MSP_SET_MIXER_CONFIG: {
        mixer_config_t mixerConfig = ctx.motor_mixer.get_mixer_config();
        mixerConfig.type = src.read_u8();
        mixerConfig.yaw_motors_reversed = src.read_u8();
        ctx.motor_mixer.set_mixer_config(mixerConfig);
        break;
    }
    case MSP_SET_RX_CONFIG: {
        rx_config_t rx_config = ctx.cockpit.get_rx_config();

        rx_config.serial_rx_provider = src.read_u8();
        rx_config.max_check = src.read_u16();
        rx_config.mid_rc = src.read_u16();
        rx_config.min_check = src.read_u16();
        rx_config.spektrum_sat_bind = src.read_u8();
        if (src.bytes_remaining() >= 4) {
            rx_config.rx_min_usec = src.read_u16();
            rx_config.rx_max_usec = src.read_u16();
        }
        if (src.bytes_remaining() >= 4) {
            src.read_u8(); // not required in API 1.44, was rx_config.rcInterpolation
            src.read_u8(); // not required in API 1.44, was rx_config.rcInterpolationInterval
            rx_config.airModeActivateThreshold = static_cast<uint8_t>((src.read_u16() - 1000) / 10);
        }
        if (src.bytes_remaining() >= 6) {
#if defined(USE_RX_SPI)
            rxSpiConfig_mutable()->rx_spi_protocol = src.read_u8();
            rxSpiConfig_mutable()->rx_spi_id = src.read_u32();
            rxSpiConfig_mutable()->rx_spi_rf_channel_count = src.read_u8();
#else
            src.read_u8();
            src.read_u32();
            src.read_u8();
#endif
        }
        if (src.bytes_remaining() >= 1) {
            rx_config.fpvCam_angle_degrees = src.read_u8();
        }

        ctx.cockpit.set_rx_config(rx_config);
        break;
    }
    case MSP_SET_FAILSAFE_CONFIG: {
        failsafe_config_t failsafe_config{};
        failsafe_config.delay_deciseconds = src.read_u8();
        failsafe_config.landing_time_seconds = src.read_u8();
        failsafe_config.throttle_pwm = src.read_u16();
        failsafe_config.switch_mode = src.read_u8();
        failsafe_config.throttle_low_delay_deciseconds = src.read_u16();
        failsafe_config.procedure = src.read_u8();
        ctx.cockpit.set_failsafe_config(failsafe_config);
        break;
    }
    case MSP_SET_RXFAIL_CONFIG: {
        const size_t index = src.read_u8();
        if (index < RX_MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            rx_failsafe_channel_configs_t rx_failsafe_channel_configs = ctx.cockpit.get_rx_failsafe_channel_configs();
            rx_failsafe_channel_configs[index].mode = src.read_u8();
            rx_failsafe_channel_configs[index].step = RX::channel_valueToFailStep(src.read_u16());
            ctx.cockpit.set_rx_failsafe_channel_configs(rx_failsafe_channel_configs);
            break;
        }
        return MSP_RESULT_ERROR;
    }
    case MSP_SET_RSSI_CONFIG: {
        rx_config_t rx_config = ctx.cockpit.get_rx_config();
        rx_config.rssi_channel = src.read_u8();
        ctx.cockpit.set_rx_config(rx_config);
        break;
    }
    case MSP_SET_RX_MAP:
        return MSP_RESULT_ERROR;
    case MSP_SET_CF_SERIAL_CONFIG: {
#if false
        const uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);
        const size_t dataSize = src.bytes_remaining();
        if (dataSize % portConfigSize != 0) {
            return MSP_RESULT_ERROR;
        }
        uint8_t remainingPortsInPacket = dataSize / portConfigSize;
        while (remainingPortsInPacket--) {
            const uint8_t identifier = src.read_u8();
            ProtoFlightSerialPort::config_t* config = nullptr; //!!TODO serialFindPortConfigurationMutable(identifier);
            if (!config) {
                return MSP_RESULT_ERROR;
            }
            config->functionMask = src.read_u16();
            config->msp_baudrate_index = src.read_u8();
            config->gps_baudrate_index = src.read_u8();
            config->telemetry_baudrate_index = src.read_u8();
            config->blackbox_baudrate_index = src.read_u8();
        }
        break;
#else
        return MSP_RESULT_ERROR;
#endif
    }
    case MSP2_COMMON_SET_SERIAL_CONFIG:
        return MSP_RESULT_ERROR;
    case MSP_SET_ACC_TRIM:
        return MSP_RESULT_ERROR;
    case MSP_ACC_CALIBRATION:
        return MSP_RESULT_ERROR;
    case MSP_MAG_CALIBRATION:
        return MSP_RESULT_ERROR;
    case MSP_EEPROM_WRITE:
        if (ctx.motor_mixer.motors_is_on()) {
            // can't save to non volatile storage if the motors are on
            return MSP_RESULT_ERROR;
        }
        ctx.nvs.store_all(ctx.imu_filters, ctx.flight_controller, ctx.motor_mixer, ctx.cockpit, ctx.rc_modes);
        break;
#ifdef USE_BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if (ctx.blackbox != nullptr && ctx.blackbox->may_edit_config()) {
            Blackbox::config_t blackboxConfig = ctx.blackbox->get_config();
            blackboxConfig.device = static_cast<Blackbox::device_e>(src.read_u8());
            const int rateNumerator = src.read_u8();
            const int rateDenominator = src.read_u8();
            uint16_t pRatio = 0;
            if (src.bytes_remaining() >= 2) {
                // p_ratio specified, so use it directly
                pRatio = src.read_u16();
            } else {
                // p_ratio not specified in MSP, so calculate it from old rateNum and rateDenom
                pRatio = static_cast<uint16_t>(ctx.blackbox->calculate_p_denominator(rateNumerator, rateDenominator));
            }

            if (src.bytes_remaining() >= 1) {
                // sample_rate specified, so use it directly
                blackboxConfig.sample_rate = static_cast<Blackbox::sample_rate_e>(src.read_u8());
            } else {
                // sample_rate not specified in MSP, so calculate it from old p_ratio
                blackboxConfig.sample_rate = static_cast<Blackbox::sample_rate_e>(ctx.blackbox->calculate_sample_rate(pRatio));
            }

            // Added in MSP API 1.45
            if (src.bytes_remaining() >= 4) {
                blackboxConfig.fields_disabled_mask = src.read_u32();
            }
            ctx.blackbox->init(blackboxConfig);
        }
        break;
#endif

#if defined(USE_VTX)
    case MSP_SET_VTX_CONFIG: {
        VTX::type_e vtxType = ctx.vtx->get_device_type();
        vtx_config_t vtxConfig = ctx.vtx->get_config();
        const uint16_t newFrequency = src.read_u16();
        if (newFrequency <= VTX::MSP_BAND_CHANNEL_CHECK_VALUE) {  // Value is band and channel
            const auto newBand = static_cast<uint8_t>((newFrequency / 8) + 1);
            const auto newChannel = static_cast<uint8_t>((newFrequency % 8) + 1);
            vtxConfig.band = newBand;
            vtxConfig.channel = newChannel;
            vtxConfig.frequency_mhz = VTX::lookup_frequency(newBand, newChannel);
        } else if (newFrequency <= VTX::MAX_FREQUENCY_MHZ) { // Value is frequency in MHz
            vtxConfig.band = 0;
            vtxConfig.frequency_mhz = newFrequency;
        }
        if (src.bytes_remaining() >= 2) {
            vtxConfig.power = src.read_u8();
            const uint8_t newPitmode = src.read_u8();
            if (vtxType != VTX::UNKNOWN) {
                // Delegate pitmode to vtx directly
                uint32_t vtxCurrentStatus {};
                ctx.vtx->get_status(vtxCurrentStatus);
                if ((vtxCurrentStatus & VTX::STATUS_PIT_MODE) != newPitmode) {
                    ctx.vtx->set_pit_mode(newPitmode);
                }
            }
        }
        if (src.bytes_remaining() > 0) {
            vtxConfig.lowPowerDisarm = src.read_u8();
        }
        // API version 1.42 - this parameter kept separate since clients may already be supplying
        if (src.bytes_remaining() >= 2) {
            vtxConfig.pit_mode_frequency_mhz = src.read_u16();
        }
        // API version 1.42 - extensions for non-encoded versions of the band, channel or frequency
        if (src.bytes_remaining() >= 4) {
            // Added standalone values for band, channel and frequency to move
            // away from the flawed encoded combined method originally implemented.
            uint8_t newBand = src.read_u8();
            const uint8_t newChannel = src.read_u8();
            uint16_t newFreq = src.read_u16();
            if (newBand) {
                newFreq = VTX::lookup_frequency(newBand, newChannel);
            }
            vtxConfig.band = newBand;
            vtxConfig.channel = newChannel;
            vtxConfig.frequency_mhz = newFreq;
        }
        ctx.vtx->set_config(vtxConfig);
        // API version 1.42 - extensions for vtxtable support
        if (src.bytes_remaining() >= 4) {
#if defined(USE_VTX_TABLE)
#else
            src.read_u8();
            src.read_u8();
            src.read_u8();
            src.read_u8();
#endif // USE_VTX_TABLE
        }
#if defined(USE_VTX_MSP)
        setMspVtxDeviceStatusReady(srcDesc);
#endif
        break;
    }
#endif // USE_VTX

    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

void MSP_Protoflight::set_msp_vtx_device_status_ready(msp_context_t& ctx)
{
    (void)ctx;
}
