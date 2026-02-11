#include "Cockpit.h"
#include "Debug.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"
#include "ProtoflightSerialPort.h"

#include <AHRS.h>
#include <Blackbox.h>
#include <MSP_Protocol.h>
#include <ReceiverBase.h>

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


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
#if (__cplusplus >= 202002L)
        for (auto ii : std::views::iota(size_t{0}, size_t{FlightController::RPY_AXIS_COUNT})) {
#else
        for (size_t ii = 0; ii < FlightController::RPY_AXIS_COUNT; ++ii) {
#endif
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
    case MSP_SET_MODE_RANGE: {
        const uint8_t macIndex = src.readU8();
        if (macIndex >= RC_Modes::MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            return RESULT_ERROR;
        }
        const uint8_t boxId = src.readU8();
        const MSP_Box::box_t* box = MSP_Box::findBoxByPermanentId(boxId);
        if (box == nullptr) {
            return RESULT_ERROR;
        }
        RC_Modes::mode_activation_condition_t mac = _cockpit.getRC_Modes().getModeActivationCondition(macIndex);
        mac.modeId = static_cast<MSP_Box::id_e>(box->id);
        mac.auxiliaryChannelIndex = src.readU8();
        mac.range.start_step = src.readU8();
        mac.range.end_step = src.readU8();
        if (src.bytesRemaining() >= 2) {
            mac.modeLogic = static_cast<RC_Modes::mode_logic_e>(src.readU8());
            const uint8_t linkedToIndex = src.readU8();
            const MSP_Box::box_t* linkBox = MSP_Box::findBoxByPermanentId(linkedToIndex);
            if (linkBox) {
                mac.linkedTo = static_cast<MSP_Box::id_e>(linkBox->id);
            }
        }
        _cockpit.getRC_ModesMutable().setModeActivationCondition(macIndex, mac);
        _cockpit.getRC_ModesMutable().analyzeModeActivationConditions();
        break;
    }
    case MSP_SET_ADJUSTMENT_RANGE: {
#if defined(USE_RC_ADJUSTMENTS)
        const uint8_t adjustmentRangeIndex = src.readU8();
        if (adjustmentRangeIndex >= RC_Adjustments::MAX_ADJUSTMENT_RANGE_COUNT) {
            return RESULT_ERROR;
        }
        RC_Adjustments& rcAdjustments = _cockpit.getRC_Adjustments();
        RC_Adjustments::adjustment_range_t adjustmentRange = rcAdjustments.getAdjustmentRange(adjustmentRangeIndex);
        src.readU8(); // was adjustmentRange.adjustmentIndex
        adjustmentRange.auxChannelIndex = src.readU8();
        adjustmentRange.range.start_step = src.readU8();
        adjustmentRange.range.end_step = src.readU8();
        adjustmentRange.adjustmentConfig = src.readU8();
        adjustmentRange.auxSwitchChannelIndex = src.readU8();
        rcAdjustments.setAdjustmentRange(adjustmentRangeIndex, adjustmentRange);
        rcAdjustments.activeAdjustmentRangeReset();
        break;
#else
        return RESULT_ERROR;
#endif
    }
    case MSP_SET_RC_TUNING: {
        if (src.bytesRemaining() < 10) {
            return RESULT_ERROR;
        }
        rates_t rates = _cockpit.getRates();
        uint8_t value = src.readU8();
        if (rates.rcRates[rates_t::PITCH] == rates.rcRates[rates_t::ROLL]) {
            rates.rcRates[rates_t::PITCH] = value;
        }
        rates.rcRates[rates_t::ROLL] = value;

        value = src.readU8();
        if (rates.rcExpos[rates_t::PITCH] == rates.rcExpos[rates_t::ROLL]) {
            rates.rcExpos[rates_t::PITCH] = value;
        }
        rates.rcExpos[rates_t::ROLL] = value;

        rates.rcRates[rates_t::ROLL] = src.readU8();
        rates.rcRates[rates_t::PITCH] = src.readU8();
        rates.rcRates[rates_t::YAW] = src.readU8();
        src.readU8(); // skip tpa_rate
        rates.throttleMidpoint = src.readU8();
        rates.throttleExpo = src.readU8();
        src.readU16(); // skip tpa_breakpoint

        if (src.bytesRemaining() >= 1) {
            rates.rcExpos[rates_t::YAW] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcRates[rates_t::YAW] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcRates[rates_t::PITCH] = src.readU8();
        }
        if (src.bytesRemaining() >= 1) {
            rates.rcExpos[rates_t::PITCH] = src.readU8();
        }
        // version 1.41
        if (src.bytesRemaining() >= 2) {
            rates.throttleLimitType = src.readU8();
            rates.throttleLimitPercent = src.readU8();
        }
        // version 1.42
        if (src.bytesRemaining() >= 6) {
            rates.rateLimits[rates_t::ROLL] = src.readU16();
            rates.rateLimits[rates_t::PITCH] = src.readU16();
            rates.rateLimits[rates_t::YAW] = src.readU16();
        }
        // version 1.43
        if (src.bytesRemaining() >= 1) {
            src.readU8(); // hardcoded to RATES_TYPE_ACTUAL
        }
        _cockpit.setRates(rates, _flightController);
        break;
    }
    case MSP_SET_MOTOR_CONFIG:
        return RESULT_ERROR;
#if defined(USE_GPS)
    case MSP_SET_GPS_CONFIG: {
        if (_gps == nullptr) {
            return RESULT_ERROR;
        }
        GPS::config_t gpsConfig = _gps->getConfig();
        gpsConfig.provider = src.readU8();
        gpsConfig.sbasMode = src.readU8();
        gpsConfig.autoConfig = src.readU8();
        gpsConfig.autoBaud = src.readU8();
        if (src.bytesRemaining() >= 2) {
            // Added in API version 1.43
            gpsConfig.gps_set_home_point_once = src.readU8();
            gpsConfig.gps_ublox_use_galileo = src.readU8();
        }
        _gps->setConfig(gpsConfig);
        break;
    }
#endif // USE_GPS
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
    case MSP_SET_ADVANCED_CONFIG: {
        MotorMixerBase::motor_config_t motorConfig = _flightController.getMotorMixer().get_motor_config();
        src.readU8();  // was gyro_sync_denom - removed in API 1.43
        src.readU8(); // pid_process_denom
        motorConfig.device.use_continuous_update = src.readU8();
        motorConfig.device.motor_protocol = src.readU8();
        motorConfig.device.motor_pwm_rate = src.readU16();
        if (src.bytesRemaining() >= 2) {
            motorConfig.motor_idle = src.readU16();
        }
        if (src.bytesRemaining() >= 1) {
            src.readU8(); // was gyro_use_32kHz
        }
        if (src.bytesRemaining() >= 1) {
            motorConfig.device.motor_inversion = src.readU8();
        }
        if (src.bytesRemaining() >= 8) {
            src.readU8();  // deprecated gyro_to_use
            src.readU8();  // gyro_high_fsr
            src.readU8();  // gyroMovementCalibrationThreshold
            src.readU16(); // gyroCalibrationDuration
            src.readU16(); // gyro_offset_yaw
            src.readU8();  // checkOverflow
        }
        if (src.bytesRemaining() >= 1) {
            //Added in MSP API 1.42
            _debug.setMode(static_cast<debug_mode_e>(src.readU8()));
        }
        _flightController.getMotorMixerMutable().set_motor_config(motorConfig);
        break;
    }
    case MSP_SET_FILTER_CONFIG: {
        auto& imuFilters = static_cast<IMU_Filters&>(_ahrs.getIMU_Filters()); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
        IMU_Filters::config_t imuFiltersConfig = imuFilters.getConfig();
        RpmFilters* rpmFilters = imuFilters.getRPM_FiltersMutable();
        RpmFilters::config_t rpmFiltersConfig = rpmFilters ? rpmFilters->get_config() : RpmFilters::config_t {}; // cppcheck-suppress knownConditionTrueFalse
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
            rpmFilters->set_config(rpmFiltersConfig);
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
#if defined(USE_ITERM_RELAX)
            FlightController::iterm_relax_config_t itermRelaxConfig = _flightController.getITermRelaxConfig();
            itermRelaxConfig.iterm_relax = src.readU8();
            itermRelaxConfig.iterm_relax_type = src.readU8();
            _flightController.setITermRelaxConfig(itermRelaxConfig);
#else
            src.readU8();
            src.readU8();
#endif
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
    case MSP_SET_MIXER_CONFIG: {
        MotorMixerBase::mixer_config_t mixerConfig = _flightController.getMotorMixer().get_mixer_config();
        mixerConfig.type = src.readU8();
        mixerConfig.yaw_motors_reversed = src.readU8();
        _flightController.getMotorMixerMutable().set_mixer_config(mixerConfig);
        break;
    }
    case MSP_SET_RX_CONFIG: {
        RX::config_t rxConfig = _cockpit.getRX_Config();

        rxConfig.serial_rx_provider = src.readU8();
        rxConfig.max_check = src.readU16();
        rxConfig.mid_rc = src.readU16();
        rxConfig.min_check = src.readU16();
        rxConfig.spektrum_sat_bind = src.readU8();
        if (src.bytesRemaining() >= 4) {
            rxConfig.rx_min_usec = src.readU16();
            rxConfig.rx_max_usec = src.readU16();
        }
        if (src.bytesRemaining() >= 4) {
            src.readU8(); // not required in API 1.44, was rxConfig.rcInterpolation
            src.readU8(); // not required in API 1.44, was rxConfig.rcInterpolationInterval
            rxConfig.airModeActivateThreshold = static_cast<uint8_t>((src.readU16() - 1000) / 10);
        }
        if (src.bytesRemaining() >= 6) {
#if defined(USE_RX_SPI)
            rxSpiConfigMutable()->rx_spi_protocol = src.readU8();
            rxSpiConfigMutable()->rx_spi_id = src.readU32();
            rxSpiConfigMutable()->rx_spi_rf_channel_count = src.readU8();
#else
            src.readU8();
            src.readU32();
            src.readU8();
#endif
        }
        if (src.bytesRemaining() >= 1) {
            rxConfig.fpvCamAngleDegrees = src.readU8();
        }

        _cockpit.setRX_Config(rxConfig);
        break;
    }
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
    case MSP_SET_RXFAIL_CONFIG: {
        const size_t index = src.readU8();
        if (index < RX::MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            RX::failsafe_channel_configs_t rxFailsafeChannelConfigs = _cockpit.getRX_FailsafeChannelConfigs();
            rxFailsafeChannelConfigs[index].mode = src.readU8();
            rxFailsafeChannelConfigs[index].step = RX::channelValueToFailStep(src.readU16());
            _cockpit.setRX_FailsafeChannelConfigs(rxFailsafeChannelConfigs);
            break;
        }
        return RESULT_ERROR;
    }
    case MSP_SET_RSSI_CONFIG: {
        RX::config_t rxConfig = _cockpit.getRX_Config();
        rxConfig.rssi_channel = src.readU8();
        _cockpit.setRX_Config(rxConfig);
        break;
    }
    case MSP_SET_RX_MAP:
        return RESULT_ERROR;
    case MSP_SET_CF_SERIAL_CONFIG: {
#if false
        const uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);
        const size_t dataSize = src.bytesRemaining();
        if (dataSize % portConfigSize != 0) {
            return RESULT_ERROR;
        }
        uint8_t remainingPortsInPacket = dataSize / portConfigSize;
        while (remainingPortsInPacket--) {
            const uint8_t identifier = src.readU8();
            ProtoFlightSerialPort::config_t* config = nullptr; //!!TODO serialFindPortConfigurationMutable(identifier);
            if (!config) {
                return RESULT_ERROR;
            }
            config->functionMask = src.readU16();
            config->msp_baudrateIndex = src.readU8();
            config->gps_baudrateIndex = src.readU8();
            config->telemetry_baudrateIndex = src.readU8();
            config->blackbox_baudrateIndex = src.readU8();
        }
        break;
#else
        return RESULT_ERROR;
#endif
    }
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
        _nonVolatileStorage.storeAll(_imuFilters, _flightController, _cockpit, _pidProfileIndex, _ratesProfileIndex);
        break;
#ifdef USE_BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if (_blackbox != nullptr && _blackbox->mayEditConfig()) {
            Blackbox::config_t blackboxConfig = _blackbox->getConfig();
            blackboxConfig.device = static_cast<Blackbox::device_e>(src.readU8());
            const int rateNumerator = src.readU8();
            const int rateDenominator = src.readU8();
            uint16_t pRatio = 0;
            if (src.bytesRemaining() >= 2) {
                // p_ratio specified, so use it directly
                pRatio = src.readU16();
            } else {
                // p_ratio not specified in MSP, so calculate it from old rateNum and rateDenom
                pRatio = static_cast<uint16_t>(_blackbox->calculateP_Denominator(rateNumerator, rateDenominator));
            }

            if (src.bytesRemaining() >= 1) {
                // sample_rate specified, so use it directly
                blackboxConfig.sample_rate = static_cast<Blackbox::sample_rate_e>(src.readU8());
            } else {
                // sample_rate not specified in MSP, so calculate it from old p_ratio
                blackboxConfig.sample_rate = static_cast<Blackbox::sample_rate_e>(_blackbox->calculateSampleRate(pRatio));
            }

            // Added in MSP API 1.45
            if (src.bytesRemaining() >= 4) {
                blackboxConfig.fieldsDisabledMask = src.readU32();
            }
            _blackbox->init(blackboxConfig);
        }
        break;
#endif

#if defined(USE_VTX)
    case MSP_SET_VTX_CONFIG: {
        VTX::type_e vtxType = _vtx->getDeviceType();
        VTX::config_t vtxConfig = _vtx->getConfig();
        const uint16_t newFrequency = src.readU16();
        if (newFrequency <= VTX::MSP_BAND_CHANNEL_CHECK_VALUE) {  // Value is band and channel
            const auto newBand = static_cast<uint8_t>((newFrequency / 8) + 1);
            const auto newChannel = static_cast<uint8_t>((newFrequency % 8) + 1);
            vtxConfig.band = newBand;
            vtxConfig.channel = newChannel;
            vtxConfig.frequencyMHz = VTX::lookupFrequency(newBand, newChannel);
        } else if (newFrequency <= VTX::MAX_FREQUENCY_MHZ) { // Value is frequency in MHz
            vtxConfig.band = 0;
            vtxConfig.frequencyMHz = newFrequency;
        }
        if (src.bytesRemaining() >= 2) {
            vtxConfig.power = src.readU8();
            const uint8_t newPitmode = src.readU8();
            if (vtxType != VTX::UNKNOWN) {
                // Delegate pitmode to vtx directly
                uint32_t vtxCurrentStatus {};
                _vtx->getStatus(vtxCurrentStatus);
                if ((vtxCurrentStatus & VTX::STATUS_PIT_MODE) != newPitmode) {
                    _vtx->setPitMode(newPitmode);
                }
            }
        }
        if (src.bytesRemaining()) {
            vtxConfig.lowPowerDisarm = src.readU8();
        }
        // API version 1.42 - this parameter kept separate since clients may already be supplying
        if (src.bytesRemaining() >= 2) {
            vtxConfig.pitModeFrequencyMHz = src.readU16();
        }
        // API version 1.42 - extensions for non-encoded versions of the band, channel or frequency
        if (src.bytesRemaining() >= 4) {
            // Added standalone values for band, channel and frequency to move
            // away from the flawed encoded combined method originally implemented.
            uint8_t newBand = src.readU8();
            const uint8_t newChannel = src.readU8();
            uint16_t newFreq = src.readU16();
            if (newBand) {
                newFreq = VTX::lookupFrequency(newBand, newChannel);
            }
            vtxConfig.band = newBand;
            vtxConfig.channel = newChannel;
            vtxConfig.frequencyMHz = newFreq;
        }
        _vtx->setConfig(vtxConfig);
        // API version 1.42 - extensions for vtxtable support
        if (src.bytesRemaining() >= 4) {
#if defined(USE_VTX_TABLE)
#else
            src.readU8();
            src.readU8();
            src.readU8();
            src.readU8();
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
        return RESULT_ERROR;
    }
    return RESULT_ACK;
}

void MSP_Protoflight::setMSP_VTX_DeviceStatusReady(descriptor_t srcDesc)
{
    (void)srcDesc;
}
