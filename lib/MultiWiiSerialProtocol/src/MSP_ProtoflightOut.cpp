#include "Cockpit.h"
#include "Features.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"
#include "VTX_Base.h"
#include "version.h"

#include <AHRS.h>
#include <Debug.h>
#include <MSP_Protocol.h>
#include <RPM_Filters.h>
#include <ReceiverBase.h>
#include <cassert>


const char* const targetName = "TARGETNAME";

const char* const buildDate = "MMM DD YYYY"; // MMM = Jan/Feb/...

const char* const buildTime = "HH:MM:SS";

//enum  gyroHardware_e { GYRO_NONE = 0, GYRO_DEFAULT = 1, GYRO_VIRTUAL = 20 };
//enum  accelerationSensor_e { ACC_DEFAULT = 0, ACC_NONE = 1, ACC_VIRTUAL = 21 };

enum { SIGNATURE_LENGTH = 32 };

enum { U_ID_0 = 0 };
enum { U_ID_1 = 1 };
enum { U_ID_2 = 2 };


static const char* const flightControllerIdentifier = FC_FIRMWARE_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char* const TARGET_BOARD_IDENTIFIER = "A405";
static const char* const boardIdentifier = TARGET_BOARD_IDENTIFIER;

/*!
Returns true if the command was processed, false otherwise.
May set mspPostProcessFunc to a function to be called once the command has been processed
*/
MSP_Base::result_e MSP_Protoflight::processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn)
{
    (void)srcDesc;
    (void)postProcessFn;

    switch (cmdMSP) {
    case MSP_API_VERSION:
        dst.writeU8(MSP_PROTOCOL_VERSION);
        dst.writeU8(MSP_API_VERSION_MAJOR);
        dst.writeU8(MSP_API_VERSION_MINOR);
        break;
    case MSP_STATUS_EX:
        [[fallthrough]];
    case MSP_STATUS: {
        dst.writeU16(static_cast<uint16_t>(_flightController.getTaskIntervalMicroseconds()));
        dst.writeU16(0); // I2C error counter
        static constexpr uint16_t SENSOR_ACCELEROMETER = 0x01;
        static constexpr uint16_t SENSOR_GYROSCOPE = 0x01U << 5U;
        dst.writeU16(SENSOR_ACCELEROMETER | SENSOR_GYROSCOPE);
        std::bitset<MSP_Box::BOX_COUNT> flightModeFlags;
        const size_t flagBits = _mspBox.packFlightModeFlags(flightModeFlags, _cockpit);
        dst.writeData(&flightModeFlags, 4); // unconditional part of flags, first 32 bits
        dst.writeU8(_nonVolatileStorage.getCurrentPidProfileIndex());
        dst.writeU16(10); //constrain(getAverageSystemLoadPercent(), 0, LOAD_PERCENTAGE_ONE))
        if (cmdMSP == MSP_STATUS_EX) {
            dst.writeU8(NonVolatileStorage::PID_PROFILE_COUNT);
            dst.writeU8(_nonVolatileStorage.getCurrentRateProfileIndex());
        } else { // MSP_STATUS
            dst.writeU16(0); // gyro cycle time
        }

        // write flightModeFlags header. Lowest 4 bits contain number of bytes that follow
        // header is emmitted even when all bits fit into 32 bits to allow future extension
        size_t byteCount = (flagBits - 32 + 7) / 8;        // 32 already stored, round up
        byteCount = static_cast<uint8_t>(byteCount);
        if (byteCount > 15) {
            byteCount = 15; // limit to 16 bytes (128 bits)
        }
        dst.writeU8(static_cast<uint8_t>(byteCount));
        dst.writeData(reinterpret_cast<uint8_t*>(&flightModeFlags) + 4, byteCount); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast)

        // Write arming disable flags
        // 1 byte, flag count
        dst.writeU8(ARMING_DISABLE_FLAGS_COUNT);
        // 4 bytes, flags
        const uint32_t armingDisableFlags = _cockpit.getArmingDisableFlags();
        dst.writeU32(armingDisableFlags);

        // config state flags - bits to indicate the state of the configuration, reboot required, etc.
        // other flags can be added as needed
        const bool rebootRequired = false;
        dst.writeU8(rebootRequired);

        dst.writeU16(0); // CPU temperature, added in API v1.46
        break;
    }
    case MSP_FAILSAFE_CONFIG: {
        const Cockpit::failsafe_config_t failsafeConfig = _cockpit.getFailsafeConfig();
        dst.writeU8(failsafeConfig.delay_deciseconds);
        dst.writeU8(failsafeConfig.landing_time_seconds);
        dst.writeU16(failsafeConfig.throttle_pwm);
        dst.writeU8(failsafeConfig.switch_mode);
        dst.writeU16(failsafeConfig.throttle_low_delay_deciseconds);
        dst.writeU8(failsafeConfig.procedure);
        break;
    }
    case MSP_SET_MIXER_CONFIG: {
        enum { MIXER_QUAD_X = 3 };
        dst.writeU8(MIXER_QUAD_X); // mixer mode
        dst.writeU8(0); // yaw_motors_reversed
        break;
    }
    case MSP_RAW_IMU: {
        IMU_Base::xyz_int32_t acc {};
        _ahrs.readAccRaw(acc.x, acc.y, acc.z);
        dst.writeU16(static_cast<uint16_t>(acc.x));
        dst.writeU16(static_cast<uint16_t>(acc.y));
        dst.writeU16(static_cast<uint16_t>(acc.z));
        IMU_Base::xyz_int32_t gyro {};
        _ahrs.readGyroRaw(gyro.x, gyro.y, gyro.z);
        dst.writeU16(static_cast<uint16_t>(gyro.x));
        dst.writeU16(static_cast<uint16_t>(gyro.y));
        dst.writeU16(static_cast<uint16_t>(gyro.z));
        // write zeros for magnetometer
        int32_t magX = 0;
        int32_t magY = 0;
        int32_t magZ = 0;
        //_ahrs.readMagRaw(magX, magY, magZ);
        dst.writeU16(static_cast<uint16_t>(magX));
        dst.writeU16(static_cast<uint16_t>(magY));
        dst.writeU16(static_cast<uint16_t>(magZ));
        break;
    }
    case MSP_NAME:
        dst.writeStringWithZeroTerminator("Martin Budden"); // pilot/aircraft name
        break;

    case MSP_MOTOR:
        for (size_t ii = 0; ii < 8; ++ii) {
            dst.writeU16(0);
        }
        break;

    case MSP_RC: {
        const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM();
        dst.writeU16(controls.throttle);
        dst.writeU16(controls.roll);
        dst.writeU16(controls.pitch);
        dst.writeU16(controls.yaw);
        for (size_t ii = 0; ii < _receiver.getAuxiliaryChannelCount(); ++ii) {
            dst.writeU16(_receiver.getAuxiliaryChannel(ii));
        }
        break;
    }
    case MSP_RC_TUNING: {
        const Cockpit::rates_t rates = _cockpit.getRates();
        dst.writeU8(static_cast<uint8_t>(rates.rcRates[Cockpit::ROLL]));
        dst.writeU8(static_cast<uint8_t>(rates.rcExpos[Cockpit::ROLL]));
        dst.writeU8(static_cast<uint8_t>(rates.rates[Cockpit::ROLL]));
        dst.writeU8(static_cast<uint8_t>(rates.rates[Cockpit::PITCH]));
        dst.writeU8(static_cast<uint8_t>(rates.rates[Cockpit::YAW]));
        dst.writeU8(0); // was tpa_rate
        dst.writeU8(rates.throttleMidpoint);
        dst.writeU8(rates.throttleExpo);
        dst.writeU16(0);   // was tpa_breakpoint
        dst.writeU8(static_cast<uint8_t>(rates.rcExpos[Cockpit::YAW]));
        dst.writeU8(static_cast<uint8_t>(rates.rcRates[Cockpit::YAW]));
        dst.writeU8(static_cast<uint8_t>(rates.rcRates[Cockpit::PITCH]));
        dst.writeU8(static_cast<uint8_t>(rates.rcExpos[Cockpit::PITCH]));

        // added in 1.41
        dst.writeU8(rates.throttleLimitType);
        dst.writeU8(rates.throttleLimitPercent);

        // added in 1.42
        dst.writeU8(static_cast<uint8_t>(rates.rateLimits[Cockpit::ROLL]));
        dst.writeU8(static_cast<uint8_t>(rates.rateLimits[Cockpit::PITCH]));
        dst.writeU8(static_cast<uint8_t>(rates.rateLimits[Cockpit::YAW]));

        // added in 1.43
        dst.writeU8(Cockpit::RATES_TYPE_ACTUAL); // hardcoded, since we only support RATES_TYPE_ACTUAL rates.ratesType);
        break;
    }
    case MSP_ATTITUDE: {
        const Quaternion quaternion {}; //!!TODO:= _ahrs.getOrientationForInstrumentationUsingLock();

        dst.writeU16(static_cast<uint16_t>(quaternion.calculateRollDegrees()*10.0F));
        dst.writeU16(static_cast<uint16_t>(quaternion.calculatePitchDegrees()*10.0F));
        dst.writeU16(static_cast<uint16_t>(quaternion.calculateYawDegrees()));
        break;
    }
    case MSP_ALTITUDE:
        dst.writeU32(0);
        dst.writeU16(0);
        break;

    case MSP_SONAR_ALTITUDE:
        dst.writeU32(0);
        break;

    case MSP_BOARD_ALIGNMENT_CONFIG:
        //dst.writeU16(boardAlignment()->rollDegrees);
        //dst.writeU16(boardAlignment()->pitchDegrees);
        //dst.writeU16(boardAlignment()->yawDegrees);
        dst.writeU16(0);
        dst.writeU16(0);
        dst.writeU16(0);
        break;

    case MSP_ARMING_CONFIG:
        dst.writeU8(0); //armingConfig()->auto_disarm_delay);
        dst.writeU8(0);
        dst.writeU8(0); //imuConfig()->small_angle);
        dst.writeU8(0); //armingConfig()->gyro_cal_on_first_arm);
        break;

    case MSP_PID: {
        for (size_t ii = 0; ii < FlightController::PID_COUNT; ++ii) {
            const auto pidIndex = static_cast<FlightController::pid_index_e>(ii);
            const FlightController::PIDF_uint16_t pid = _flightController.getPID_MSP(pidIndex);
            dst.writeU8(static_cast<uint8_t>(pid.kp));
            dst.writeU8(static_cast<uint8_t>(pid.ki));
            dst.writeU8(static_cast<uint8_t>(pid.kd));
        }
        const FlightController::PIDF_uint16_t pid = _flightController.getPID_MSP(FlightController::ROLL_ANGLE_DEGREES);
        dst.writeU8(static_cast<uint8_t>(pid.kp));
        dst.writeU8(static_cast<uint8_t>(pid.ki));
        dst.writeU8(static_cast<uint8_t>(pid.kd));
        break;
    }
    case MSP_PIDNAMES:
        for (size_t ii = 0; ii < FlightController::PID_COUNT; ++ii) {
            const std::string& pidName = _flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
            dst.writeStringWithZeroTerminator(pidName);
        }
        break;

    case MSP_PID_CONTROLLER: {
        enum { PID_CONTROLLER_BETAFLIGHT = 1 };
        dst.writeU8(PID_CONTROLLER_BETAFLIGHT);
        break;
    }
    case MSP_SENSOR_ALIGNMENT: {
        const uint8_t gyroAlignment = 0; //gyroDeviceConfig(0)->alignment;
        dst.writeU8(gyroAlignment);
        dst.writeU8(gyroAlignment);  // Starting with 4.0 gyro and acc alignment are the same
        dst.writeU8(0); // mag alignment

        // API 1.41 - Add multi-gyro indicator, selected gyro, and support for separate gyro 1 & 2 alignment
        dst.writeU8(0); // getGyroDetectionFlags()
        enum { GYRO_CONFIG_USE_GYRO_1 = 0 };
        dst.writeU8(GYRO_CONFIG_USE_GYRO_1);
        dst.writeU8(0); // alignment
        enum { ALIGN_DEFAULT = 0 };
        dst.writeU8(ALIGN_DEFAULT);
        break;
    }
    case MSP_FILTER_CONFIG : {
        auto& imuFilters = static_cast<IMU_Filters&>(_ahrs.getIMU_Filters()); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
        const IMU_Filters::config_t imuFiltersConfig = imuFilters.getConfig();
        const FlightController::filters_config_t fcFilters = _flightController.getFiltersConfig();
        const RPM_Filters* rpmFilters = imuFilters.getRPM_Filters();
        const RPM_Filters::config_t rpmFiltersConfig = rpmFilters ? rpmFilters->getConfig() : RPM_Filters::config_t {}; // cppcheck-suppress knownConditionTrueFalse

        dst.writeU8(static_cast<uint8_t>(imuFiltersConfig.gyro_lpf1_hz));
        dst.writeU16(fcFilters.dterm_lpf1_hz);
        dst.writeU16(fcFilters.yaw_lpf_hz);
        dst.writeU8(static_cast<uint8_t>(imuFiltersConfig.gyro_lpf1_hz));
        dst.writeU16(fcFilters.dterm_lpf1_hz);
        dst.writeU16(fcFilters.yaw_lpf_hz);
        dst.writeU16(imuFiltersConfig.gyro_notch1_hz);
        dst.writeU16(imuFiltersConfig.gyro_notch1_cutoff);
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.writeU16(fcFilters.dterm_notch_hz);
        dst.writeU16(fcFilters.dterm_notch_cutoff);
#else
        dst.writeU16(0);
        dst.writeU16(0);
#endif
        dst.writeU16(imuFiltersConfig.gyro_notch2_hz);
        dst.writeU16(imuFiltersConfig.gyro_notch2_cutoff);
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.writeU8(fcFilters.dterm_lpf1_type);
#else
        dst.writeU8(0);
#endif
        dst.writeU8(0); // gyro_hardware_lpf set in driver
        dst.writeU8(0); // was gyro_32khz_hardware_lpf
        dst.writeU16(imuFiltersConfig.gyro_lpf1_hz);
        dst.writeU16(imuFiltersConfig.gyro_lpf2_hz);
        dst.writeU8(imuFiltersConfig.gyro_lpf1_type);
        dst.writeU8(imuFiltersConfig.gyro_lpf2_type);
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.writeU16(fcFilters.dterm_lpf2_hz);
#else
        dst.writeU16(0);
#endif
        // Added in MSP API 1.41
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.writeU8(fcFilters.dterm_lpf2_type);
#else
        dst.writeU8(0);
#endif
        dst.writeU16(imuFiltersConfig.gyro_dynamic_lpf1_min_hz);
        dst.writeU16(imuFiltersConfig.gyro_dynamic_lpf1_max_hz);
#if defined(USE_DTERM_FILTERS_EXTENDED)
        dst.writeU16(fcFilters.dterm_dynamic_lpf1_min_hz);
        dst.writeU16(fcFilters.dterm_dynamic_lpf1_max_hz);
#else
        dst.writeU16(0);
        dst.writeU16(0);
#endif
        // Added in MSP API 1.42
        dst.writeU8(0);  // DEPRECATED 1.43: dyn_notch_range
        dst.writeU8(0);  // DEPRECATED 1.44: dyn_notch_width_percent
        dst.writeU16(0); // dynNotchConfig.dyn_notch_q
        dst.writeU16(0); // dynNotchConfig.dyn_notch_min_hz
        dst.writeU8(rpmFiltersConfig.rpm_filter_harmonics);
        dst.writeU8(rpmFiltersConfig.rpm_filter_min_hz);
        // Added in MSP API 1.43
        dst.writeU16(0); // dynNotchConfig.dyn_notch_max_hz
        break;
    }
    case MSP_SENSOR_CONFIG: {
        // use sensorIndex_e index: 0:GyroHardware, 1:AccHardware, 2:BaroHardware, 3:MagHardware, 4:RangefinderHardware
        // hardcode a value for now
        const IMU_Base& imu = _ahrs.getIMU();
        //dst.writeU8(ACC_BMI270);
        dst.writeU8(static_cast<uint8_t>(imu.getAccIdMSP()));
        enum barometer_e { BAROMETER_DEFAULT = 0, BAROMETER_NONE = 1, BAROMETER_VIRTUAL = 11 };
        enum magnetometer_e { MAGNETOMETER_DEFAULT = 0, MAGNETOMETER_NONE = 1 };
        enum rangefinder_e { RANGEFINDER_NONE = 0 };
        dst.writeU8(BAROMETER_NONE);
        dst.writeU8(MAGNETOMETER_NONE);
        dst.writeU8(RANGEFINDER_NONE);
        break;
    }
    // Added in MSP API 1.46
    case MSP2_SENSOR_CONFIG_ACTIVE: {
        // just hardcode some values for now
        const IMU_Base& imu = _ahrs.getIMU();
        dst.writeU8(static_cast<uint8_t>(imu.getGyroIdMSP()));
        dst.writeU8(static_cast<uint8_t>(imu.getAccIdMSP()));
        //dst.writeU8(GYRO_BMI270);
        //dst.writeU8(ACC_BMI270);
        dst.writeU8(SENSOR_NOT_AVAILABLE); // baro
        dst.writeU8(SENSOR_NOT_AVAILABLE); // mag
        dst.writeU8(SENSOR_NOT_AVAILABLE); // Rangefinder
        break;
    }
    case MSP_FC_VARIANT:
        dst.writeData(flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    case MSP_FC_VERSION:
        dst.writeU8(FC_VERSION_MAJOR);
        dst.writeU8(FC_VERSION_MINOR);
        dst.writeU8(FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO: {
        dst.writeData(boardIdentifier, BOARD_IDENTIFIER_LENGTH);
        dst.writeU16(0); //hardware revision
        dst.writeU8(0);  // 0 == FC
        const uint8_t targetCapabilities = 0;
        dst.writeU8(targetCapabilities);

        // Target name with explicit length
        dst.writeU8(static_cast<uint8_t>(strlen(targetName)));
        dst.writeData(targetName, strlen(targetName));

        // board name strings
        dst.writeU8(0);
        dst.writeU8(0);

#if defined(USE_SIGNATURE)
        // Signature
        dst.writeData(getSignature(), SIGNATURE_LENGTH);
#else
        std::array<uint8_t, SIGNATURE_LENGTH> emptySignature;
        memset(&emptySignature[0], 0, sizeof(emptySignature));
        dst.writeData(&emptySignature, sizeof(emptySignature));
#endif

        dst.writeU8(0xFF); // unknown MCU type
        //dst.writeU8(getMcuTypeId());

        // Added in API version 1.42
        dst.writeU8(0);
        //dst.writeU8(systemConfig()->configurationState);

        // Added in API version 1.43
        //dst.writeU16(0);
        dst.writeU16(static_cast<uint16_t>(_ahrs.getIMU().getGyroSampleRateHz())); // informational so the configurator can display the correct gyro/pid frequencies in the drop-down

        // Configuration warnings / problems (uint32_t)
        const uint32_t configurationProblems = 0;
        dst.writeU32(configurationProblems);

        // Added in MSP API 1.44
        dst.writeU8(1); // SPI registered device count
        dst.writeU8(0); // I2C registered device count
        break;
    }

    case MSP_BUILD_INFO:
        dst.writeData(buildDate, BUILD_DATE_LENGTH);
        dst.writeData(buildTime, BUILD_TIME_LENGTH);
        //dst.writeData(shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        // Added in API version 1.46
        //dst.writeBuildInfoFlags();
        break;

    case MSP_DEBUG: {
        for (size_t ii = 0; ii < Debug::VALUE_COUNT; ++ii) {
            dst.writeU16(static_cast<uint16_t>(_debug.get(ii)));
        }
        break;
    }

    case MSP_UID:
        dst.writeU32(U_ID_0);
        dst.writeU32(U_ID_1);
        dst.writeU32(U_ID_2);
        break;

    case MSP_FEATURE_CONFIG:
        dst.writeU32(_cockpit.enabledFeatures());
        break;

    case MSP_TRANSPONDER_CONFIG: {
        dst.writeU8(0); // no providers
        break;
    }

    default:
        return RESULT_CMD_UNKNOWN;
    }
    return RESULT_ACK;
}


void MSP_Protoflight::serializeVTX(StreamBuf& dst)
{
#if defined(USE_VTX)
    assert(_vtx != nullptr);

    const VTX_Base::type_e vtxType = _vtx->getDeviceType();
    const bool deviceReady = _vtx->isReady();

    uint8_t band = 0;
    uint8_t channel = 0;
    const bool bandAndChannelAvailable = _vtx->getBandAndChannel(band, channel);

    uint8_t powerIndex = 0;
    const bool powerIndexAvailable = _vtx->getPowerIndex(powerIndex);

    uint16_t frequency = 0;
    const bool frequencyAvailable = _vtx->getFrequency(frequency);

    //uint8_t vtxStatus = 0; // pit mode and/or locked
    //!!const bool vtxStatusAvailable = _vtx->getStatus();

    dst.writeU8(MSP_PROTOCOL_VERSION);

    dst.writeU8(vtxType);
    dst.writeU8(deviceReady);

    dst.writeU8(bandAndChannelAvailable);
    dst.writeU8(band);
    dst.writeU8(channel);

    dst.writeU8(powerIndexAvailable);
    dst.writeU8(powerIndex);

    dst.writeU8(frequencyAvailable);
    dst.writeU16(frequency);

    //dst.writeU8(vtxStatusAvailable);
    //dst.writeU32(vtxStatus);

    // serialize power levels
    //vtxCommonSerializePowerLevels(vtxDevice, dst);

    const uint8_t powerLevelCount = _vtx->getPowerLevelCount();
    dst.writeU8(powerLevelCount);

#if false
    uint16_t levels[VTX_TABLE_MAX_POWER_LEVELS];
    uint16_t powers[VTX_TABLE_MAX_POWER_LEVELS];
    vtxCommonGetVTXPowerLevels(vtxDevice, levels, powers);

    for (int i = 0; i < powerLevelCount; i++) {
        dst.writeU16(levels[i]);
        dst.writeU16(powers[i]);
    }
#endif

    // serialize custom device status
    if (vtxType == VTX_Base::SMART_AUDIO) {
        dst.writeU8(0);
#if false
        //!!TODO custom device status for SmartAudio
        enum { SMART_AUDIO_CUSTOM_DEVICE_STATUS_SIZE = 5 };
        dst.writeU8(VTX_CUSTOM_DEVICE_STATUS_SIZE);
        dst.writeU8(saDevice.version);
        dst.writeU8(saDevice.mode);
        dst.writeU16(saDevice.orfreq); // pit frequency
        dst.writeU8(saDevice.willBootIntoPitMode);
#endif
    } else {
        dst.writeU8(0);
    }
#else
    (void)dst;
#endif
}
