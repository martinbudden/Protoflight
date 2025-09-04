#include "MSP_Box.h"
#include "MSP_ProtoFlight.h"
#include "MSP_Protocol.h"
#include "features.h"
#include "version.h"

#include <AHRS.h>
#include <Debug.h>
#include <Features.h>
#include <FlightController.h>
#include <IMU_Filters.h>
#include <MSP_protocol.h>
#include <RadioController.h>
#include <ReceiverBase.h>

const char* const targetName = "TARGETNAME";

const char* const buildDate = "MMM DD YYYY"; // MMM = Jan/Feb/...

const char* const buildTime = "HH:MM:SS";

//enum  gyroHardware_e { GYRO_NONE = 0, GYRO_DEFAULT = 1, GYRO_VIRTUAL = 20 };
//enum  accelerationSensor_e { ACC_DEFAULT = 0, ACC_NONE = 1, ACC_VIRTUAL = 21 };

enum { SIGNATURE_LENGTH = 32 };

enum { U_ID_0 = 0 };
enum { U_ID_1 = 1 };
enum { U_ID_2 = 2 };

enum  armingDisableFlags_e {
    ARMING_DISABLED_NO_GYRO         = (1U << 0U),
    ARMING_DISABLED_FAILSAFE        = (1U << 1U),
    ARMING_DISABLED_RX_FAILSAFE     = (1U << 2U),
    ARMING_DISABLED_NOT_DISARMED    = (1U << 3U),
    ARMING_DISABLED_BOXFAILSAFE     = (1U << 4U),
    ARMING_DISABLED_RUNAWAY_TAKEOFF = (1U << 5U),
    ARMING_DISABLED_CRASH_DETECTED  = (1U << 6U),
    ARMING_DISABLED_THROTTLE        = (1U << 7U),
    ARMING_DISABLED_ANGLE           = (1U << 8U),
    ARMING_DISABLED_BOOT_GRACE_TIME = (1U << 9U),
    ARMING_DISABLED_NOPREARM        = (1U << 10U),
    ARMING_DISABLED_LOAD            = (1U << 11U),
    ARMING_DISABLED_CALIBRATING     = (1U << 12U),
    ARMING_DISABLED_CLI             = (1U << 13U),
    ARMING_DISABLED_CMS_MENU        = (1U << 14U),
    ARMING_DISABLED_BST             = (1U << 15U),
    ARMING_DISABLED_MSP             = (1U << 16U),
    ARMING_DISABLED_PARALYZE        = (1U << 17U),
    ARMING_DISABLED_GPS             = (1U << 18U),
    ARMING_DISABLED_RESC            = (1U << 19U),
    ARMING_DISABLED_DSHOT_TELEM     = (1U << 20U),
    ARMING_DISABLED_REBOOT_REQUIRED = (1U << 21U),
    ARMING_DISABLED_DSHOT_BITBANG   = (1U << 22U),
    ARMING_DISABLED_ACC_CALIBRATION = (1U << 23U),
    ARMING_DISABLED_MOTOR_PROTOCOL  = (1U << 24U),
    ARMING_DISABLED_CRASHFLIP       = (1U << 25U),
    ARMING_DISABLED_ARM_SWITCH      = (1U << 26U), // Needs to be the last element, since it's always activated if one of the others is active when arming
};

enum { ARMING_DISABLE_FLAGS_COUNT = 27 };

armingDisableFlags_e getArmingDisableFlags()
{
    return ARMING_DISABLED_ARM_SWITCH;
}


static const char* const flightControllerIdentifier = FC_FIRMWARE_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char* const TARGET_BOARD_IDENTIFIER = "A405";
static const char* const boardIdentifier = TARGET_BOARD_IDENTIFIER;

static inline int constrain(int value, int low, int high)
{
    return value < low ? low : value > high ? high : value;
}


MSP_ProtoFlight::MSP_ProtoFlight(NonVolatileStorage& nonVolatileStorage, Features& features, AHRS& ahrs, FlightController& flightController, RadioController& radioController, ReceiverBase& receiver, Debug& debug) :
    _nonVolatileStorage(nonVolatileStorage),
    _features(features),
    _ahrs(ahrs),
    _flightController(flightController),
    _radioController(radioController),
    _receiver(receiver),
    _debug(debug)
{
    //_mspBox.init(features, ahrs, flightController);
    enum { MSP_OVERRIDE_OFF = false, AIRMODE_OFF = false, ANTI_GRAVITY_OFF = false };
    _mspBox.init(
        ahrs.isSensorAvailable(AHRS::SENSOR_ACCELEROMETER),
        features.featureIsEnabled(Features::FEATURE_INFLIGHT_ACC_CAL),
        MSP_OVERRIDE_OFF,
        AIRMODE_OFF,
        ANTI_GRAVITY_OFF
    );
}

/*!
Returns true if the command was processed, false otherwise.
May set mspPostProcessFunc to a function to be called once the command has been processed
*/
MSP_Base::result_e MSP_ProtoFlight::processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn)
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
        dst.writeU16(static_cast<uint16_t>(_ahrs.getTaskIntervalMicroSeconds()));
        dst.writeU16(0); // I2C error counter
        // NOLINTBEGIN(hicpp-signed-bitwise)
        dst.writeU16(_ahrs.isSensorAvailable(AHRS::SENSOR_ACCELEROMETER)
            | _ahrs.isSensorAvailable(AHRS::SENSOR_BAROMETER) << 1
            | _ahrs.isSensorAvailable(AHRS::SENSOR_MAGNETOMETER) << 2
            | _ahrs.isSensorAvailable(AHRS::SENSOR_GPS) << 3
            | _ahrs.isSensorAvailable(AHRS::SENSOR_RANGEFINDER) << 4
            | _ahrs.isSensorAvailable(AHRS::SENSOR_GYROSCOPE) << 5
        );
        // NOLINTEND(hicpp-signed-bitwise)
        std::bitset<MSP_Box::BOX_COUNT> flightModeFlags;
        const size_t flagBits = _mspBox.packFlightModeFlags(flightModeFlags, _flightController);
        dst.writeData(&flightModeFlags, 4); // unconditional part of flags, first 32 bits
        dst.writeU8(_flightController.getCurrentPidProfileIndex());
        dst.writeU16(10); //constrain(getAverageSystemLoadPercent(), 0, LOAD_PERCENTAGE_ONE))
        if (cmdMSP == MSP_STATUS_EX) {
            dst.writeU8(_flightController.getPidProfileCount());
            dst.writeU8(_flightController.getCurrentControlRateProfileIndex());
        } else { // MSP_STATUS
            dst.writeU16(0); // gyro cycle time
        }

        // write flightModeFlags header. Lowest 4 bits contain number of bytes that follow
        // header is emmitted even when all bits fit into 32 bits to allow future extension
        size_t byteCount = (flagBits - 32 + 7) / 8;        // 32 already stored, round up
        byteCount = constrain(static_cast<uint8_t>(byteCount), 0, 15);        // limit to 16 bytes (128 bits)
        dst.writeU8(static_cast<uint8_t>(byteCount));
        dst.writeData(reinterpret_cast<uint8_t*>(&flightModeFlags) + 4, byteCount); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast)

        // Write arming disable flags
        // 1 byte, flag count
        dst.writeU8(ARMING_DISABLE_FLAGS_COUNT);
        // 4 bytes, flags
        const uint32_t armingDisableFlags = getArmingDisableFlags();
        dst.writeU32(armingDisableFlags);

        // config state flags - bits to indicate the state of the configuration, reboot required, etc.
        // other flags can be added as needed
        const bool rebootRequired = false;
        dst.writeU8(rebootRequired);

        dst.writeU16(0); // CPU temperature, added in API v1.46
        break;
    }
    case MSP_FAILSAFE_CONFIG: {
        const RadioController::failsafe_t failsafe = _radioController.getFailsafe();;
        dst.writeU8(failsafe.delay);
        dst.writeU8(failsafe.landing_time);
        dst.writeU16(failsafe.throttle);
        dst.writeU8(failsafe.switch_mode);
        dst.writeU16(failsafe.throttle_low_delay);
        dst.writeU8(failsafe.procedure);
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
        _ahrs.readMagRaw(magX, magY, magZ);
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
        const RadioController::rates_t rates = _radioController.getRates();
        dst.writeU8(static_cast<uint8_t>(rates.rcRates[RadioController::ROLL]));
        dst.writeU8(static_cast<uint8_t>(rates.rcExpos[RadioController::ROLL]));
        dst.writeU8(static_cast<uint8_t>(rates.rates[RadioController::ROLL]));
        dst.writeU8(static_cast<uint8_t>(rates.rates[RadioController::PITCH]));
        dst.writeU8(static_cast<uint8_t>(rates.rates[RadioController::YAW]));
        dst.writeU8(0); // was tpa_rate
        dst.writeU8(rates.throttleMidpoint);
        dst.writeU8(rates.throttleExpo);
        dst.writeU16(0);   // was tpa_breakpoint
        dst.writeU8(static_cast<uint8_t>(rates.rcExpos[RadioController::YAW]));
        dst.writeU8(static_cast<uint8_t>(rates.rcRates[RadioController::YAW]));
        dst.writeU8(static_cast<uint8_t>(rates.rcRates[RadioController::PITCH]));
        dst.writeU8(static_cast<uint8_t>(rates.rcExpos[RadioController::PITCH]));

        // added in 1.41
        dst.writeU8(rates.throttleLimitType);
        dst.writeU8(rates.throttleLimitPercent);

        // added in 1.42
        dst.writeU8(static_cast<uint8_t>(rates.rateLimits[RadioController::ROLL]));
        dst.writeU8(static_cast<uint8_t>(rates.rateLimits[RadioController::PITCH]));
        dst.writeU8(static_cast<uint8_t>(rates.rateLimits[RadioController::YAW]));

        // added in 1.43
        dst.writeU8(rates.ratesType);
        break;
    }
    case MSP_ATTITUDE: {
        const Quaternion quaternion = _ahrs.getOrientationForInstrumentationUsingLock();

        dst.writeU16(static_cast<uint16_t>(quaternion.calculateRollDegrees()));
        dst.writeU16(static_cast<uint16_t>(quaternion.calculatePitchDegrees()));
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

    case MSP_PID:
        for (size_t ii = 0; ii < FlightController::PID_COUNT; ++ii) {
            const auto pidIndex = static_cast<FlightController::pid_index_e>(ii);
            const FlightController::PIDF_uint16_t pid =  _flightController.getPID_MSP(pidIndex);
            dst.writeU8(static_cast<uint8_t>(pid.kp));
            dst.writeU8(static_cast<uint8_t>(pid.ki));
            dst.writeU8(static_cast<uint8_t>(pid.kd));
        }
        break;

    case MSP_PIDNAMES:
        for (size_t ii = 0; ii < FlightController::PID_COUNT; ++ii) {
            const std::string& pidName =  _flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
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
        const IMU_Filters::config_t imuFiltersConfig = static_cast<IMU_Filters&>(_ahrs.getIMU_Filters()).getConfig(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
        const FlightController::filters_config_t fcFilters = _flightController.getFiltersConfig();
        dst.writeU8(static_cast<uint8_t>(imuFiltersConfig.gyro_lpf1_hz));
        dst.writeU16(fcFilters.dterm_lpf1_hz);
        dst.writeU16(fcFilters.yaw_lpf_hz);
        dst.writeU8(static_cast<uint8_t>(imuFiltersConfig.gyro_lpf1_hz));
        dst.writeU16(fcFilters.dterm_lpf1_hz);
        dst.writeU16(fcFilters.yaw_lpf_hz);
        dst.writeU16(imuFiltersConfig.gyro_notch1_hz);
        dst.writeU16(imuFiltersConfig.gyro_notch1_cutoff);
        dst.writeU16(fcFilters.dterm_notch_hz);
        dst.writeU16(fcFilters.dterm_notch_cutoff);
        dst.writeU16(imuFiltersConfig.gyro_notch2_hz);
        dst.writeU16(imuFiltersConfig.gyro_notch2_cutoff);
        dst.writeU8(fcFilters.dterm_lpf1_type);
        dst.writeU8(imuFiltersConfig.gyro_hardware_lpf);
        dst.writeU8(0); // was gyro_32khz_hardware_lpf
        dst.writeU16(imuFiltersConfig.gyro_lpf1_hz);
        dst.writeU16(imuFiltersConfig.gyro_lpf2_hz);
        dst.writeU8(imuFiltersConfig.gyro_lpf1_type);
        dst.writeU8(imuFiltersConfig.gyro_lpf2_type);
        dst.writeU16(fcFilters.dterm_lpf2_hz);
        // Added in MSP API 1.41
        dst.writeU8(fcFilters.dterm_lpf2_type);
        dst.writeU16(imuFiltersConfig.gyro_dynamic_lpf1_min_hz);
        dst.writeU16(imuFiltersConfig.gyro_dynamic_lpf1_max_hz);
        dst.writeU16(fcFilters.dterm_dynamic_lpf1_min_hz);
        dst.writeU16(fcFilters.dterm_dynamic_lpf1_max_hz);
        break;
    }
    case MSP_SENSOR_CONFIG: {
        // use sensorIndex_e index: 0:GyroHardware, 1:AccHardware, 2:BaroHardware, 3:MagHardware, 4:RangefinderHardware
        // hardcode a value for now
        const IMU_Base& imu = _ahrs.getIMU();
        //dst.writeU8(ACC_BMI270);
        dst.writeU8(static_cast<uint8_t>(imu.getAccIdMSP()));
        dst.writeU8(_ahrs.getBarometerID_MSP());
        dst.writeU8(_ahrs.getMagnetometerID_MSP());
        dst.writeU8(_ahrs.getRangeFinderID_MSP());
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
            dst.writeU16(_debug.get(ii));
        }
        break;
    }

    case MSP_UID:
        dst.writeU32(U_ID_0);
        dst.writeU32(U_ID_1);
        dst.writeU32(U_ID_2);
        break;

    case MSP_FEATURE_CONFIG:
        dst.writeU32(_features.enabledFeatures());
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

void MSP_ProtoFlight::rebootFn(serialPort_t* serialPort)
{
    (void)serialPort;

    _flightController.motorsSwitchOff();

    switch (_rebootMode) {
    case REBOOT_FIRMWARE:
        //systemReset();
        break;
    case REBOOT_BOOTLOADER_ROM:
        //systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
        break;
    case REBOOT_MSC:
        [[fallthrough]];
    case REBOOT_MSC_UTC: {
        //const int16_t timezoneOffsetMinutes = 0;//(_rebootMode == REBOOT_MSC) ? timeConfig()->tz_offsetMinutes : 0;
        //systemResetToMsc(timezoneOffsetMinutes);
        break;
    }
    case REBOOT_BOOTLOADER_FLASH:
        //systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);
        break;
    default:
        return;
    }
}

MSP_Base::result_e MSP_ProtoFlight::processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBuf& src)
{
    switch (cmdMSP) {
    case MSP_BOXNAMES: {
        const int page = src.bytesRemaining() ? src.readU8() : 0;
        _mspBox.serializeBoxReplyBoxName(dst, page);
        break;
    }
    case MSP_BOXIDS: {
        const int page = src.bytesRemaining() ? src.readU8() : 0;
        _mspBox.serializeBoxReplyPermanentId(dst, page);
        break;
    }
    case MSP_REBOOT:
        if (src.bytesRemaining()) {
            _rebootMode = src.readU8();
            if (_rebootMode >= REBOOT_COUNT || _rebootMode == REBOOT_MSC || _rebootMode == REBOOT_MSC_UTC) {
                return RESULT_ERROR;
            }
        } else {
            _rebootMode = REBOOT_FIRMWARE;
        }

        dst.writeU8(_rebootMode);

        if (postProcessFn) {
                *postProcessFn = static_cast<MSP_Base::postProcessFnPtr>(&MSP_ProtoFlight::rebootFn);
        }

        break;
    case MSP_RESET_CONF: {
        if (src.bytesRemaining() >= 1) {
            // Added in MSP API 1.42
            src.readU8();
        }

        const bool success = false;
        if (!_flightController.isArmingFlagSet(FlightController::ARMED)) {
            //success = resetEEPROM(); //!!TODO: implement this
            //if (success && postProcessFn) {
            if (postProcessFn) {
                _rebootMode = REBOOT_FIRMWARE;
                *postProcessFn = static_cast<MSP_Base::postProcessFnPtr>(&MSP_ProtoFlight::rebootFn);
            }
        }
        // Added in API version 1.42
        dst.writeU8(success);
        break;
    }
    default:
        return processOutCommand(cmdMSP, dst, srcDesc, postProcessFn);
    }
    return RESULT_ACK;
}
