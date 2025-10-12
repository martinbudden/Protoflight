#include "BlackboxProtoFlight.h"
#include "IMU_Filters.h"
#include "RadioController.h"

#include <BlackboxCallbacksBase.h>
#include <DynamicIdleController.h>

// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#ifndef BLACKBOX_PRINT_HEADER_LINE
#define BLACKBOX_PRINT_HEADER_LINE(name, format, ...) case __COUNTER__: \
                                                headerPrintfHeaderLine(name, format, __VA_ARGS__); \
                                                break;
#define BLACKBOX_PRINT_HEADER_LINE_CUSTOM(...) case __COUNTER__: \
                                                    {__VA_ARGS__}; \
                                               break;
#endif
// NOLINTEND(cppcoreguidelines-macro-usage)


/*!
Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0.
Returns true iff transmission is complete, otherwise call again later to continue transmission.
*/
Blackbox::write_e BlackboxProtoFlight::writeSystemInformation()
{
    constexpr float radiansToDegrees {180.0F / static_cast<float>(M_PI)};
    constexpr float gyroScale {radiansToDegrees * 10.0F};

    enum { PWM_TYPE_BRUSHED = 4 };
    enum { SERIALRX_TARGET_CUSTOM = 11 };
    enum { DEBUG_MODE_RX_STATE_TIME = 76 };

    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    if (!headerReserveBufferSpace()) {
        return WRITE_NOT_COMPLETE;
    }
// See https://github.com/betaflight/blackbox-log-viewer/blob/master/src/flightlog_parser.js for parsing of fields
    const FlightController::filters_config_t fcFiltersConfig = _flightController.getFiltersConfig();
    const FlightController::anti_gravity_config_t antiGravityConfig = _flightController.getAntiGravityConfig();

    const RadioController::rates_t rates = _radioController.getRates();

    const IMU_Filters::config_t imuFiltersConfig = _imuFilters.getConfig();

    const DynamicIdleController* dynamicIdleController = _flightController.getMixer().getDynamicIdleController();
    const DynamicIdleController::config_t* dynamicIdleControllerConfig = dynamicIdleController ? &dynamicIdleController->getConfig() : nullptr;

    //const BlackboxCallbacksBase::rates_t& currentControlRateProfile = _callbacks.currentControlRateProfile();
    //const pidProfile_t& currentPidProfile = _callbacks.getCurrentPidProfile();
    //enum { PID_ROLL, PID_PITCH, PID_YAW, PID_LEVEL, PID_MAG, PID_ITEM_COUNT };

    struct firmware_t {
        const char* date;
        const char* time;
        const char* version;
    };
#if defined(FIRMWARE)
    const firmware_t firmware = FIRMWARE;
#else
    const firmware_t firmware = {.date="2025.Jun.28",.time="00:00:00",.version="0.0.1"};
#endif

    switch (_xmitState.headerIndex) {
// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s",                   "Protoflight");
        BLACKBOX_PRINT_HEADER_LINE("Firmware revision", "%s %s (%s) %s",    "Protoflight", firmware.version, "7", "alpha");
        BLACKBOX_PRINT_HEADER_LINE("Firmware date", "%s %s",                firmware.date, firmware.time);
        //BLACKBOX_PRINT_HEADER_LINE("DeviceUID", "%08x%08x%08x",             U_ID_0, U_ID_1, U_ID_2);
        //BLACKBOX_PRINT_HEADER_LINE("Board information", "%s %s",            getManufacturerId(), getBoardName());
        BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s",              "2025-01-01T00:00:00.000");
        BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s",                      "TestCraft");
// Example
//H I interval:256  is 8kHz PIDloop, 32 is 1kHz PIDloop, 16 is 500Hz pidloop, 8 is 250Hz pidloop
//H P interval:1/8  is 1kHz logging
//H P denom:32
// me
//H Craft name:TestCraft
//H I interval:6
//H P interval:1
//H P ratio:6
        BLACKBOX_PRINT_HEADER_LINE("I interval", "%d",                      _IInterval); // definitely needed
        BLACKBOX_PRINT_HEADER_LINE("P interval", "%d",                      _PInterval); // definitely needed
// "P denom" ignored by blackbox-log-view
//        BLACKBOX_PRINT_HEADER_LINE("P denom", "%d",                         static_cast<uint16_t>(blackboxIInterval / blackboxPInterval));

        BLACKBOX_PRINT_HEADER_LINE("minthrottle", "%d",                     1000);
        BLACKBOX_PRINT_HEADER_LINE("maxthrottle", "%d",                     2000);
// Baseflight uses a gyroScale that gives radians per microsecond as output, whereas Cleanflight produces degrees
// per second and leaves the conversion to radians per microsecond to the IMU. Let's just convert Cleanflight's scale to
// match Baseflight so we can use Baseflight's IMU for both:
// sysConfig.gyroScale * (Math.PI / 180.0) * 0.000001
        BLACKBOX_PRINT_HEADER_LINE("gyro_scale","0x%x",                     BlackboxEncoder::castFloatBytesToInt(0.000001F / gyroScale));
        BLACKBOX_PRINT_HEADER_LINE("motorOutput", "%d,%d",                  158,2047);
        BLACKBOX_PRINT_HEADER_LINE("acc_1G", "%u",                          4096);

/*
H vbat_scale:110
H vbatcellvoltage:33,35,43
H vbatref:113
H currentSensor:0,235 // current
H looptime:125
H gyro_sync_denom:1
H pid_process_denom:1
H thr_mid:50
H thr_expo:0
*/
        BLACKBOX_PRINT_HEADER_LINE("vbat_scale", "%u",                      110); // I think this means 11V, ie 3S

        BLACKBOX_PRINT_HEADER_LINE("vbatcellvoltage", "%u,%u,%u",           33,35,43); // min cell voltage, warning cell voltage, max cell voltage
        BLACKBOX_PRINT_HEADER_LINE("vbatref", "%u",                         112);

        BLACKBOX_PRINT_HEADER_LINE("currentSensor", "%d,%d",                0, 235); // current meter offset, current meter scale
        BLACKBOX_PRINT_HEADER_LINE("looptime", "%d",                        _flightController.getTaskIntervalMicroseconds());
        BLACKBOX_PRINT_HEADER_LINE("gyro_sync_denom", "%d",                 1); // not sure if this is used
        BLACKBOX_PRINT_HEADER_LINE("pid_process_denom", "%d",               1); // nots sure if this is used

        BLACKBOX_PRINT_HEADER_LINE("thr_mid", "%d",                         rates.throttleMidpoint);
        BLACKBOX_PRINT_HEADER_LINE("thr_expo", "%d",                        rates.throttleExpo);
        BLACKBOX_PRINT_HEADER_LINE("mixer_type", "%s",                      "LEGACY"); // "LEGACY", "LINEAR", "DYNAMIC", "EZLANDING"
;

/*
 H rc_rates:145,145,120
 H rc_expo:20,20,0
 H rates:75,75,70
 H rollPID:44,40,30
 H pitchPID:58,50,35
 H yawPID:70,45,20

//H altPID:50,0,0
//H posPID:15,0,0
//H posrPID:34,14,53
//H navrPID:25,33,83
H levelPID:50,50,75
//H magPID:40
//H velPID:55,55,75
*/
        BLACKBOX_PRINT_HEADER_LINE("rc_rates", "%d,%d,%d",                  rates.rcRates[RadioController::ROLL],
                                                                            rates.rcRates[RadioController::PITCH],
                                                                            rates.rcRates[RadioController::YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rc_expo", "%d,%d,%d",                   rates.rcExpos[RadioController::ROLL],
                                                                            rates.rcExpos[RadioController::PITCH],
                                                                            rates.rcExpos[RadioController::YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rates", "%d,%d,%d",                     rates.rates[RadioController::ROLL],
                                                                            rates.rates[RadioController::PITCH],
                                                                            rates.rates[RadioController::YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rate_limits", "%d,%d,%d",               rates.rateLimits[RadioController::ROLL],
                                                                            rates.rateLimits[RadioController::PITCH],
                                                                            rates.rateLimits[RadioController::YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rollPID", "%d,%d,%d",                   _flightController.getPID_MSP(FlightController::ROLL_RATE_DPS).kp,
                                                                            _flightController.getPID_MSP(FlightController::ROLL_RATE_DPS).ki,
                                                                            _flightController.getPID_MSP(FlightController::ROLL_RATE_DPS).kd);
        BLACKBOX_PRINT_HEADER_LINE("pitchPID", "%d,%d,%d",                  _flightController.getPID_MSP(FlightController::PITCH_RATE_DPS).kp,
                                                                            _flightController.getPID_MSP(FlightController::PITCH_RATE_DPS).ki,
                                                                            _flightController.getPID_MSP(FlightController::PITCH_RATE_DPS).kd);
        BLACKBOX_PRINT_HEADER_LINE("yawPID", "%d,%d,%d",                    _flightController.getPID_MSP(FlightController::YAW_RATE_DPS).kp,
                                                                            _flightController.getPID_MSP(FlightController::YAW_RATE_DPS).ki,
                                                                            _flightController.getPID_MSP(FlightController::YAW_RATE_DPS).kd);
        BLACKBOX_PRINT_HEADER_LINE("rollAnglePID", "%d,%d,%d",              _flightController.getPID_MSP(FlightController::ROLL_ANGLE_DEGREES).kp,
                                                                            _flightController.getPID_MSP(FlightController::ROLL_ANGLE_DEGREES).ki,
                                                                            _flightController.getPID_MSP(FlightController::ROLL_ANGLE_DEGREES).kd);
        BLACKBOX_PRINT_HEADER_LINE("pitchAnglePID", "%d,%d,%d",             _flightController.getPID_MSP(FlightController::PITCH_ANGLE_DEGREES).kp,
                                                                            _flightController.getPID_MSP(FlightController::PITCH_ANGLE_DEGREES).ki,
                                                                            _flightController.getPID_MSP(FlightController::PITCH_ANGLE_DEGREES).kd);
/*
 H dterm_filter_type:0
 H dterm_lpf_hz:100
 H yaw_lpf_hz:0
 H dterm_notch_hz:0
 H dterm_notch_cutoff:160
 H iterm_windup:50
H vbat_pid_gain:0
H pidAtMinThrottle:1
H anti_gravity_threshold:350
H anti_gravity_gain:1000
*/
        BLACKBOX_PRINT_HEADER_LINE("dterm_lpf1_type", "%d",                 fcFiltersConfig.dterm_lpf1_type);
        BLACKBOX_PRINT_HEADER_LINE("dterm_lpf1_static_hz", "%d",            fcFiltersConfig.dterm_lpf1_hz);
        BLACKBOX_PRINT_HEADER_LINE("dterm_lpf2_type", "%d",                 fcFiltersConfig.dterm_lpf2_type);
        BLACKBOX_PRINT_HEADER_LINE("dterm_lpf2_static_hz", "%d",            fcFiltersConfig.dterm_lpf2_hz);
        //BLACKBOX_PRINT_HEADER_LINE("yaw_lowpass_hz", "%d",                  currentPidProfile.yaw_lowpass_hz);
        BLACKBOX_PRINT_HEADER_LINE("dterm_notch_hz", "%d",                  fcFiltersConfig.dterm_notch_hz);
        BLACKBOX_PRINT_HEADER_LINE("dterm_notch_cutoff", "%d",              fcFiltersConfig.dterm_notch_cutoff);
        //BLACKBOX_PRINT_HEADER_LINE("iterm_windup", "%d",                    currentPidProfile.itermWindup);
        //BLACKBOX_PRINT_HEADER_LINE("pid_at_min_throttle", "%d",             currentPidProfile.pidAtMinThrottle);

        // Betaflight PID controller parameters
        BLACKBOX_PRINT_HEADER_LINE("anti_gravity_gain", "%d",               antiGravityConfig.i_gain);
        BLACKBOX_PRINT_HEADER_LINE("anti_gravity_cutoff_hz", "%d",          antiGravityConfig.cutoff_hz);
        BLACKBOX_PRINT_HEADER_LINE("anti_gravity_p_gain", "%d",             antiGravityConfig.p_gain);
#if defined(USE_INTEGRATED_YAW_CONTROL)
        BLACKBOX_PRINT_HEADER_LINE("use_integrated_yaw", "%d",              currentPidProfile.use_integrated_yaw);
#endif
        BLACKBOX_PRINT_HEADER_LINE("ff_weight", "%d,%d,%d",                 _flightController.getPID_MSP(FlightController::ROLL_RATE_DPS).kf,
                                                                            _flightController.getPID_MSP(FlightController::PITCH_RATE_DPS).kf,
                                                                            _flightController.getPID_MSP(FlightController::ROLL_RATE_DPS).kf);


        // End of Betaflight controller parameters
/*
//H setpoint_relaxation_ratio:50
//H dterm_setpoint_weight:100
H acc_limit_yaw:100
H acc_limit:0
H pidsum_limit:500
H pidsum_limit_yaw:400
H deadband:0
H yaw_deadband:0
H gyro_lpf:0
H gyro_lowpass_type:0
H gyro_lowpass_hz:90
H gyro_notch_hz:0,0
H gyro_notch_cutoff:300,100
*/
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf1_type", "%d",                  imuFiltersConfig.gyro_lpf1_type);
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf1_static_hz", "%d",             imuFiltersConfig.gyro_lpf1_hz);
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf2_type", "%d",                  imuFiltersConfig.gyro_lpf2_type);
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf2_static_hz", "%d",             imuFiltersConfig.gyro_lpf2_hz);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_hz", "%d,%d",                0,0);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_cutoff", "%d,%d",            300,100);
/*
H acc_lpf_hz:1000
H acc_hardware:1
//H baro_hardware:1
//H mag_hardware:1
 H gyro_cal_on_first_arm:0
H rc_interpolation:2
H rc_interpolation_interval:19
//H airmode_activate_throttle:32
 H serialrx_provider:3
H use_unsynced_pwm:0
H motor_pwm_protocol:6
H motor_pwm_rate:480
H dshot_idle_value:550
H debug_mode:0
H features:541130760
*/
        BLACKBOX_PRINT_HEADER_LINE("acc_lpf_hz", "%d",                      1000);
        BLACKBOX_PRINT_HEADER_LINE("acc_hardware", "%d",                    1);
        BLACKBOX_PRINT_HEADER_LINE("gyro_cal_on_first_arm", "%d",           0);
        BLACKBOX_PRINT_HEADER_LINE("serialrx_provider", "%d",               SERIALRX_TARGET_CUSTOM); // custom
        BLACKBOX_PRINT_HEADER_LINE("use_unsynced_pwm", "%d",                0);
        BLACKBOX_PRINT_HEADER_LINE("motor_pwm_protocol", "%d",              PWM_TYPE_BRUSHED);
        BLACKBOX_PRINT_HEADER_LINE("motor_pwm_rate", "%d",                  480);
        BLACKBOX_PRINT_HEADER_LINE("motor_idle", "%d",                      550);
        BLACKBOX_PRINT_HEADER_LINE("debug_mode", "%d",                      getDebugMode());
        BLACKBOX_PRINT_HEADER_LINE("features", "%d",                        541130760); //0x2041'0008

        BLACKBOX_PRINT_HEADER_LINE("dyn_idle_min_rpm_100", "%d",            dynamicIdleControllerConfig ? dynamicIdleControllerConfig->dyn_idle_min_rpm_100 : 0);
        BLACKBOX_PRINT_HEADER_LINE("dyn_idle_p_gain", "%d",                 dynamicIdleControllerConfig ? dynamicIdleControllerConfig->dyn_idle_p_gain : 0);
        BLACKBOX_PRINT_HEADER_LINE("dyn_idle_i_gain", "%d",                 dynamicIdleControllerConfig ? dynamicIdleControllerConfig->dyn_idle_i_gain : 0);
        BLACKBOX_PRINT_HEADER_LINE("dyn_idle_d_gain", "%d",                 dynamicIdleControllerConfig ? dynamicIdleControllerConfig->dyn_idle_d_gain : 0);
        BLACKBOX_PRINT_HEADER_LINE("dyn_idle_max_increase", "%d",           dynamicIdleControllerConfig ? dynamicIdleControllerConfig->dyn_idle_max_increase : 0);

// NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        default:
            return WRITE_COMPLETE;
    }

    _xmitState.headerIndex++;
    return WRITE_NOT_COMPLETE;
}
