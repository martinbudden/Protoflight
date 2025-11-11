#include "OSD.h"
#include "OSD_Elements.h"
#include "OSD_Symbols.h"

#include "FlightController.h"

//
// Drawing functions
//

std::array<OSD_Elements::elementDrawFn, OSD_ITEM_COUNT> OSD_Elements::elementDrawFunctions {};
std::array<OSD_Elements::elementDrawFn, OSD_ITEM_COUNT> OSD_Elements::elementDrawBackgroundFunctions {};


void OSD_Elements::initDrawFunctions()
{
    elementDrawFunctions.fill(nullptr);
    elementDrawBackgroundFunctions.fill(nullptr);

    elementDrawFunctions[OSD_RSSI_VALUE]         = &OSD_Elements::drawRSSI;
    elementDrawFunctions[OSD_MAIN_BATT_VOLTAGE]  = &OSD_Elements::drawMainBatteryVoltage;
    elementDrawFunctions[OSD_CROSSHAIRS]         = &OSD_Elements::drawCrosshairs;
    elementDrawFunctions[OSD_ARTIFICIAL_HORIZON] = &OSD_Elements::drawArtificialHorizon,
    elementDrawFunctions[OSD_ROLL_PIDS]          = &OSD_Elements::drawPIDsRoll;
    elementDrawFunctions[OSD_PITCH_PIDS]         = &OSD_Elements::drawPIDsPitch;
    elementDrawFunctions[OSD_YAW_PIDS]           = &OSD_Elements::drawPIDsYaw;

    elementDrawBackgroundFunctions[OSD_HORIZON_SIDEBARS]        = &OSD_Elements::drawBackgroundHorizonSidebars;
};

void OSD_Elements::formatPID(char * buf, const char * label, uint8_t axis) // NOLINT(readability-non-const-parameter)
{
    const FlightController::PIDF_uint16_t pid = _flightController.getPID_MSP(axis);
    sprintf(buf, "%s %3d %3d %3d %3d %3d", label,
        pid.kp,
        pid.ki,
        pid.kd,
        pid.ks,
        pid.kk
    );
}

void OSD_Elements::drawPIDsRoll(element_t&element)
{
    formatPID(&element.buf[0], "ROL", FlightController::ROLL_RATE_DPS);
}

void OSD_Elements::drawPIDsPitch(element_t& element)
{
    formatPID(&element.buf[0], "PIT", FlightController::PITCH_RATE_DPS);
}

void OSD_Elements::drawPIDsYaw(element_t&element)
{
    formatPID(&element.buf[0], "YAW", FlightController::YAW_RATE_DPS);
}

void OSD_Elements::drawRSSI(element_t&element)
{
    (void)element;
}

void OSD_Elements::drawMainBatteryVoltage(element_t&element)
{
    (void)element;
}

void OSD_Elements::drawCrosshairs(element_t&element)
{
    element.buf[0] = SYM_AH_CENTER_LINE;
    element.buf[1] = SYM_AH_CENTER;
    element.buf[2] = SYM_AH_CENTER_LINE_RIGHT;
    element.buf[3] = 0;
}

void OSD_Elements::drawArtificialHorizon(element_t& element)
{
    enum { AH_SYMBOL_COUNT = 9 };
    enum { AH_SYMBOL_SIDE_COUNT = 4 };
    static int x = -AH_SYMBOL_SIDE_COUNT;
    // Get pitch and roll limits in tenths of degrees
    const float maxPitch = static_cast<float>(_osd.getConfig().ahMaxPitch) * 10.0F;
    const float maxRoll = static_cast<float>(_osd.getConfig().ahMaxRoll) * 10.0F;
    const float ahSign = _osd.getConfig().ahInvert ? -1.0F : 1.0F;
    const float rollAngle = std::clamp(_flightController.getRollAngleDegreesRaw() * ahSign, -maxRoll, maxRoll);
    float pitchAngle = std::clamp(_flightController.getRollAngleDegreesRaw() * ahSign, -maxPitch, maxPitch);
    // Convert pitchAngle to y compensation value
    // (maxPitch / 25) divisor matches previous settings of fixed divisor of 8 and fixed max AHI pitch angle of 20.0 degrees
    if (maxPitch > 0.0F) {
        pitchAngle = ((pitchAngle * 25.0F) / maxPitch);
    }
    pitchAngle -= 4 * AH_SYMBOL_COUNT + 5;

    const int y = ((-static_cast<int>(rollAngle) * x) / 64) - static_cast<int>(pitchAngle);
    if (y >= 0 && y <= 81) {
        element.offsetX = static_cast<uint8_t>(x);
        element.offsetY = static_cast<uint8_t>(y / AH_SYMBOL_COUNT);

        sprintf(&element.buf[0], "%c", (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
    } else {
        element.drawElement = false;  // element does not need to be rendered
    }

    if (x == AH_SYMBOL_SIDE_COUNT) {
        // Rendering is complete, so prepare to start again
        x = -AH_SYMBOL_SIDE_COUNT;
    } else {
        // Rendering not yet complete
        element.rendered = false;
        ++x;
    }
}

void OSD_Elements::drawBackgroundHorizonSidebars(element_t& element)
{
    (void)element;
}