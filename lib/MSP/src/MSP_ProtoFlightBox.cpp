/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


#include "MSP_ProtoFlightBox.h"

#include <FlightController.h>

/*!
return state of given boxId box, handling ARM and FLIGHT_MODE
*/
bool MSP_ProtoFlightBox::getBoxIdState(const FlightController& flightController, boxId_e boxId)
{
    static const uint8_t boxIdToFlightModeMap[BOX_ID_FLIGHTMODE_LAST+1] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
        [BOX_ARM]        = 0, // not used
        [BOX_ANGLE]      = FlightController::LOG2_ANGLE_MODE,
        [BOX_HORIZON]    = FlightController::LOG2_HORIZON_MODE,
        [BOX_MAG]        = FlightController::LOG2_MAG_MODE,
        [BOX_ALTHOLD]    = FlightController::LOG2_ALT_HOLD_MODE,
        [BOX_HEADFREE]   = FlightController::LOG2_HEADFREE_MODE,
        [BOX_PASSTHRU]   = FlightController::LOG2_PASSTHRU_MODE,
        [BOX_FAILSAFE]   = FlightController::LOG2_FAILSAFE_MODE,
        [BOX_GPS_RESCUE] = FlightController::LOG2_GPS_RESCUE_MODE
    };
    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode

    if (boxId == BOX_ARM) {
        return flightController.isArmingFlagSet(FlightController::ARMED);
    }
    if (boxId <= BOX_ID_FLIGHTMODE_LAST) {
        return flightController.isFlightModeFlagSet(static_cast<FlightController::flight_mode_flag_e>(1U << boxIdToFlightModeMap[boxId])); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    return flightController.isRcModeActive(static_cast<boxId_e>(boxId));
}

/*!
pack used flightModeFlags into supplied bitset
returns number of bits used
*/
size_t MSP_ProtoFlightBox::packFlightModeFlags(std::bitset<BOX_COUNT>& flightModeFlags, const FlightController& flightController)
{
    // Serialize the flags in the order we delivered them, ignoring BOX NAMES and BOX INDEXES
    flightModeFlags.reset();
    // map boxId_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    size_t mspBoxIdx = 0;    // index of active boxId (matches sent permanentId and boxNames)
    for (int boxId = 0; boxId < BOX_COUNT; ++boxId) {
        if (getActiveBoxId(static_cast<boxId_e>(boxId))) {
            if (getBoxIdState(flightController, static_cast<boxId_e>(boxId))) {
                flightModeFlags.set(mspBoxIdx); // box is enabled
            }
            ++mspBoxIdx; // box is active, count it
        }
    }
    // return count of used bits
    return mspBoxIdx;
}
