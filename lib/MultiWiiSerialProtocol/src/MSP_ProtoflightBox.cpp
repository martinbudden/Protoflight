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


#include "MSP_ProtoflightBox.h"

#include "Cockpit.h"

/*!
return state of given boxId box, handling ARM and FLIGHT_MODE
*/
bool MSP_ProtoflightBox::getBoxIdState(const Cockpit& cockpit, MSP_Box::id_e boxId)
{
#if false
    static constexpr std::array<uint8_t, BOX_ID_FLIGHTMODE_LAST + 1> boxIdToFlightModeMap = {
        /*[BOX_ARM]*/           0, // not used
        /*[BOX_ANGLE]*/         Autopilot::LOG2_ANGLE_MODE,
        /*[BOX_HORIZON]*/       Autopilot::LOG2_HORIZON_MODE,
        /*[BOX_MAG]*/           Autopilot::LOG2_MAG_MODE,
        /*[BOX_ALTITUDE_HOLD]*/ Autopilot::LOG2_ALTITUDE_HOLD_MODE,
        /*[BOX_POS_HOLD]*/      Autopilot::LOG2_POSITION_HOLD_MODE,
        /*[BOX_HEADFREE]*/      Autopilot::LOG2_HEADFREE_MODE,
        /*[BOX_CHIRP]*/         Autopilot::LOG2_CHIRP_MODE,
        /*[BOX_PASSTHRU]*/      Autopilot::LOG2_PASSTHRU_MODE,
        /*[BOX_FAILSAFE]*/      Autopilot::LOG2_FAILSAFE_MODE,
        /*[BOX_GPS_RESCUE]*/    Autopilot::LOG2_GPS_RESCUE_MODE
    };
    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode
#endif

    if (boxId == MSP_Box::BOX_ARM) {
        return cockpit.isArmed();
    }
#if false
    if (boxId <= MSP_Box::BOX_ID_FLIGHTMODE_LAST) {
        return cockpit.isFlightModeFlagSet(1U << boxIdToFlightModeMap[boxId]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
#endif
    return cockpit.isRcModeActive(boxId);
}

/*!
pack used flightModeFlags into supplied bitset
returns number of bits used
*/
size_t MSP_ProtoflightBox::packFlightModeFlags(MSP_Box::bitset_t& flightModeFlags, const Cockpit& cockpit)
{
    // Serialize the flags in the order we delivered them, ignoring BOX NAMES and BOX INDEXES
    flightModeFlags.reset();
    // map box_id_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    size_t mspBoxIdx = 0;    // index of active boxId (matches sent permanentId and boxNames)
    for (int boxId = 0; boxId < MSP_Box::BOX_COUNT; ++boxId) {
        if (getActiveBoxId(static_cast<MSP_Box::id_e>(boxId))) {
            if (getBoxIdState(cockpit, static_cast<MSP_Box::id_e>(boxId))) {
                flightModeFlags.set(mspBoxIdx); // box is enabled
            }
            ++mspBoxIdx; // box is active, count it
        }
    }
    // return count of used bits
    return mspBoxIdx;
}
