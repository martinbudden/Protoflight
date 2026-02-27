#include "MspBox.h"

#include <cstring>
#include <stream_buf_writer.h>

const MspBox::box_t* MspBox::findBoxByBoxId(uint8_t boxId)
{
    for (const box_t& box : boxes) {
        // cppcheck-suppress useStlAlgorithm
        if (box.id == boxId) {
            return &box;
        }
    }
    return nullptr;
}

const MspBox::box_t* MspBox::findBoxByPermanentId(uint8_t permanent_id)
{
    for (const box_t& box : boxes) {
        // cppcheck-suppress useStlAlgorithm
        if (box.permanent_id == permanent_id) {
            return &box;
        }
    }
    return nullptr;
}

bool MspBox::get_active_box_id(uint8_t boxId) const
{
    return _activeBoxIds[boxId];
}

void MspBox::set_active_box_id(uint8_t boxId) // NOLINT(readability-convert-member-functions-to-static)
{
    if (findBoxByBoxId(boxId) != nullptr) {
        _activeBoxIds.set(boxId);
    }
}

void MspBox::reset_active_box_id(uint8_t boxId)
{
    _activeBoxIds.reset(boxId);
}

int MspBox::serialize_box_name(StreamBufWriter& dst, const box_t* box) // box may be nullptr
{
    if (box == nullptr) {
        return -1;
    }
#if defined(LIBRARY_MULTI_WII_SERIAL_PROTOCOL_USE_CUSTOM_BOX_NAMES)
    const char* name = nullptr;
    size_t len {};
    if (box->id >= BOX_USER1 && box->id <= BOX_USER4) {
        const int n = box->id - BOX_USER1;
        name = modeActivationConfig()->box_user_names[n];
        // possibly there is no '\0' in boxname
        if (*name) {
            len = strnlen(name, sizeof(modeActivationConfig()->box_user_names[n]));
        } else {
            name = nullptr;
        }
    }
    if (name == nullptr) {
        name = box->name;
        len = strlen(name);
    }
#else
    const char* name = box->name;
    size_t len = strlen(name);
#endif
    if (dst.bytes_remaining() < len + 1) {
        // boxname or separator won't fit
        return -1;
    }
    dst.write_data(name, len);
    dst.write_u8(';');
    return static_cast<int>(len) + 1;
}

void MspBox::serialize_box_reply_box_name(StreamBufWriter& dst, size_t page) const
{
    size_t boxIndex = 0;
    const size_t pageStart = page * MAX_BOXES_PER_PAGE;
    const size_t pageEnd = pageStart + MAX_BOXES_PER_PAGE;
    for (uint8_t id = 0; id < BOX_COUNT; ++id) {
        if (get_active_box_id(id)) {
            if (boxIndex >= pageStart && boxIndex < pageEnd) {
                if (serialize_box_name(dst, findBoxByBoxId(id)) < 0) {
                    // failed to serialize, abort
                    return;
                }
            }
        ++boxIndex; // count active boxes
        }
    }
}
void MspBox::serialize_box_reply_permanent_id(StreamBufWriter& dst, size_t page) const
{
    size_t boxIndex = 0;
    const size_t pageStart = page * MAX_BOXES_PER_PAGE;
    const size_t pageEnd = pageStart + MAX_BOXES_PER_PAGE;
    for (uint8_t id = 0; id < BOX_COUNT; ++id) {
        if (get_active_box_id(id)) {
            if (boxIndex >= pageStart && boxIndex < pageEnd) {
                const box_t* box = findBoxByBoxId(id);
                if (box == nullptr || dst.bytes_remaining() < 1) {
                    // failed to serialize, abort
                    return;
                }
                dst.write_u8(box->permanent_id);
            }
        ++boxIndex; // count active boxes
        }
    }
}
