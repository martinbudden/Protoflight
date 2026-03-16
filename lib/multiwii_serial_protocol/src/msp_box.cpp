#include "msp_box.h"

#include <cstring>
#include <stream_buf_writer.h>

const MspBox::box_t* MspBox::find_box_by_box_id(uint8_t box_id)
{
    for (const box_t& box : BOXES) {
        // cppcheck-suppress useStlAlgorithm
        if (box.id == box_id) {
            return &box;
        }
    }
    return nullptr;
}

const MspBox::box_t* MspBox::find_box_by_permanent_id(uint8_t permanent_id)
{
    for (const box_t& box : BOXES) {
        // cppcheck-suppress useStlAlgorithm
        if (box.permanent_id == permanent_id) {
            return &box;
        }
    }
    return nullptr;
}

bool MspBox::get_active_box_id(uint8_t box_id) const
{
    return _active_box_ids[box_id];
}

void MspBox::set_active_box_id(uint8_t box_id) // NOLINT(readability-convert-member-functions-to-static)
{
    if (find_box_by_box_id(box_id) != nullptr) {
        _active_box_ids.set(box_id);
    }
}

void MspBox::reset_active_box_id(uint8_t box_id)
{
    _active_box_ids.reset(box_id);
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
    size_t box_index = 0;
    const size_t page_start = page * MAX_BOXES_PER_PAGE;
    const size_t page_end = page_start + MAX_BOXES_PER_PAGE;
    for (uint8_t id = 0; id < BOX_COUNT; ++id) {
        if (get_active_box_id(id)) {
            if (box_index >= page_start && box_index < page_end) {
                if (serialize_box_name(dst, find_box_by_box_id(id)) < 0) {
                    // failed to serialize, abort
                    return;
                }
            }
        ++box_index; // count active boxes
        }
    }
}
void MspBox::serialize_box_reply_permanent_id(StreamBufWriter& dst, size_t page) const
{
    size_t box_index = 0;
    const size_t page_start = page * MAX_BOXES_PER_PAGE;
    const size_t page_end = page_start + MAX_BOXES_PER_PAGE;
    for (uint8_t id = 0; id < BOX_COUNT; ++id) {
        if (get_active_box_id(id)) {
            if (box_index >= page_start && box_index < page_end) {
                const box_t* box = find_box_by_box_id(id);
                if (box == nullptr || dst.bytes_remaining() < 1) {
                    // failed to serialize, abort
                    return;
                }
                dst.write_u8(box->permanent_id);
            }
        ++box_index; // count active boxes
        }
    }
}
