#include "cms_types.h"
#include "cmsx.h"
#include "vtx.h" // previously had problem with test code not building if VTX included here

struct data_t {
    int16_t temperature;
    uint16_t frequency;
    uint8_t pit_mode;
    uint8_t power_index;
    uint8_t band;
    uint8_t channel;
};

static data_t data {};


static const void* menuConfirm(CMSX& cmsx, cms_context_t& ctx, const CMSX::menu_t* menu)
{
    // save VTX settings here
    cmsx.save_config_and_notify(ctx);
    cmsx.menu_exit(ctx, menu);

    return CMSX::MENU_BACK;
}

static const std::array<CMSX::osd_entry_t, 4> menu_vtx_confirmEntries
{{
    { "CONFIRM", OME_LABEL, nullptr,        nullptr },
    { "YES",     OME_EXIT,  menuConfirm,    CMSX::MENU_EXIT },
    { "NO",      OME_BACK,  nullptr,        nullptr },
    { nullptr,   OME_END,   nullptr,        nullptr }
}};

static CMSX::menu_t menu_vtx_confirm = {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_vtx_confirmEntries[0],
};

// Note VTX band is 1-based rather than zero-based: !!TODO:Need to look at VTX::BAND_COUNT being 1-based
enum { VTX_BAND_COUNT = VTX::BAND_COUNT + 1 };

const std::array<const char *, VTX_BAND_COUNT> band_names
{{
    "--------",
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
}};

const std::array<char, VTX_BAND_COUNT> band_letters { '-', 'A', 'B', 'E', 'F', 'R' };

enum { VTX_CHANNEL_COUNT = VTX::CHANNEL_COUNT + 1 };
const std::array<const char *, VTX_CHANNEL_COUNT> channelNames { "-", "1", "2", "3", "4", "5", "6", "7", "8" };

enum { VTX_PIT_MODE_COUNT = 3 };
const std::array<const char * const, VTX_PIT_MODE_COUNT> pit_modeNames { "---", "OFF", "ON " };


static const void* pit_modeChange([[maybe_unused]] CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)ctx;
    data.pit_mode = 0;
    return nullptr;
}

static const void* bandChange([[maybe_unused]] CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)ctx;
    data.band = 0;
    return nullptr;
}

static const void* channelChange([[maybe_unused]] CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)ctx;
    data.channel = 0;
    return nullptr;
}

static const void* powerChange([[maybe_unused]] CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)ctx;
    data.power_index = 0;
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryPitMode     = osd_table_t  { &data.pit_mode, VTX_PIT_MODE_COUNT - 1, &pit_modeNames[0] };
static auto entryBand        = osd_table_t  { &data.band, VTX_BAND_COUNT - 1, &band_names[0] };
static auto entryChannel     = osd_table_t  { &data.channel, VTX_CHANNEL_COUNT - 1, &channelNames[0] };
static auto entryFrequency   = osd_uint16_t { &data.frequency, 5600, 5900, 0 };
static auto entryPower       = osd_table_t  {}; // set up dynamically in menu_vtx_OnEnter
static auto entryTemperature = osd_int16_t  { &data.temperature, -100, 300, 0 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static std::array<char, 31> statusString = { "- -- ---- ----" };
//                                            m bc ffff tppp
//                                            01234567890123

static const void* menu_vtx_OnEnter([[maybe_unused]] CMSX& cmsx, cms_context_t& ctx) // cppcheck-suppress constParameterCallback
{
    VTX* vtx = ctx.vtx;
    if (vtx) {
        vtx->get_band_and_channel(data.band, data.channel);
        vtx->get_power_index(data.power_index);
    }
#if false
    entryBand.val = &data.band;
    entryBand.max = VTX::BAND_COUNT; //vtxTableBandCount;
    entryBand.names = &VTX::BAND_NAMES[0];

    entryChannel.val = &data.channel;
    entryChannel.max = VTX::CHANNEL_COUNT;//vtxTableChannelCount;
    entryChannel.names = &VTX::CHANNEL_NAMES[0];

    entryPower.val = &data.power_index;
    //entryPower.max = vtxTable_power_levels;
    //entryPower.names = vtxTablePowerLabels;
#endif

    return vtx;
}


CMSX::menu_t menuConfig = {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = nullptr
};

// RTC6705 does not support bands and channels, only frequencies.
static const std::array<CMSX::osd_entry_t, 12> menu_vtx_Entries
{{
    { "- VTX -", OME_LABEL, nullptr, nullptr }, // dynamically changed according to VTX type

    { "",          OME_LABEL | OME_DYNAMIC, nullptr,          &statusString[0] }, // cppcheck-suppress badBitmaskCheck
    { "PIT",       OME_TABLE,               pit_modeChange,    &entryPitMode },      // M R ST
    { "BAND",      OME_TABLE,               bandChange,       &entryBand },         // M R S T
    { "CHAN",      OME_TABLE,               channelChange,    &entryChannel },      // M R S T
    { "(FREQ)",    OME_UINT16 | OME_DYNAMIC,nullptr,          &entryFrequency },    // M S T
    { "POWER",     OME_TABLE,               powerChange,      &entryPower },        // M R S T
    { "TEMP(C)",   OME_INT16 | OME_DYNAMIC, nullptr,          &entryTemperature },  // T
    { "CONFIG",    OME_SUBMENU,             CMSX::menu_change, &menuConfig },        // S
    { "SAVE&EXIT", OME_SUBMENU,             CMSX::menu_change, &menu_vtx_confirm },

    { "BACK",      OME_BACK, nullptr, nullptr },
    { nullptr,     OME_END, nullptr, nullptr }
}};

CMSX::menu_t CMSX::menu_vtx = {
    .on_enter = menu_vtx_OnEnter,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_vtx_Entries[0]
};
