#include "CMSX.h"
#include "CMS_Types.h"
#include "VTX.h" // previously had problem with test code not building if VTX included here

struct data_t {
    int16_t temperature;
    uint16_t frequency;
    uint8_t pitMode;
    uint8_t powerIndex;
    uint8_t band;
    uint8_t channel;
};

static data_t data {};


static const void* menuConfirm(CMSX& cmsx, cms_parameter_group_t& pg, const CMSX::menu_t* menu)
{
    // save VTX settings here
    cmsx.saveConfigAndNotify(pg);
    cmsx.menuExit(pg, menu);

    return CMSX::MENU_BACK;
}

static const std::array<CMSX::OSD_Entry, 4> menuVTX_confirmEntries
{{
    { "CONFIRM", OME_LABEL, nullptr,        nullptr },
    { "YES",     OME_EXIT,  menuConfirm,    CMSX::MENU_EXIT },
    { "NO",      OME_BACK,  nullptr,        nullptr },
    { nullptr,   OME_END,   nullptr,        nullptr }
}};

static CMSX::menu_t menuVTX_confirm = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuVTX_confirmEntries[0],
};

// Note VTX band is 1-based rather than zero-based: !!TODO:Need to look at VTX::BAND_COUNT being 1-based
enum { VTX_BAND_COUNT = VTX::BAND_COUNT + 1 };

const std::array<const char *, VTX_BAND_COUNT> bandNames
{{
    "--------",
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
}};

const std::array<char, VTX_BAND_COUNT> bandLetters { '-', 'A', 'B', 'E', 'F', 'R' };

enum { VTX_CHANNEL_COUNT = VTX::CHANNEL_COUNT + 1 };
const std::array<const char *, VTX_CHANNEL_COUNT> channelNames { "-", "1", "2", "3", "4", "5", "6", "7", "8" };

enum { VTX_PIT_MODE_COUNT = 3 };
const std::array<const char * const, VTX_PIT_MODE_COUNT> pitModeNames { "---", "OFF", "ON " };


static const void* pitModeChange([[maybe_unused]] CMSX& cmsx, cms_parameter_group_t& pg, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)pg;
    data.pitMode = 0;
    return nullptr;
}

static const void* bandChange([[maybe_unused]] CMSX& cmsx, cms_parameter_group_t& pg, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)pg;
    data.band = 0;
    return nullptr;
}

static const void* channelChange([[maybe_unused]] CMSX& cmsx, cms_parameter_group_t& pg, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)pg;
    data.channel = 0;
    return nullptr;
}

static const void* powerChange([[maybe_unused]] CMSX& cmsx, cms_parameter_group_t& pg, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)pg;
    data.powerIndex = 0;
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryPitMode     = osd_table_t  { &data.pitMode, VTX_PIT_MODE_COUNT - 1, &pitModeNames[0] };
static auto entryBand        = osd_table_t  { &data.band, VTX_BAND_COUNT - 1, &bandNames[0] };
static auto entryChannel     = osd_table_t  { &data.channel, VTX_CHANNEL_COUNT - 1, &channelNames[0] };
static auto entryFrequency   = osd_uint16_t { &data.frequency, 5600, 5900, 0 };
static auto entryPower       = osd_table_t  {}; // set up dynamically in menuVTX_OnEnter
static auto entryTemperature = osd_int16_t  { &data.temperature, -100, 300, 0 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static std::array<char, 31> statusString = { "- -- ---- ----" };
//                                            m bc ffff tppp
//                                            01234567890123

static const void* menuVTX_OnEnter([[maybe_unused]] CMSX& cmsx, cms_parameter_group_t& pg) // cppcheck-suppress constParameterCallback
{
    VTX* vtx = pg.vtx;
    if (vtx) {
        vtx->getBandAndChannel(data.band, data.channel);
        vtx->getPowerIndex(data.powerIndex);
    }
#if false
    entryBand.val = &data.band;
    entryBand.max = VTX::BAND_COUNT; //vtxTableBandCount;
    entryBand.names = &VTX::BandNames[0];

    entryChannel.val = &data.channel;
    entryChannel.max = VTX::CHANNEL_COUNT;//vtxTableChannelCount;
    entryChannel.names = &VTX::ChannelNames[0];

    entryPower.val = &data.powerIndex;
    //entryPower.max = vtxTablePowerLevels;
    //entryPower.names = vtxTablePowerLabels;
#endif

    return vtx;
}


CMSX::menu_t menuConfig = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = nullptr
};

// RTC6705 does not support bands and channels, only frequencies.
static const std::array<CMSX::OSD_Entry, 12> menuVTX_Entries
{{
    { "- VTX -", OME_LABEL, nullptr, nullptr }, // dynamically changed according to VTX type

    { "",          OME_LABEL | OME_DYNAMIC, nullptr,          &statusString[0] }, // cppcheck-suppress badBitmaskCheck
    { "PIT",       OME_TABLE,               pitModeChange,    &entryPitMode },      // M R ST
    { "BAND",      OME_TABLE,               bandChange,       &entryBand },         // M R S T
    { "CHAN",      OME_TABLE,               channelChange,    &entryChannel },      // M R S T
    { "(FREQ)",    OME_UINT16 | OME_DYNAMIC,nullptr,          &entryFrequency },    // M S T
    { "POWER",     OME_TABLE,               powerChange,      &entryPower },        // M R S T
    { "TEMP(C)",   OME_INT16 | OME_DYNAMIC, nullptr,          &entryTemperature },  // T
    { "CONFIG",    OME_SUBMENU,             CMSX::menuChange, &menuConfig },        // S
    { "SAVE&EXIT", OME_SUBMENU,             CMSX::menuChange, &menuVTX_confirm },

    { "BACK",      OME_BACK, nullptr, nullptr },
    { nullptr,     OME_END, nullptr, nullptr }
}};

CMSX::menu_t CMSX::menuVTX = {
    .onEnter = menuVTX_OnEnter,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuVTX_Entries[0]
};
