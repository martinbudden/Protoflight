#include "CMSX.h"
#include "CMS_Types.h"
#include "Targets.h"
#if defined(USE_VTX)
#include "VTX_Base.h" // test code won't build if VTX_Base included here
#endif

struct data_t {
    int16_t temperature;
    uint16_t frequency;
    uint8_t pitMode;
    uint8_t powerIndex;
    uint8_t band;
    uint8_t channel;
};

static data_t data {};


static const void* menuConfirm(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::menu_t* menu)
{
    // save VTX settings here
    cmsx.saveConfigAndNotify();
    cmsx.menuExit(displayPort, menu);

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

const std::array<const char *, 6> bandNames
{{
    "--------",
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
}};


const std::array<char, 6> bandLetters { '-', 'A', 'B', 'E', 'F', 'R' };
const std::array<const char *, 9> channelNames { "-", "1", "2", "3", "4", "5", "6", "7", "8" };
const std::array<const char * const, 3> pitModeNames { "---", "OFF", "ON " };


static const void* pitModeChange([[maybe_unused]] CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    data.pitMode = 0;
    return nullptr;
}

static const void* bandChange([[maybe_unused]] CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    data.band = 0;
    return nullptr;
}

static const void* channelChange([[maybe_unused]] CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    data.channel = 0;
    return nullptr;
}

static const void* powerChange([[maybe_unused]] CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    data.powerIndex = 0;
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryPitMode     = OSD_TABLE_t  { &data.pitMode, 3 - 1, &pitModeNames[0] };
static auto entryBand        = OSD_TABLE_t  { &data.band, 6-1, &bandNames[0] };
static auto entryChannel     = OSD_TABLE_t  { &data.channel, 9-1, &channelNames[0] };
static auto entryFrequency   = OSD_UINT16_t { &data.frequency, 5600, 5900, 0 };
static auto entryPower       = OSD_TABLE_t  {}; // set up dynamically in menuVTX_OnEnter
static auto entryTemperature = OSD_INT16_t  { &data.temperature, -100, 300, 0 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static std::array<char, 31> statusString = { "- -- ---- ----" };
//                                            m bc ffff tppp
//                                            01234567890123

static const void* menuVTX_OnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    (void)cmsx;
#if defined(USE_VTX)
    VTX_Base* vtx = cmsx.getVTX();
    if (vtx) {
        vtx->getBandAndChannel(data.band, data.channel);
        vtx->getPowerIndex(data.powerIndex);
    }
#if false
    entryBand.val = &data.band;
    entryBand.max = VTX_Base::BAND_COUNT; //vtxTableBandCount;
    entryBand.names = &VTX_Base::BandNames[0];

    entryChannel.val = &data.channel;
    entryChannel.max = VTX_Base::CHANNEL_COUNT;//vtxTableChannelCount;
    entryChannel.names = &VTX_Base::ChannelNames[0];

    entryPower.val = &data.powerIndex;
    //entryPower.max = vtxTablePowerLevels;
    //entryPower.names = vtxTablePowerLabels;
#endif
#endif

    return nullptr;
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
