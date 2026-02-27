#pragma once

#include <blackbox.h>

struct blackbox_parameter_group_t;


/*!
Class to write out the Blackbox header, written in blackboxWriteSysinfo()
*/
class BlackboxProtoflight : public Blackbox {
public:
    BlackboxProtoflight(uint32_t pidLooptimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice) :
        Blackbox(pidLooptimeUs, callbacks, serialDevice)
        {}
public:
    virtual Blackbox::write_e write_system_information(const blackbox_parameter_group_t& pg) override;
};
