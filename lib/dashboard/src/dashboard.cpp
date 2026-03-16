#include "buttons_base.h"
#include "dashboard.h"
#include "display_port_base.h"
#include "screen_base.h"


Dashboard::Dashboard(ScreenBase* screen, ButtonsBase* buttons) :
    _screen(screen),
    _buttons(buttons)
{
}

void Dashboard::update_dashboard(uint32_t tick_count, dashboard_context_t& ctx)
{
    if (ctx.display_port.is_grabbed()) {
        return;
    }
    if (tick_count - _screen_tick_count > 101) {
        _screen_tick_count = tick_count;
        if (_screen) {
            _screen->update(ctx.ahrs_message_queue, ctx.motor_mixer, ctx.receiver);
        }
    }
    // update the buttons every 149 ticks (0.15 seconds)
    if (tick_count - _buttons_tick_count > 149) {
        _buttons_tick_count = tick_count;
        if (_buttons) {
            _buttons->update(ctx.flight_controller, ctx.motor_mixer, ctx.receiver, _screen);
        }
    }
}
