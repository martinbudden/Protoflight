#include "Main.h"
#include <barometer_bmp280.h>


/*!
Statically allocate the Barometer.
*/
BarometerBase* Main::createBarometer()
{
#if defined(USE_BAROMETER)
#if defined(USE_BAROMETER_BMP280)
    static BarometerBmp280 bmp280(BusBase::BAROMETER_I2C_INDEX , BusI2c::BAROMETER_I2C_PINS, BarometerBmp280::I2C_ADDRESS);
    return &bmp280;
#else
    return nullptr;
#endif
#else
    return nullptr;
#endif
}
