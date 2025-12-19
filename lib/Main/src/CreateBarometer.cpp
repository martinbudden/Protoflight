#include "Main.h"
#include <BarometerBMP280.h>


/*!
Statically allocate the Barometer.
*/
BarometerBase* Main::createBarometer()
{
#if defined(USE_BAROMETER)
#if defined(USE_BAROMETER_BMP280)
    static BarometerBMP280 bmp280(BUS_BASE::BAROMETER_I2C_INDEX , BUS_I2C::BAROMETER_I2C_PINS, BarometerBMP280::I2C_ADDRESS);
    return &bmp280;
#else
    return nullptr;
#endif
#else
    return nullptr;
#endif
}
