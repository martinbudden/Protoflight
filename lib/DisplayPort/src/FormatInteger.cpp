#include "FormatInteger.h"


void ui2a(unsigned int num, char* bf)
{
    const unsigned int base = 10;
    unsigned int d = 1;
    while (num / d >= base) {
        d *= base;
    }
    while (d != 0) {
        const unsigned int dgt = num / d;
        *bf++ = static_cast<char>(dgt + '0'); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        // Next digit
        num %= d;
        d /= base;
    }
    *bf = 0;
}

void i2a(int num, char* bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-'; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    ui2a(static_cast<unsigned int>(num), bf);
}

/*!
Format integer as "%6.3f".
*/
void formatFixed6point3(int value, char* bf)
{
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    i2a(100000 + value, bf);
    bf[0] = bf[1];
    bf[1] = bf[2];
    bf[2] = '.';

    if (bf[5] == '0' || bf[5] == '.') {
        bf[5] = 0;
        if (bf[4] == '0' || bf[4] == '.') {
            bf[4] = 0;
        }
    }
    if (bf[0] == '0') {
        bf[0] = ' ';
    }
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}
