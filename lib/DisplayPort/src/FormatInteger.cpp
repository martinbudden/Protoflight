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
