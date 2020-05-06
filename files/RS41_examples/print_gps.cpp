/*
 * GPS messages reception example
 * Copyright (C) 2020  Silvano Seva silseva@fastwebnet.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdio>
#include "miosix.h"
#include "drivers/serial_stm32.h"
#include "interfaces-impl/hwmapping.h"

using namespace std;
using namespace miosix;

int main()
{
    // Put GPS reset line to high impedance to make it running
    gps::nReset::mode(Mode::INPUT);

    // Serial driver object for USART1
    STM32Serial sgps(1,9600);

    // Pick one character at a time and print it to serial console
    char input;
    while(1)
    {
        ssize_t n = sgps.readBlock(&input, 1, 0);
        if(n > 0)
        {
            iprintf("%c", input);
        }
    }

    return 0;
}
