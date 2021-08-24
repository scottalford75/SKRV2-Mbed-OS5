/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"


// Blinking rate in milliseconds
#define BLINKING_RATE_MS  500

Serial pc(PA_9, PA_10);

// Initialise the digital pin LED1 as an output
DigitalOut mot(PC_13);
DigitalOut led(PB_4);

int main()
{
    pc.printf("SKRV2 is alive");
    mot = 1;

    while (true) {
        led = !led;
        thread_sleep_for(BLINKING_RATE_MS);
    }
}
