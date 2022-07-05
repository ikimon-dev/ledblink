/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"

#include "DEV_Config.h"

#define BACK_LIGHT_PIN

int main()
{

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    DEV_Digital_Write(LED_PIN);

    while (true)
    {
        DEV_Digital_Write(LED_PIN,1);

        sleep_ms(250);
        DEV_Digital_Write(LED_PIN,0);
        sleep_ms(250);
    }
}
