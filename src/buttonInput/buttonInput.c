/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"

#define BACK_LIGHT_PIN

#define BUTTON_GPIO 9

int main()
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    while (true)
    {

        if (gpio_get(BUTTON_GPIO))
        {
            gpio_put(LED_PIN, 1);
        }
        else
        {
            gpio_put(LED_PIN, 0);
        }
    }
}
