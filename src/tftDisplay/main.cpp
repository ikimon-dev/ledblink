/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"

#include "st7789_lcd.pio.h"

#define PIN_DIN 3
#define PIN_CLK 2
#define PIN_CS 5
#define PIN_DC 7
#define PIN_RESET 6

#define ST7735X_CMD_SWREST 0x01  // ソフトウェアリセット
#define ST7735X_CMD_SLPIN 0x10   // スリープする
#define ST7735X_CMD_SLPOUT 0x11  // スリープからの復帰
#define ST7735X_CMD_INVOFF 0x20  // 反転表示ＯＦＦ
#define ST7735X_CMD_INVON 0x21   // 反転表示ＯＮ
#define ST7735X_CMD_DISPOFF 0x28 // ディスプレイをＯＦＦ
#define ST7735X_CMD_DISPON 0x29  // ディスプレイをＯＮ
#define ST7735X_CMD_CASET 0x2A   // 列アドレスの設定
#define ST7735X_CMD_RASET 0x2B   // 行アドレスの設定
#define ST7735X_CMD_RAMWR 0x2C   // メモリに書き込む
#define ST7735X_CMD_MADCTL 0x36  //
#define ST7735X_CMD_COLMOD 0x3A  // 色の階調を指定する

#define BACK_LIGHT_PIN

static const uint8_t init_seq[] = {
    1, 20, 0x01,                        // Software reset
    1, 10, 0x11,                        // Exit sleep mode
    2, 2, 0x3a, 0x55,                   // Set colour mode to 16 bit
    2, 0, 0x36, 0x00,                   // Set MADCTL: row then column, refresh is bottom to top ????
    5, 0, 0x2a, 0x00, 0x00, 0x00, 0xf0, // CASET: column addresses from 0 to 240 (f0)
    5, 0, 0x2b, 0x00, 0x00, 0x00, 0xf0, // RASET: row addresses from 0 to 240 (f0)
    1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
    1, 2, 0x13,                         // Normal display on, then 10 ms delay
    1, 2, 0x29,                         // Main screen turn on, then wait 500 ms
    0                                   // Terminate list
};

void SetPinDC(bool pin_dc);
void SetPinCS(bool pin_cs);
void SendByte();

static inline void lcd_set_dc_cs(bool dc, bool cs)
{
    sleep_us(1);
    gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
    sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count)
{
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(0, 0);
    st7789_lcd_put(pio, sm, *cmd++);
    if (count >= 2)
    {
        st7789_lcd_wait_idle(pio, sm);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            st7789_lcd_put(pio, sm, *cmd++);
    }
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq)
{
    const uint8_t *cmd = init_seq;
    while (*cmd)
    {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void st7789_start_pixels(PIO pio, uint sm)
{
    uint8_t cmd = 0x2c; // RAMWR
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
}

void z144_TFT_Initialize()
{

    gpio_init(PIN_CS);
    gpio_init(PIN_DC);
    gpio_init(PIN_RESET);
    // gpio_init(PIN_BL);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    // gpio_set_dir(PIN_BL, GPIO_OUT);
}

int main()
{
    stdio_init_all();
    
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        z144_TFT_Initialize();
    }
}
