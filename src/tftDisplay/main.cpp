/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

// TFT  <-> PICO

// CS   <-> PIN_CS      Chip select pin #
// SCK  <-> PIN_CLK     SPI Clock pin #
// SDA  <-> PIN_MOSI    SPI MOSI pin #
// A0   <-> PIN_DC      Data/Command pin #
// RSt  <-> PIN_RESET   Reset pin

#define PIN_MOSI 3
#define PIN_CLK 2
#define PIN_CS 5
#define PIN_DC 7
#define PIN_RESET 6

#define SPI_PORT spi0

#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01 // ソフトウェアリセット
#define ST77XX_SLPIN 0x10   // スリープする
#define ST77XX_SLPOUT 0x11  // スリープからの復帰
#define ST77XX_PTLON 0x12
#define ST77XX_NORON 0x13
#define ST77XX_INVOFF 0x20  // 反転表示ＯＦＦ
#define ST77XX_INVON 0x21   // 反転表示ＯＮ
#define ST77XX_DISPOFF 0x28 // ディスプレイをＯＦＦ
#define ST77XX_DISPON 0x29  // ディスプレイをＯＮ
#define ST77XX_CASET 0x2A   // 列アドレスの設定
#define ST77XX_RASET 0x2B   // 行アドレスの設定
#define ST77XX_RAMWR 0x2C   // メモリに書き込む
#define ST77XX_MADCTL 0x36  //
#define ST77XX_COLMOD 0x3A  // 色の階調を指定する

#define ST_CMD_DELAY 0x80 // special signifier for command lists

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Some ready-made 16-bit ('565') color settings:
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_ORANGE 0xFC00

static const uint8_t InitCMD[] = {
    18,                               // 18 commands in list:
    ST77XX_SWRESET, ST_CMD_DELAY,     //  1: Software reset, no args, w/delay
    50,                               //     50 ms delay
    ST77XX_SLPOUT, ST_CMD_DELAY,      //  2: Out of sleep mode, no args, w/delay
    255,                              //     255 = max (500 ms) delay
    ST77XX_COLMOD, 1 + ST_CMD_DELAY,  //  3: Set color mode, 1 arg + delay:
    0x05,                             //     16-bit color
    10,                               //     10 ms delay
    ST7735_FRMCTR1, 3 + ST_CMD_DELAY, //  4: Frame rate control, 3 args + delay:
    0x00,                             //     fastest refresh
    0x06,                             //     6 lines front porch
    0x03,                             //     3 lines back porch
    10,                               //     10 ms delay
    ST77XX_MADCTL, 1,                 //  5: Mem access ctl (directions), 1 arg:
    0x08,                             //     Row/col addr, bottom-top refresh
    ST7735_DISSET5, 2,                //  6: Display settings #5, 2 args:
    0x15,                             //     1 clk cycle nonoverlap, 2 cycle gate
                                      //     rise, 3 cycle osc equalize
    0x02,                             //     Fix on VTL
    ST7735_INVCTR, 1,                 //  7: Display inversion control, 1 arg:
    0x0,                              //     Line inversion
    ST7735_PWCTR1, 2 + ST_CMD_DELAY,  //  8: Power control, 2 args + delay:
    0x02,                             //     GVDD = 4.7V
    0x70,                             //     1.0uA
    10,                               //     10 ms delay
    ST7735_PWCTR2, 1,                 //  9: Power control, 1 arg, no delay:
    0x05,                             //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3, 2,                 // 10: Power control, 2 args, no delay:
    0x01,                             //     Opamp current small
    0x02,                             //     Boost frequency
    ST7735_VMCTR1, 2 + ST_CMD_DELAY,  // 11: Power control, 2 args + delay:
    0x3C,                             //     VCOMH = 4V
    0x38,                             //     VCOML = -1.1V
    10,                               //     10 ms delay
    ST7735_PWCTR6, 2,                 // 12: Power control, 2 args, no delay:
    0x11, 0x15,
    ST7735_GMCTRP1, 16,     // 13: Gamma Adjustments (pos. polarity), 16 args + delay:
    0x09, 0x16, 0x09, 0x20, //     (Not entirely necessary, but provides
    0x21, 0x1B, 0x13, 0x19, //      accurate colors)
    0x17, 0x15, 0x1E, 0x2B,
    0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1, 16 + ST_CMD_DELAY, // 14: Gamma Adjustments (neg. polarity), 16 args + delay:
    0x0B, 0x14, 0x08, 0x1E,            //     (Not entirely necessary, but provides
    0x22, 0x1D, 0x18, 0x1E,            //      accurate colors)
    0x1B, 0x1A, 0x24, 0x2B,
    0x06, 0x06, 0x02, 0x0F,
    10,                           //     10 ms delay
    ST77XX_CASET, 4,              // 15: Column addr set, 4 args, no delay:
    0x00, 0x02,                   //     XSTART = 2
    0x00, 0x81,                   //     XEND = 129
    ST77XX_RASET, 4,              // 16: Row addr set, 4 args, no delay:
    0x00, 0x02,                   //     XSTART = 1
    0x00, 0x81,                   //     XEND = 160
    ST77XX_NORON, ST_CMD_DELAY,   // 17: Normal display on, no args, w/delay
    10,                           //     10 ms delay
    ST77XX_DISPOFF, ST_CMD_DELAY, // 18: Main screen turn on, no args, delay
    255};                         //     255 = max (500 ms) delay

static const uint8_t Rcmd2green144[] = { // 7735R init, part 2 (green 1.44 tab)
    2,                                   //  2 commands in list:
    ST77XX_CASET, 4,                     //  1: Column addr set, 4 args, no delay:
    0x00, 0x00,                          //     XSTART = 0
    0x00, 0x7F,                          //     XEND = 127
    ST77XX_RASET, 4,                     //  2: Row addr set, 4 args, no delay:
    0x00, 0x00,                          //     XSTART = 0
    0x00, 0x7F};                         //     XEND = 127

static inline void cs_select()
{
    gpio_put(PIN_CS, 0); // Active low
    asm volatile("nop \n nop \n nop \n nop \n nop");
    asm volatile("nop \n nop \n nop \n nop \n nop");
    asm volatile("nop \n nop \n nop \n nop \n nop");
    asm volatile("nop \n nop \n nop \n nop \n nop");
}

static inline void cs_deselect()
{
    asm volatile("nop \n nop \n nop \n nop \n nop");
    asm volatile("nop \n nop \n nop \n nop \n nop");
    asm volatile("nop \n nop \n nop \n nop \n nop");
    asm volatile("nop \n nop \n nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
}

static inline void tft_reset()
{
    sleep_ms(100);
    gpio_put(PIN_RESET, 1);
    sleep_ms(100);
    gpio_put(PIN_RESET, 0);
    sleep_ms(100);
    gpio_put(PIN_RESET, 1);
}

int main()
{
    stdio_init_all();

    // This example will use SPI0 at 0.5MHz.
    spi_init(SPI_PORT, 100 * 1000);

    gpio_set_function(PIN_CLK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_2pins_with_func(PIN_MOSI, PIN_CLK, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_pull_up(PIN_CS);
    gpio_put(PIN_CS, 1);
    // Make the CS pin available to picotool
    // bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));

    gpio_init(PIN_DC);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_pull_up(PIN_DC);
    gpio_put(PIN_DC, 0);
    bi_decl(bi_1pin_with_name(PIN_DC, "SPI DC"));

    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_pull_up(PIN_RESET);
    gpio_put(PIN_RESET, 0);
    bi_decl(bi_1pin_with_name(PIN_CS, "SPI RESET"));

    tft_reset();

    uint8_t addr = 0, numCommands = 0, cmd = 0, numArgs = 0;
    uint16_t ms;

    numCommands = InitCMD[addr++]; // Number of commands to follow

    while (numCommands--)
    {                                // For each command...
        cmd = InitCMD[addr++];       // Read command
        numArgs = InitCMD[addr++];   // Number of args to follow
        ms = numArgs & ST_CMD_DELAY; // If hibit set, delay follows args
        numArgs &= ~ST_CMD_DELAY;    // Mask out delay bit

        uint8_t buf[128] = {0};

        buf[0] = cmd;

        uint8_t i = 0;

        for (i = 0; i < numArgs; i++)
        {
            buf[i + 1] = InitCMD[addr + i];
        }

        cs_select();

        spi_write_blocking(SPI_PORT, buf, numArgs + 1);

        cs_deselect();

        addr += numArgs;

        if (ms)
        {
            ms = InitCMD[addr++]; // Read post-command delay time (ms)

            if (ms == 255)
            {
                ms = 500;
            } // If 255, delay for 500 ms

            sleep_ms(ms);
        }
    }

    addr = 0;
    numCommands = 0;
    cmd = 0;
    numArgs = 0;
    ms = 0;
    numCommands = Rcmd2green144[addr];

    while (numCommands--)
    {                                    // For each command...
        cmd = Rcmd2green144[++addr];     // Read command
        numArgs = Rcmd2green144[++addr]; // Number of args to follow
        ms = numArgs & ST_CMD_DELAY;     // If hibit set, delay follows args
        numArgs &= ~ST_CMD_DELAY;        // Mask out delay bit

        uint8_t buf[128] = {0};

        buf[0] = cmd;

        uint8_t i = 0;

        for (i = 0; i < numArgs; i++)
        {
            buf[i + 1] = Rcmd2green144[addr + i];
        }

        cs_select();

        spi_write_blocking(SPI_PORT, buf, numArgs + 1);

        cs_deselect();

        addr += numArgs;

        if (ms)
        {
            ms = Rcmd2green144[addr++]; // Read post-command delay time (ms)

            if (ms == 255)
            {
                ms = 500;
            } // If 255, delay for 500 ms

            sleep_ms(ms);
        }
    }

    for (;;)
        ;

    return 0;
}
