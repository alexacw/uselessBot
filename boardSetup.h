/**
 * @file boardSetup.h
 * @author Alex Au (alex_acw@outlook.com)
 * @brief
 * @version 0.1
 * @date 2018-12-30
 *
 * @copyright Copyright (c) 2018
 *
 */
#ifndef boardSetup_H_
#define boardSetup_H_

#include "CRC.h"
#include "DR16.h"
#include "ch.h"
#include "coreConfig.h"
#include "hal.h"
#include "usbcfg.h"

// SHELL_SD: if this is not defined, means shell will run on SDU1 from
// usbcfg.c
// #define SHELL_SD UARTD2

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Set the up SPI1 driver for the OLED screen
     */
    static inline void setup_SPI1(void)
    {
        palSetLineMode(LINE_OLED_5_SCLK, PAL_MODE_ALTERNATE(5));
        palSetLineMode(LINE_OLED_6_MOSI, PAL_MODE_ALTERNATE(5));

        static const SPIConfig SPI1_Config = {
            false, NULL, (ioline_t)NULL, 0b010 << SPI_CR1_BR_Pos, 0};
        spiStart(&SPID1, &SPI1_Config);
    }

    static inline void setup_SD7(void)
    {
        palSetLineMode(LINE_USART7_RX, PAL_MODE_ALTERNATE(8));
        palSetLineMode(LINE_USART7_TX, PAL_MODE_ALTERNATE(8));

        static const SerialConfig SD7_Config = {
            115200U, 0, USART_CR2_STOP1_BITS, 0};
        sdStart(&SD7, &SD7_Config);
    }

    /**
     * @brief Initializes pin interrupt callbacks and events
     * notice that pins with same number (event with different port)
     * cannot have their interrupt enabled together
     */
    static inline void setup_Pins(void){
        // enable interrupt from the user button (normal high)
        // palEnableLineEvent(LINE_USER_BUTTON,
        // PAL_EVENT_MODE_FALLING_EDGE);
    };

    /**
     * @brief Setup all the peripherals that the modules won't do that
     * for you
     */
    static inline void board_setup_all(void)
    {
        CRC32_init();
        setup_Pins();
        setup_SPI1();
        setup_USB();
        setup_SD7();
    };

#ifdef __cplusplus
}
#endif

#endif  // define boardSetup_H_