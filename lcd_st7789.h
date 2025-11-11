/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-03-23     Vandoul       First version
 */
#ifndef __LCD_ST7789_H__
#define __LCD_ST7789_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "drivers/spi.h"
#include <drv_common.h>
/* 0-0 angle|1-90 angle|2-180 angle|-270 angle */
#define USE_DIRECTION   0

/* lcd size */
#define LCD_W PKG_ST_7789_WIDTH
#define LCD_H PKG_ST_7789_HEIGHT

//#define PKG_ST_7789_DC_PIN 30 GET_PIN(A-, 1)
//#define PKG_ST_7789_RES_PIN 26
//#define PKG_ST_7789_CS_PIN 28
//#define PKG_ST_7789_BLK_PIN 1

#define PKG_ST_7789_DC_PIN  GET_PIN(B, 14)
#define PKG_ST_7789_RES_PIN GET_PIN(B, 10)
#define PKG_ST_7789_CS_PIN  GET_PIN(B, 12)
#define PKG_ST_7789_BLK_PIN GET_PIN(A, 1)


#define LCD_DC_CLR  rt_pin_write(PKG_ST_7789_DC_PIN, PIN_LOW)
#define LCD_DC_SET  rt_pin_write(PKG_ST_7789_DC_PIN, PIN_HIGH)
#define LCD_RES_CLR rt_pin_write(PKG_ST_7789_RES_PIN, PIN_LOW)
#define LCD_RES_SET rt_pin_write(PKG_ST_7789_RES_PIN, PIN_HIGH)
#define LCD_CS_CLR rt_pin_write(PKG_ST_7789_CS_PIN, PIN_LOW)
#define LCD_CS_SET rt_pin_write(PKG_ST_7789_CS_PIN, PIN_HIGH)
#define LCD_BLK_CLR rt_pin_write(PKG_ST_7789_BLK_PIN, PIN_HIGH)
#define DELAY       rt_thread_mdelay

typedef enum
{
    LCD_PORTAIT            = 0,        /* Normal portrait orientation       */
    LCD_PORTAIT_180        = 1,        /* Upside-down portrait orientation  */
    LCD_CROSSWISE          = 2,        /* Normal landscape orientation      */
    LCD_CROSSWISE_180      = 3,        /* Upside-down landscape orientation */
} lcd_dir_enum;

typedef enum
{
    RGB565_WHITE    = (0xFFFF),        /* Color representation for white     */
    RGB565_BLACK    = (0x0000),        /* Color representation for black     */
    RGB565_BLUE     = (0x001F),        /* Color representation for blue      */
    RGB565_PURPLE   = (0xF81F),        /* Color representation for purple    */
    RGB565_PINK     = (0xFE19),        /* Color representation for pink      */
    RGB565_RED      = (0xF800),        /* Color representation for red       */
    RGB565_MAGENTA  = (0xF81F),        /* Color representation for magenta   */
    RGB565_GREEN    = (0x07E0),        /* Color representation for green     */
    RGB565_CYAN     = (0x07FF),        /* Color representation for cyan      */
    RGB565_YELLOW   = (0xFFE0),        /* Color representation for yellow    */
    RGB565_BROWN    = (0xBC40),        /* Color representation for brown     */
    RGB565_GRAY     = (0x8430),        /* Color representation for gray      */
    RGB565_39C5BB   = (0x3616),        /* Custom color representation        */
    RGB565_66CCFF   = (0x665F),        /* Custom color representation        */
} rgb565_color_enum;

void lcd_clear(uint16_t color);
void lcd_region_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t color);
#ifdef __cplusplus
}
#endif
#endif
