/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2025-11-23     ryh       First version
 */
#include "rtthread.h"
#include "lcd_st7789.h"
#include "drv_spi.h"

#define DBG_TAG "st7789"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define PKG_USING_ST7789
#ifdef PKG_USING_ST7789

#define LCD_CMD_COL_ADDR_SET                0x2A
#define LCD_CMD_ROW_ADDR_SET                0x2B
#define LCD_CMD_MEMORY_WRITE                0x2C
/**
 * MADCTL(36H):
 * +---------------------------------------+
 * | B7 | B6 | B5 | B4 | B3 | B2 | B1 | B0 |
 * | MY | MX | MV | ML | RGB| MH |  - |  - |
 * +---------------------------------------+
 * [3] 0=RGB,1=BGR
*/
#define LCD_CMD_MEMORY_DATA_ACCESS_CTRL     0x36
/**
 * COLMOD(3AH):
 * +---------------------------------------+
 * | B7 | B6 | B5 | B4 | B3 | B2 | B1 | B0 |
 * |  0 | D6 | D5 | D4 |  0 | D2 | D1 | D0 |
 * +---------------------------------------+
 * [6:4] RGB interface Color format:
 *      101=65K RGB
 *      110=262K RGB
 * [2:0] Control interface color format:
 *      011=12bit/pixel
 *      101=16bit/pixel
 *      110=18bit/pixel
 *      111=16M truncated
*/
#define LCD_CMD_INTERFACE_PIXEL_FORMAT      0x3A

#define OFFSET_X 0
#define OFFSET_Y 20

typedef struct
{
    struct rt_device parent;
    uint16_t width;   /* LCD width */
    uint16_t height;  /* LCD high */
    uint32_t id;      /* LCD ID */
    uint8_t dir;      /* 0:Vertical | 1:Vertical */
} _lcd_dev;

/* LCD param */
_lcd_dev lcddev = {
    .width = LCD_W,
    .height = LCD_H,
    .dir = LCD_CROSSWISE,
};
static struct rt_spi_device *spi_dev_lcd;
static void LCD_RESET(void)
{
    LCD_RES_CLR;
    rt_thread_mdelay(100);
    LCD_RES_SET;
    rt_thread_mdelay(100);
}

static void lcd_write_command(uint8_t reg)
{
    LCD_DC_CLR;
    rt_spi_send(spi_dev_lcd, &reg, 1);
}

static void lcd_write_8bit_data(uint8_t data)
{
    LCD_DC_SET;
    rt_spi_send(spi_dev_lcd, &data, 1);
}

void lcd_write_8bit_data_array(const uint8_t *data, uint32_t len)
{
    LCD_DC_SET;
    rt_spi_send(spi_dev_lcd, data, len);
}


static void lcd_write_16bit_data(uint16_t data)
{
    LCD_DC_SET;
    uint8_t buf[2] = {data >> 8, data & 0xff};
    rt_spi_send(spi_dev_lcd, &buf, 2);
}

void lcd_write_16bit_data_array(const uint16_t *data, uint32_t len)
{
    LCD_DC_SET;
    rt_spi_send(spi_dev_lcd, data, len * 2);
}

void lcd_set_dir(uint8_t direction)
{
    lcddev.dir = direction;
    lcd_write_command(LCD_CMD_MEMORY_DATA_ACCESS_CTRL);
    switch (direction)
    {
    case 0:
        lcddev.width = LCD_W;
        lcddev.height = LCD_H;
        lcd_write_8bit_data((0 << 3) | (0 << 5) | (0 << 6) | (0 << 7)); /* BGR==0,MV==0,MX==0,MY==0 */
        break;
    case 1:
        lcddev.width = LCD_H;
        lcddev.height = LCD_W;
        lcd_write_8bit_data((0 << 3) | (1 << 5) | (1 << 6) | (0 << 7)); /* BGR==0,MV==1,MX==0,MY==1 */
        break;
    case 2:
        lcddev.width = LCD_W;
        lcddev.height = LCD_H;
        lcd_write_8bit_data((0 << 3) | (0 << 5) | (1 << 6) | (1 << 7)); /* BGR==0,MV==0,MX==1,MY==1 */
        break;
    case 3:
        lcddev.width = LCD_H;
        lcddev.height = LCD_W;
        lcd_write_8bit_data((0 << 3) | (1 << 5) | (0 << 6) | (1 << 7)); /* BGR==0,MV==1,MX==0,MY==1 */
        break;
    default:
        break;
    }
}

void lcd_set_region(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    switch (lcddev.dir)
    {
        case LCD_PORTAIT:
        {
            lcd_write_command(LCD_CMD_COL_ADDR_SET);
            lcd_write_16bit_data(x1 + PKG_ST_7789_X_OFFSET);
            lcd_write_16bit_data(x2 + PKG_ST_7789_X_OFFSET);

            lcd_write_command(LCD_CMD_ROW_ADDR_SET);
            lcd_write_16bit_data(y1 + PKG_ST_7789_Y_OFFSET);
            lcd_write_16bit_data(y2 + PKG_ST_7789_Y_OFFSET);

            lcd_write_command(LCD_CMD_MEMORY_WRITE);
        }break;
        case LCD_PORTAIT_180:
        {
            lcd_write_command(LCD_CMD_COL_ADDR_SET);
            lcd_write_16bit_data(x1 + PKG_ST_7789_X_OFFSET);
            lcd_write_16bit_data(x2 + PKG_ST_7789_X_OFFSET);

            lcd_write_command(LCD_CMD_ROW_ADDR_SET);
            lcd_write_16bit_data(y1 + PKG_ST_7789_Y_OFFSET);
            lcd_write_16bit_data(y2 + PKG_ST_7789_Y_OFFSET);

            lcd_write_command(LCD_CMD_MEMORY_WRITE);
        }break;
        case LCD_CROSSWISE:
        {
            lcd_write_command(LCD_CMD_COL_ADDR_SET);
            lcd_write_16bit_data(x1 + PKG_ST_7789_Y_OFFSET);
            lcd_write_16bit_data(x2 + PKG_ST_7789_Y_OFFSET);

            lcd_write_command(LCD_CMD_ROW_ADDR_SET);
            lcd_write_16bit_data(y1 + PKG_ST_7789_X_OFFSET);
            lcd_write_16bit_data(y2 + PKG_ST_7789_X_OFFSET);

            lcd_write_command(LCD_CMD_MEMORY_WRITE);
        }break;
        case LCD_CROSSWISE_180:
        {
            lcd_write_command(LCD_CMD_COL_ADDR_SET);
            lcd_write_16bit_data(x1 + PKG_ST_7789_Y_OFFSET);
            lcd_write_16bit_data(x2 + PKG_ST_7789_Y_OFFSET);

            lcd_write_command(LCD_CMD_ROW_ADDR_SET);
            lcd_write_16bit_data(y1 + PKG_ST_7789_X_OFFSET);
            lcd_write_16bit_data(y2 + PKG_ST_7789_X_OFFSET);

            lcd_write_command(LCD_CMD_MEMORY_WRITE);
        }break;
    }
}

/* lcd enter the minimum power consumption mode and backlight off. */
void lcd_enter_sleep(void)
{
//    rt_pin_write(LCD_PWR_PIN, PIN_LOW);
    rt_thread_mdelay(5);
    lcd_write_command(0x10);
}

/* lcd turn off sleep mode and backlight on. */
void lcd_exit_sleep(void)
{
//    rt_pin_write(LCD_PWR_PIN, PIN_HIGH);
    rt_thread_mdelay(5);
    lcd_write_command(0x11);
    rt_thread_mdelay(120);
}

static void _st7789_init(void)
{
    lcd_write_command(0x11);
    HAL_Delay(120);
    lcd_write_command(0x36);
    lcd_write_8bit_data(0x00);

    lcd_write_command(0x3A);
    lcd_write_8bit_data(0x05);

    lcd_write_command(0xB2);
    lcd_write_8bit_data(0x0C);
    lcd_write_8bit_data(0x0C);
    lcd_write_8bit_data(0x00);
    lcd_write_8bit_data(0x33);
    lcd_write_8bit_data(0x33);

    lcd_write_command(0xB7);
    lcd_write_8bit_data(0x35);

    lcd_write_command(0xBB);
    lcd_write_8bit_data(0x32); // Vcom=1.35V

    lcd_write_command(0xC2);
    lcd_write_8bit_data(0x01);

    lcd_write_command(0xC3);
    lcd_write_8bit_data(0x15); // GVDD=4.8V

    lcd_write_command(0xC4);
    lcd_write_8bit_data(0x20); // VDV, 0x20:0v

    lcd_write_command(0xC6);
    lcd_write_8bit_data(0x0F); // 0x0F:60Hz

    lcd_write_command(0xD0);
    lcd_write_8bit_data(0xA4);
    lcd_write_8bit_data(0xA1);

    lcd_write_command(0xE0);
    lcd_write_8bit_data(0xD0);
    lcd_write_8bit_data(0x08);
    lcd_write_8bit_data(0x0E);
    lcd_write_8bit_data(0x09);
    lcd_write_8bit_data(0x09);
    lcd_write_8bit_data(0x05);
    lcd_write_8bit_data(0x31);
    lcd_write_8bit_data(0x33);
    lcd_write_8bit_data(0x48);
    lcd_write_8bit_data(0x17);
    lcd_write_8bit_data(0x14);
    lcd_write_8bit_data(0x15);
    lcd_write_8bit_data(0x31);
    lcd_write_8bit_data(0x34);

    lcd_write_command(0xE1);
    lcd_write_8bit_data(0xD0);
    lcd_write_8bit_data(0x08);
    lcd_write_8bit_data(0x0E);
    lcd_write_8bit_data(0x09);
    lcd_write_8bit_data(0x09);
    lcd_write_8bit_data(0x15);
    lcd_write_8bit_data(0x31);
    lcd_write_8bit_data(0x33);
    lcd_write_8bit_data(0x48);
    lcd_write_8bit_data(0x17);
    lcd_write_8bit_data(0x14);
    lcd_write_8bit_data(0x15);
    lcd_write_8bit_data(0x31);
    lcd_write_8bit_data(0x34);
    lcd_write_command(0x21);

    lcd_write_command(0x29);
}

static void Lcd_pin_init(void)
{
    rt_pin_mode(PKG_ST_7789_DC_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(PKG_ST_7789_RES_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(PKG_ST_7789_BLK_PIN, PIN_MODE_OUTPUT);
}

static void lcd_init(void)
{
    Lcd_pin_init();

    LCD_RESET();        /* LCD Hardware Reset */
    rt_thread_mdelay(120);         /* Delay 120ms */
    _st7789_init();    /* IlI9341 init */
    lcd_set_dir(LCD_PORTAIT);
    LCD_BLK_CLR;        /* Open Backlight */

    rt_kprintf("lcd init\n");
}

void lcd_clear(uint16_t color)
{
    uint16_t color_buffer[lcddev.width];
    uint16_t i = 0, j = 0;

    lcd_set_region(0, 0, lcddev.width - 1, lcddev.height - 1);
    for (i = 0; i < lcddev.width; i++)
    {
        color_buffer[i] = color;
    }
    for (j = 0; j < lcddev.height; j++)
    {
        lcd_write_16bit_data_array(color_buffer, lcddev.width);
    }
}

void lcd_region_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t color)
{
    uint16_t height, width;
    uint16_t i = 0, j = 0;

    width = xend - xsta + 1;  /* Calculate fill width */
    height = yend - ysta + 1; /* Calculate fill height */
    rt_kprintf("lcd_region_fill: width: %d, height: %d, color:%x\n", width, height, color);

    uint16_t color_buffer[width];

    lcd_set_region(xsta, ysta, xend, yend); // Set fill area for the LCD

    for (i = 0; i < width; i++)
    {
        color_buffer[i] = color;
    }
    for (j = 0; j < height; j++)
    {
        lcd_write_16bit_data_array(color_buffer, width);
    }
}

rt_err_t spi_lcd_init(uint32_t freq)
{
    rt_err_t res = RT_EOK;

    rt_hw_spi_device_attach(PKG_ST_7789_SPI_BUS_NAME, PKG_ST_7789_SPI_DEVICE_NAME, GPIOB, GPIO_PIN_12);

    spi_dev_lcd = (struct rt_spi_device *)rt_device_find("spi20");
    if (spi_dev_lcd == RT_NULL)
    {
        rt_kprintf("spi20 device not found!\n");
        return -RT_ERROR;
    }

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;  // 注意：改为MODE_0
    cfg.max_hz = freq * 1000 * 1000;

    rt_spi_configure(spi_dev_lcd, &cfg);

    lcd_init();

    lcd_clear(RGB565_WHITE);

    return res;
}

static void lcd_ops_blit_line(const char *pixel, int x, int y, rt_size_t size)
{
//    rt_kprintf("lcd_ops_blit_line\n0");
    lcd_set_region(x, y, x + size, y);
    lcd_write_8bit_data_array((const uint8_t *)pixel, size * 2);
}

struct rt_device_graphic_ops lcd_graphic_ops =
{
    .set_pixel  = RT_NULL,//lcd_ops_set_pixel ,
    .get_pixel  = RT_NULL,//lcd_ops_get_pixel ,
    .draw_hline = RT_NULL,//lcd_ops_draw_hline,
    .draw_vline = RT_NULL,//lcd_ops_draw_vline,
    .blit_line  = lcd_ops_blit_line ,
};

rt_err_t lcd_dev_ops_init(rt_device_t dev)
{
    spi_lcd_init(42);
    return RT_EOK;
}

rt_err_t lcd_dev_ops_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

rt_err_t lcd_dev_ops_close(rt_device_t dev)
{
    return RT_EOK;
}

rt_size_t lcd_dev_ops_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
//    rt_memcpy(buffer, &((uint16_t *)frame_buf)[pos], size * 2);
    return RT_EOK;
}

rt_size_t lcd_dev_ops_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
//    rt_memcpy(&((uint16_t *)frame_buf)[pos], buffer, size * 2);
    return RT_EOK;
}

rt_err_t lcd_dev_ops_control(rt_device_t dev, int cmd, void *args)
{
    switch(cmd)
    {
        case RTGRAPHIC_CTRL_RECT_UPDATE:
        {
            struct rt_device_rect_info *rect_info = args;
            if(rect_info == RT_NULL)
            {
                return -RT_ERROR;
            }
            // 实现矩形区域更新逻辑
            // lcd_update_rect(rect_info->x, rect_info->y, rect_info->width, rect_info->height);
            break;
        }

        case RTGRAPHIC_CTRL_POWERON:
        {
            // 实现LCD上电逻辑
            // lcd_power_on();
            break;
        }

        case RTGRAPHIC_CTRL_POWEROFF:
        {
            // 实现LCD断电逻辑
            // lcd_power_off();
            break;
        }

        case RTGRAPHIC_CTRL_GET_INFO:
        {
            struct rt_device_graphic_info *info = args;
            if(info == RT_NULL)
            {
                return -RT_ERROR;
            }
            info->pixel_format   = RTGRAPHIC_PIXEL_FORMAT_RGB565; /**< graphic format */
            info->bits_per_pixel = 16; /**< bits per pixel */
            info->width          = lcddev.width; /**< width of graphic device */
            info->height         = lcddev.height; /**< height of graphic device */
            // info->framebuffer    = (void *)frame_buf; /**< frame buffer */
            break;
        }

        case RTGRAPHIC_CTRL_SET_MODE:
        {
            // 设置显示模式
            // int mode = *(int *)args;
            // lcd_set_mode(mode);
            break;
        }

        case RTGRAPHIC_CTRL_GET_EXT:
        {
            // 获取扩展信息
            // struct rt_device_graphic_extinfo *ext_info = args;
            // 填充扩展信息
            break;
        }

        default:
            return -RT_EINVAL; // 不支持的命令
    }
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const struct rt_device_ops lcd_dev_ops =
{
    .init    = lcd_dev_ops_init   ,
    .open    = lcd_dev_ops_open   ,
    .close   = lcd_dev_ops_close  ,
    .read    = lcd_dev_ops_read   ,
    .write   = lcd_dev_ops_write  ,
    .control = lcd_dev_ops_control,
};
#endif

static int rt_hw_lcd_init(void)
{
#ifdef RT_USING_DEVICE_OPS
    lcddev.parent.ops = &lcd_dev_ops;
#else
    lcddev.parent.init    = lcd_dev_ops_init   ;
    lcddev.parent.open    = lcd_dev_ops_open   ;
    lcddev.parent.close   = lcd_dev_ops_close  ;
    lcddev.parent.read    = lcd_dev_ops_read   ;
    lcddev.parent.write   = lcd_dev_ops_write  ;
    lcddev.parent.control = lcd_dev_ops_control;
#endif
    lcddev.parent.type = RT_Device_Class_Graphic;
    lcddev.parent.user_data = &lcd_graphic_ops;
    rt_device_register(&lcddev.parent, "lcd", RT_DEVICE_FLAG_RDWR);
    LOG_I("rt_hw_lcd_init");
    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_lcd_init);
#endif

