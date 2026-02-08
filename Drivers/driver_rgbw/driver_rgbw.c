
#include "driver_rgbw.h"

#define DT_DRV_COMPAT custom_ws2812_rgbw_spi

#include <zephyr/drivers/led_strip.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_spi);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/led/led.h>

/* spi-one-frame and spi-zero-frame in DT are for 8-bit frames. */
#define SPI_FRAME_BITS 8

/*
 * SPI master configuration:
 *
 * - mode 0 (the default), 8 bit, MSB first (arbitrary), one-line SPI
 * - no shenanigans (don't hold CS, don't hold the device lock, this
 *   isn't an EEPROM)
 */
#define SPI_OPER(idx) (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | \
                       SPI_WORD_SET(SPI_FRAME_BITS))

struct ws2812_spi_cfg
{
    struct spi_dt_spec bus;
    uint8_t *px_buf;
    uint8_t one_frame;
    uint8_t zero_frame;
    uint8_t num_colors;
    const uint8_t *color_mapping;
    size_t length;
    uint16_t reset_delay;
};

static const struct ws2812_spi_cfg *dev_cfg(const struct device *dev)
{
    return dev->config;
}

/*
 * Serialize an 8-bit color channel value into an equivalent sequence
 * of SPI frames, MSbit first, where a one bit becomes SPI frame
 * one_frame, and zero bit becomes zero_frame.
 */
static inline void ws2812_spi_ser(uint8_t buf[8], uint8_t color,
                                  const uint8_t one_frame, const uint8_t zero_frame)
{
    int i;

    for (i = 0; i < 8; i++)
    {
        buf[i] = color & BIT(7 - i) ? one_frame : zero_frame;
    }
}

/*
 * Latch current color values on strip and reset its state machines.
 */
static inline void ws2812_reset_delay(uint16_t delay)
{
    k_usleep(delay);
}

static int ws2812_strip_update(const struct device *dev,
                               void *pixels,
                               size_t num_pixels,
                               bool is_rgbw)
{
    const struct ws2812_spi_cfg *cfg = dev->config;
    uint8_t *px_buf = cfg->px_buf;
    size_t i, j;
    int rc;

    if (num_pixels > cfg->length)
    {
        return -EINVAL;
    }

    for (i = 0; i < num_pixels; i++)
    {
        uint8_t colors[4] = {0}; // GRBW by default
        if (is_rgbw)
        {
            struct led_rgbw *rgbw = &((struct led_rgbw *)pixels)[i];
            colors[0] = rgbw->g; // Assuming GRBW order; adjust if needed
            colors[1] = rgbw->r;
            colors[2] = rgbw->b;
            colors[3] = rgbw->w;
        }
        else
        {
            struct led_rgb *rgb = &((struct led_rgb *)pixels)[i];
            colors[0] = rgb->g;
            colors[1] = rgb->r;
            colors[2] = rgb->b;
            // w = 0
        }

        for (j = 0; j < cfg->num_colors; j++)
        {
            ws2812_spi_ser(px_buf, colors[j], cfg->one_frame, cfg->zero_frame);
            px_buf += 8;
        }
    }

    struct spi_buf buf = {
        .buf = cfg->px_buf,
        .len = (num_pixels * 8 * cfg->num_colors),
    };
    const struct spi_buf_set tx = {
        .buffers = &buf,
        .count = 1};

    rc = spi_write_dt(&cfg->bus, &tx);
    if (rc)
    {
        LOG_ERR("SPI write failed: %d", rc);
    }
    ws2812_reset_delay(cfg->reset_delay);
    return rc;
}

static int ws2812_strip_update_rgb(const struct device *dev,
                                   struct led_rgb *pixels,
                                   size_t num_pixels)
{
    return ws2812_strip_update(dev, pixels, num_pixels, false);
}

static size_t ws2812_strip_length(const struct device *dev)
{
    const struct ws2812_spi_cfg *cfg = dev_cfg(dev);

    return cfg->length;
}

static int ws2812_spi_init(const struct device *dev)
{

    const struct ws2812_spi_cfg *cfg = dev_cfg(dev);

    if (cfg->num_colors != 4)
    {
        LOG_ERR("RGBW requires 4 colors");
        return -EINVAL;
    }

    uint8_t i;

    if (!spi_is_ready_dt(&cfg->bus))
    {
        LOG_ERR("SPI device %s not ready", cfg->bus.bus->name);
        return -ENODEV;
    }

    for (i = 0; i < cfg->num_colors; i++)
    {
        switch (cfg->color_mapping[i])
        {
        case LED_COLOR_ID_WHITE:
        case LED_COLOR_ID_RED:
        case LED_COLOR_ID_GREEN:
        case LED_COLOR_ID_BLUE:
            break;
        default:
            LOG_ERR("%s: invalid channel to color mapping."
                    "Check the color-mapping DT property",
                    dev->name);
            return -EINVAL;
        }
    }

    return 0;
}

int led_strip_update_rgbw(const struct device *dev,
                          struct led_rgbw *pixels,
                          size_t num_pixels)
{
    return ws2812_strip_update(dev, pixels, num_pixels, true);
}

static const struct led_strip_driver_api ws2812_spi_api = {
    .update_rgb = ws2812_strip_update_rgb,
    .update_channels = NULL,
    .length = ws2812_strip_length,
};

#define WS2812_SPI_NUM_PIXELS(idx) \
    (DT_INST_PROP(idx, chain_length))
#define WS2812_SPI_HAS_WHITE(idx) \
    (DT_INST_PROP(idx, has_white_channel) == 1)
#define WS2812_SPI_ONE_FRAME(idx) \
    (DT_INST_PROP(idx, spi_one_frame))
#define WS2812_SPI_ZERO_FRAME(idx) \
    (DT_INST_PROP(idx, spi_zero_frame))
#define WS2812_SPI_BUFSZ(idx) \
    (WS2812_NUM_COLORS(idx) * 8 * WS2812_SPI_NUM_PIXELS(idx))

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)                             \
    static const uint8_t ws2812_spi_##idx##_color_mapping[] = \
        DT_INST_PROP(idx, color_mapping)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

/* Get the latch/reset delay from the "reset-delay" DT property. */
#define WS2812_RESET_DELAY(idx) DT_INST_PROP(idx, reset_delay)

#define WS2812_SPI_DEVICE(idx)                                       \
                                                                     \
    static uint8_t ws2812_spi_##idx##_px_buf[WS2812_SPI_BUFSZ(idx)]; \
                                                                     \
    WS2812_COLOR_MAPPING(idx);                                       \
                                                                     \
    static const struct ws2812_spi_cfg ws2812_spi_##idx##_cfg = {    \
        .bus = SPI_DT_SPEC_INST_GET(idx, SPI_OPER(idx), 0),          \
        .px_buf = ws2812_spi_##idx##_px_buf,                         \
        .one_frame = WS2812_SPI_ONE_FRAME(idx),                      \
        .zero_frame = WS2812_SPI_ZERO_FRAME(idx),                    \
        .num_colors = WS2812_NUM_COLORS(idx),                        \
        .color_mapping = ws2812_spi_##idx##_color_mapping,           \
        .length = DT_INST_PROP(idx, chain_length),                   \
        .reset_delay = WS2812_RESET_DELAY(idx),                      \
    };                                                               \
                                                                     \
    DEVICE_DT_INST_DEFINE(idx,                                       \
                          ws2812_spi_init,                           \
                          NULL,                                      \
                          NULL,                                      \
                          &ws2812_spi_##idx##_cfg,                   \
                          POST_KERNEL,                               \
                          CONFIG_LED_STRIP_INIT_PRIORITY,            \
                          &ws2812_spi_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_SPI_DEVICE)