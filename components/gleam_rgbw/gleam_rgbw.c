#include <stdio.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "driver_rgbw.h"
#include "gleam_rgbw.h"

LOG_MODULE_REGISTER(GLEAM_RGBW);

#define STRIP_NODE DT_CHOSEN(zephyr_led_strip)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgbw pixel_buffer[STRIP_NUM_PIXELS];

int gleam_rgbw_ledstrip_init(void)
{
    if (!device_is_ready(strip))
    {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }
    LOG_INF("LED strip initialized, pixels: %d", STRIP_NUM_PIXELS);
    return 0;
}


int gleam_rgbw_ledstrip_set(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    if (!device_is_ready(strip))
    {
        LOG_ERR("Strip not ready during set_all");
        return -ENODEV;
    }

    /* Fill buffer once — all pixels get the same color */
    for (size_t i = 0; i < STRIP_NUM_PIXELS; i++)
    {
        pixel_buffer[i].r = r;
        pixel_buffer[i].g = g;
        pixel_buffer[i].b = b;
        pixel_buffer[i].w = w;
    }

    int err = led_strip_update_rgbw(strip, pixel_buffer, STRIP_NUM_PIXELS);

    if (err)
    {
        LOG_ERR("led_strip_update_rgbw failed: %d", err);
    }

    return err;
}