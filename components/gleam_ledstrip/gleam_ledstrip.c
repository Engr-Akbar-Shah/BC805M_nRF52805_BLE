#include "gleam_ledstrip.h"

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(GLEAM_LEDSTRIP);

#define STRIP_NODE DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels[STRIP_NUM_PIXELS];

int gleam_ledstrip_init(void)
{
    if (!device_is_ready(strip))
    {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    LOG_INF("LED strip initialized, pixels: %d", STRIP_NUM_PIXELS);
    return 0;
}

int gleam_ledstrip_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < STRIP_NUM_PIXELS; i++)
    {
        pixels[i].r = r;
        pixels[i].g = g;
        pixels[i].b = b;
    }

    int err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
    if (err)
    {
        LOG_ERR("led_strip_update_rgb failed (err %d)", err);
    }

    return err;
}