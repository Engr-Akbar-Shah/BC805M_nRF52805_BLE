#ifndef DRIVER_RGBW_H_
#define DRIVER_RGBW_H_

#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/dt-bindings/led/led.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LED_COLOR_ID_WHITE
#define LED_COLOR_ID_WHITE 3
#endif

struct led_rgbw {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;
};

/* Custom RGBW update function – call this directly */
int led_strip_update_rgbw(const struct device *dev,
                          struct led_rgbw *pixels,
                          size_t num_pixels);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_RGBW_H_ */