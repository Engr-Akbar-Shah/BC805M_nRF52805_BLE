#ifndef GLEAM_LEDSTRIP_H_
#define GLEAM_LEDSTRIP_H_

#include <zephyr/types.h>

/**
 * @brief Initialize the LED strip driver.
 *
 * Must be called once at startup before using update_led_strip().
 *
 * @return 0 on success, negative errno on failure.
 */
int gleam_ledstrip_init(void);

/**
 * @brief Set all pixels on the strip to the same RGB color.
 *
 * @param r Red   (0–255)
 * @param g Green (0–255)
 * @param b Blue  (0–255)
 *
 * @return 0 on success, negative errno on failure.
 */
int gleam_ledstrip_set_rgb(uint8_t r, uint8_t g, uint8_t b);

#endif /* GLEAM_LEDSTRIP_H_ */