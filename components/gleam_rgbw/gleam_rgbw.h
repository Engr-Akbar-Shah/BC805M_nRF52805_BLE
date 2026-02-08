#ifndef GLEAM_RGBW_H_
#define GLEAM_RGBW_H_

#include <zephyr/types.h>

/**
 * @brief Initialize the RGBW LED strip driver
 *
 * @return 0 on success, negative errno on failure
 */
int gleam_rgbw_ledstrip_init(void);

/**
 * @brief Set ALL LEDs to the same RGBW color/value
 *
 * @param r Red   (0–255)
 * @param g Green (0–255)
 * @param b Blue  (0–255)
 * @param w White (0–255)
 *
 * @return 0 on success, negative error code on failure
 */
int gleam_rgbw_ledstrip_set(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

#endif /* GLEAM_RGBW_H_ */