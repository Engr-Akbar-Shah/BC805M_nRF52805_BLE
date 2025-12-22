#ifndef _H_GLEAM_ADC
#define _H_GLEAM_ADC

#include <zephyr/kernel.h>

/* Initialization */
int gleam_adc_init(void);

/* Raw reads for individual channels */
int gleam_adc_read_batt_raw(int16_t *raw);
int gleam_adc_read_ref_raw(int16_t *raw);


/* Millivolt conversion helpers */
int gleam_adc_read_batt_mv(int32_t *batt_mv);
int gleam_adc_read_ref_mv(int32_t *ref_mv);

#endif