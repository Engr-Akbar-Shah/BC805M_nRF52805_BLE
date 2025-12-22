
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

#include "gleam_adc.h"

LOG_MODULE_REGISTER(GLEAM_ADC);

static const struct adc_dt_spec adc_batt = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_ref = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);

static bool initialized = false;

/* ============================================================
 * Init
 * ============================================================ */
int gleam_adc_init(void)
{
    int err;

    if (!device_is_ready(adc_batt.dev) || !device_is_ready(adc_ref.dev))
        return -ENODEV;

    err = adc_channel_setup_dt(&adc_batt);
    if (err)
        return err;

    err = adc_channel_setup_dt(&adc_ref);
    if (err)
        return err;

    initialized = true;
    return 0;
}

/* ============================================================
 * Generic raw read
 * ============================================================ */
int gleam_adc_read_raw_dt(const struct adc_dt_spec *adc, int16_t *raw)
{
    if (!initialized)
        return -EACCES;

    struct adc_sequence seq = {
        .buffer = raw,
        .buffer_size = sizeof(*raw),
    };

    adc_sequence_init_dt(adc, &seq);
    return adc_read(adc->dev, &seq);
}

/* ============================================================
 * Channel-specific raw reads
 * ============================================================ */
int gleam_adc_read_batt_raw(int16_t *raw)
{
    return gleam_adc_read_raw_dt(&adc_batt, raw);
}

int gleam_adc_read_ref_raw(int16_t *raw)
{
    return gleam_adc_read_raw_dt(&adc_ref, raw);
}

/* ============================================================
 * mV conversion
 * ============================================================ */
int gleam_adc_read_batt_mv(int32_t *batt_mv)
{
    int16_t raw;
    int err = gleam_adc_read_batt_raw(&raw);
    if (err)
        return err;

    int32_t mv = raw;

    adc_raw_to_millivolts(adc_ref_internal(adc_batt.dev),
                          ADC_GAIN_1_6,
                          12,
                          &mv);

    /* Undo divider: (402k + 200k) / 200k */
    mv = mv * (402000 + 200000) / 200000;

    *batt_mv = mv;
    return 0;
}

int gleam_adc_read_ref_mv(int32_t *ref_mv)
{
    int16_t raw;
    int err = gleam_adc_read_ref_raw(&raw);
    if (err)
        return err;

    int32_t mv = raw;

    adc_raw_to_millivolts(adc_ref_internal(adc_ref.dev),
                          ADC_GAIN_1_6,
                          12,
                          &mv);

    *ref_mv = mv;
    return 0;
}