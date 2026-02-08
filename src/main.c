#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "gleam_ble.h"
#include "gleam_bq25188.h"
#include "gleam_adc.h"
#include "gleam_rgbw.h"


LOG_MODULE_REGISTER(MAIN);

volatile bool ble_ready = false;

#define DELAY_TIME K_MSEC(5)
#define BLINK_SLEEP_TIME_MS 200

int main(void)
{

    int err;

    err = gleam_bq25188_init();
    if (0 != err)
        return 0;

    bq25188_dump_all_verbose();

    err = gleam_adc_init();
    if (0 != err)
        return 0;

    int16_t batt_raw, ref_raw;
    int32_t batt_mv;

    gleam_adc_read_batt_raw(&batt_raw);
    gleam_adc_read_ref_raw(&ref_raw);

    gleam_adc_read_batt_mv(&batt_mv);

    LOG_INF("batt_raw=%d, ref_raw=%d, batt_mv=%d",
            batt_raw, ref_raw, batt_mv);

    if (!gpio_is_ready_dt(&dcdc))
    {
        return 0;
    }
    gpio_pin_configure_dt(&dcdc, GPIO_OUTPUT_ACTIVE);

    gpio_pin_configure_dt(&ledb, GPIO_OUTPUT_ACTIVE);
    TURNON_DCDC;
    TURNOFF_LEDB;
    // gpio_pin_toggle_dt(&dcdc);

    err = gleam_ble_init_and_adv();
    if (0 != err)
        return 0;

    err = gleam_rgbw_ledstrip_init();
    if (0 != err)
        return 0;

    //     while (1) {
    //     // For now, you can manually update the RGB values
    //     r = 0;  // Example value for red
    //     g = 255;   // Example value for green
    //     b = 0;  // Example value for blue

    //     // Call function to update LED strip with the new RGB values
    //     gleam_ledstrip_set_rgb(r, g, b);

    //     //k_sleep(DELAY_TIME);  // Adjust delay as needed
    // }

    while (1)
    {
        if(true == bq25188_intr_occur)
        {
            gleam_bq25188_read_interrupt_cause();
            bq25188_intr_occur = false;
        }
    }
    

    return 0;
}
