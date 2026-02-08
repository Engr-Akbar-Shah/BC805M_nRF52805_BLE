#ifndef GLEAM_BLE_H
#define GLEAM_BLE_H

#include <stddef.h>
#include <zephyr/drivers/gpio.h>

#define LEDB_NODE DT_ALIAS(ledb)
#define DCDC_NODE DT_ALIAS(dcdc)
#define TURNOFF_DCDC gpio_pin_set_dt(&dcdc,1)
#define TURNON_DCDC gpio_pin_set_dt(&dcdc,0)
#define TURNOFF_LEDB gpio_pin_set_dt(&ledb,1)
#define TURNON_LEDB gpio_pin_set_dt(&ledb,0)
static const struct gpio_dt_spec ledb = GPIO_DT_SPEC_GET(LEDB_NODE, gpios);
static const struct gpio_dt_spec dcdc = GPIO_DT_SPEC_GET(DCDC_NODE, gpios);


int gleam_service_init (void);

int gleam_ble_init_and_adv(void);

#endif // GLEAM_BLE_H