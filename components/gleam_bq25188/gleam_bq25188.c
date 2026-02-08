#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "gleam_bq25188.h"

LOG_MODULE_REGISTER(GLEAM_BQ25188);

#define SHOTSENSE_BUTTON_DT DT_ALIAS(wakebutton)

volatile bool bq25188_intr_occur = false;

static const struct i2c_dt_spec bq25188_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(bq25188));

/* GPIO for INT pin from devicetree */
static const struct gpio_dt_spec bq25188_int_gpio =GPIO_DT_SPEC_GET(DT_ALIAS(bqgpio), gpios);

/* Callback structure for interrupt */
static struct gpio_callback bq25188_int_cb;

static void bq25188_intr_handler(const struct device *port,
                                struct gpio_callback *cb,
                                uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* Option A: Call directly (simple, but I2C read in ISR - acceptable for low-rate interrupts) */
    // gleam_bq25188_read_interrupt_cause();

	bq25188_intr_occur = true;

	/**
	 * Either raise a flag here and then read the registers in a thread
	 * or 
	 *create a work and start it from here to read your intr, 
		*/

    /* Option B: Preferred for production - just set a flag (you implement this) */
    // atomic_set(&interrupt_pending, 1);
    // k_sem_give(&interrupt_sem);  // or raise event / post to queue
}

/* Write a single register */
static inline int gleam_bq25188_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = {reg, val};

	/* Synchronous write via i2c_write_dt() */
	return i2c_write_dt(&bq25188_i2c, buf, sizeof(buf));
}

/* Read multiple bytes starting at a register (write reg, then read) */
static inline int gleam_bq25188_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t r = reg;
	return i2c_write_read_dt(&bq25188_i2c, &r, 1, buf, len);
}

/*Configure the GPIO for interrupt reception change its pin in the overlay*/
int gleam_bq25188_config_gpio(void)
{
	int ret;
	if (!gpio_is_ready_dt(&bq25188_int_gpio)) {
        LOG_ERR("bq25188: INT GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&bq25188_int_gpio, GPIO_INPUT);
    if (ret) {
        LOG_ERR("bq25188: Failed to configure INT GPIO: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&bq25188_int_gpio, GPIO_INT_EDGE_FALLING);  // Falling edge for active-low pulse
    if (ret) {
        LOG_ERR("bq25188: Failed to enable INT interrupt: %d", ret);
        return ret;
    }

    /* Set up callback */
    gpio_init_callback(&bq25188_int_cb, bq25188_intr_handler, BIT(bq25188_int_gpio.pin));
    ret = gpio_add_callback(bq25188_int_gpio.port, &bq25188_int_cb);
    if (ret) {
        LOG_ERR("bq25188: Failed to add INT callback: %d", ret);
        return ret;
    }
    LOG_INF("bq25188: INT GPIO and masks configured for faults/events");
	return ret;
}

/* Init: check that the I2C bus is ready */
int gleam_bq25188_init(void)
{
	int ret;
	uint8_t id;

	/* 1. Check I2C bus */
	if (!device_is_ready(bq25188_i2c.bus))
	{
		LOG_ERR("bq25188: I2C bus not ready");
		return -ENODEV;
	}

	/* 2. Read Device ID (MASK_ID register upper bits) */
	ret = gleam_bq25188_read_regs(BQ25188_REG_MASK_ID, &id, 1);
	if (ret)
	{
		LOG_ERR("bq25188: Failed to read device ID ERROR : (%d)", ret);
		return ret;
	}

	uint8_t device_id = (id >> 4); /* Upper nibble = Device ID */

	LOG_INF("bq25188: Device ID = 0x%02X", device_id);

	/* ----------------------------------------------------
	 * 3. Configure Battery Charge Voltage (VBAT = 4.20V)
	 * VBATREG = (VBAT - 3.5V) / 10mV
	 * (4.20 - 3.50) / 0.01 = 70 -> 0x46
	 * ---------------------------------------------------- */
	ret = gleam_bq25188_write_reg(BQ25188_REG_VBAT_CTRL, 0x46);
	if (ret)
	{
		LOG_ERR("bq25188: Failed to set VBAT (4.2V)");
		return ret;
	}

	/* ----------------------------------------------------
	 * 4. Configure Fast Charge Current (ICHG = 500mA)
	 * Datasheet encoding → typical mid-scale value
	 * ---------------------------------------------------- */
	ret = gleam_bq25188_write_reg(BQ25188_REG_ICHG_CTRL, 0x32);
	if (ret)
	{
		LOG_ERR("bq25188: Failed to set ICHG (500mA)");
		return ret;
	}

	/* ----------------------------------------------------
	 * 5. Configure Input Current Limit (ILIM = 500mA)
	 * ---------------------------------------------------- */
	ret = gleam_bq25188_write_reg(BQ25188_REG_TMR_ILIM, 0x55);
	if (ret)
	{
		LOG_ERR("bq25188: Failed to set ILIM (500mA)");
		return ret;
	}

	/* ----------------------------------------------------
	 * 6. SYS Regulation = 4.5V (default & safest)
	 * ---------------------------------------------------- */
	ret = gleam_bq25188_write_reg(BQ25188_REG_SYS_REG, 0x02);
	if (ret)
	{
		LOG_ERR("bq25188: Failed to set SYS voltage");
		return ret;
	}

	/* ----------------------------------------------------
	 * 7. Enable Charging + Safety Features
	 * ---------------------------------------------------- */
	ret = gleam_bq25188_write_reg(BQ25188_REG_CHARGECTRL0, 0x01);
	if (ret)
	{
		LOG_ERR("bq25188: Failed to enable charging");
		return ret;
	}

	/* ----------------------------------------------------
	 * 8. Enable Watchdog + Safety Timer
	 * ---------------------------------------------------- */
	ret = gleam_bq25188_write_reg(BQ25188_REG_IC_CTRL, 0x01);
	if (ret)
	{
		LOG_ERR("bq25188: Failed to enable watchdog");
		return ret;
	}

	/* ----------------------------------------------------
	 * 8. Enable INTR on Faults and EVENTS
	 * ---------------------------------------------------- */
	uint8_t mask_val = 0;
    ret = gleam_bq25188_read_regs(BQ25188_REG_MASK_ID, &mask_val, 1);
    if (ret) {
        LOG_ERR("Failed to read MASK_ID for interrupt config");
        return ret;
    }

	mask_val &= ~0x0F;  // Clear lower 4 bits → unmask all in MASK_ID

    ret = gleam_bq25188_write_reg(BQ25188_REG_MASK_ID, mask_val);
    if (ret) {
        LOG_ERR("Failed to unmask interrupts in MASK_ID");
        return ret;
    }

    LOG_INF("bq25188: Interrupts unmasked (MASK_ID now 0x%02X)", mask_val);

	gleam_bq25188_config_gpio();

	LOG_INF("bq25188: Initialization complete");
	return 0;
}

void bq25188_dump_all_verbose(void)
{
	uint8_t val;
	int ret;

	LOG_INF("   BQ25188 FULL RAW REGISTER DUMP");

	/* ---------------------------------------------------
	 * 1. FORCE EXIT SHIP MODE
	 * --------------------------------------------------- */
	ret = gleam_bq25188_write_reg(BQ25188_REG_SHIP_RST, 0x80);
	LOG_INF("WRITE SHIP_RST (0x80) ret = %d", ret);
	k_msleep(20);

	/* ---------------------------------------------------
	 * 3. READ SYS_REG IMMEDIATELY TO CHECK SYS MODE
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_SYS_REG, &val, 1);
	LOG_INF("READ SYS_REG ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	if (ret == 0)
	{
		uint8_t sys_mode = (val >> 6) & 0x03;
		LOG_INF("SYS_MODE = %d (00=Normal,01=ForceBAT,10=OffFloat,11=Pulldown)", sys_mode);
	}

	/* ---------------------------------------------------
	 * 4. READ TS_CONTROL
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_TS_CONTROL, &val, 1);
	LOG_INF("READ TS_CONTROL ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 5. READ STAT0
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_STAT0, &val, 1);
	LOG_INF("READ STAT0 ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 6. READ STAT1
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_STAT1, &val, 1);
	LOG_INF("READ STAT1 ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 7. READ FLAG0
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_FLAG0, &val, 1);
	LOG_INF("READ FLAG0 ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 8. READ VBAT_CTRL
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_VBAT_CTRL, &val, 1);
	LOG_INF("READ VBAT_CTRL ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 9. READ ICHG_CTRL
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_ICHG_CTRL, &val, 1);
	LOG_INF("READ ICHG_CTRL ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 10. READ CHARGECTRL0
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_CHARGECTRL0, &val, 1);
	LOG_INF("READ CHARGECTRL0 ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 11. READ CHARGECTRL1
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_CHARGECTRL1, &val, 1);
	LOG_INF("READ CHARGECTRL1 ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 12. READ IC_CTRL
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_IC_CTRL, &val, 1);
	LOG_INF("READ IC_CTRL ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 13. READ TMR_ILIM
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_TMR_ILIM, &val, 1);
	LOG_INF("READ TMR_ILIM ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 14. READ SHIP_RST
	 * --------------------------------------------------- */
	ret = gleam_bq25188_read_regs(BQ25188_REG_SHIP_RST, &val, 1);
	LOG_INF("READ SHIP_RST ret=%d val=0x%02X", ret, val);
	k_msleep(10);

	/* ---------------------------------------------------
	 * 14. READ INTR MASK REG
	 * --------------------------------------------------- */
    ret = gleam_bq25188_read_regs(BQ25188_REG_MASK_ID, &val, 1);
    LOG_INF("READ MASK ret=%d val=0x%02X", ret, val);
    k_msleep(10);

	LOG_INF("   BQ25188 REGISTER DUMP COMPLETE");
}

/**
 * @brief Handle BQ25188 interrupt: read registers, decode and log causes, take actions.
 *
 * This should be called from normal context (thread/main loop) or directly from ISR
 * if latency is acceptable. Reading registers clears latched flags.
 */
void gleam_bq25188_read_interrupt_cause(void)
{
    int ret;
    uint8_t stat0 = 0, stat1 = 0, flag0 = 0;

    ret = gleam_bq25188_read_regs(BQ25188_REG_STAT0, &stat0, 1);
    if (ret) {
        LOG_ERR("Failed to read STAT0 in interrupt handler: %d", ret);
        return;
    }

    ret = gleam_bq25188_read_regs(BQ25188_REG_STAT1, &stat1, 1);
    if (ret) {
        LOG_ERR("Failed to read STAT1 in interrupt handler: %d", ret);
        return;
    }

    ret = gleam_bq25188_read_regs(BQ25188_REG_FLAG0, &flag0, 1);
    if (ret) {
        LOG_ERR("Failed to read FLAG0 in interrupt handler: %d", ret);
        return;
    }

    LOG_INF("BQ25188 interrupt handled: STAT0=0x%02X, STAT1=0x%02X, FLAG0=0x%02X",
            stat0, stat1, flag0);

    /* Decode FLAG0 latched faults */
    if (flag0 & BQ25188_FLAG0_TS_FAULT) {
        LOG_ERR(" → Temperature (TS) fault detected!");
    }
    if (flag0 & BQ25188_FLAG0_SAFETY_TMR_FAULT) {
        LOG_ERR(" → Safety timer fault detected!");
    }
    if (flag0 & BQ25188_FLAG0_WD_FAULT) {
        LOG_WRN(" → Watchdog timer fault detected - resetting WD");
        gleam_bq25188_write_reg(BQ25188_REG_IC_CTRL, 0x01);  // Kick watchdog
    }
    if (flag0 & BQ25188_FLAG0_BAT_FAULT) {
        LOG_ERR(" → Battery fault (OCP/short) detected!");
    }
    if (flag0 & BQ25188_FLAG0_VIN_OVP_FAULT) {
        LOG_ERR(" → VIN overvoltage fault detected!");
    }
    if (flag0 & BQ25188_FLAG0_TSHUT_FAULT) {
        LOG_ERR(" → Thermal shutdown fault detected!");
    }
    if (flag0 & BQ25188_FLAG0_TREG_FAULT) {
        LOG_WRN(" → Thermal regulation active (die hot)!");
    }
    if (flag0 & BQ25188_FLAG0_ILIM_FAULT) {
        LOG_WRN(" → Input current limit fault detected!");
    }

    /* Decode STAT0 real-time status / events */
    if (stat0 & BQ25188_STAT0_TS_OPEN_STAT) {
        LOG_ERR(" → TS open/disconnected detected!");
    }

    uint8_t chg_stat = (stat0 & BQ25188_STAT0_CHG_STAT_MASK) >> 5;
    switch (chg_stat) {
        case 0x00: LOG_INF(" → Charge status: Not charging"); break;
        case 0x01: LOG_INF(" → Charge status: Pre-charge"); break;
        case 0x02: LOG_INF(" → Charge status: Fast charge"); break;
        case 0x03: LOG_INF(" → Charge status: Charge complete / Termination"); break;
        default:   LOG_INF(" → Charge status: Unknown (%d)", chg_stat); break;
    }

    if (stat0 & BQ25188_STAT0_ILIM_STAT) {
        LOG_WRN(" → Input current limit active");
    }
    if (stat0 & BQ25188_STAT0_VINDPM_STAT) {
        LOG_WRN(" → VINDPM regulation active");
    }
    if (stat0 & BQ25188_STAT0_VDPPM_STAT) {
        LOG_WRN(" → VDPPM regulation active");
    }
    if (stat0 & BQ25188_STAT0_TREG_STAT) {
        LOG_WRN(" → Thermal regulation active");
    }
    if (!(stat0 & BQ25188_STAT0_PG_STAT)) {
        LOG_WRN(" → VIN power NOT good");
    }

    /* Decode STAT1 (additional faults/status, often mirrors some FLAG0 but auto-clears) */
    if (stat1 & BQ25188_STAT1_TS_STAT) {
        LOG_ERR(" → TS fault active (real-time)");
    }
    if (stat1 & BQ25188_STAT1_BAT_FAULT_STAT) {
        LOG_ERR(" → Battery fault active (real-time)");
    }
    if (stat1 & BQ25188_STAT1_VIN_OVP_STAT) {
        LOG_ERR(" → VIN OVP active (real-time)");
    }
    if (stat1 & BQ25188_STAT1_TSHUT_STAT) {
        LOG_ERR(" → Thermal shutdown active (real-time)");
    }
    if (stat1 & BQ25188_STAT1_WD_STAT) {
        LOG_WRN(" → Watchdog expired active (real-time)");
    }
    if (stat1 & BQ25188_STAT1_SAFETY_TMR_STAT) {
        LOG_WRN(" → Safety timer expired active (real-time)");
    }
    if (stat1 & BQ25188_STAT1_ILIM_STAT) {
        LOG_WRN(" → Input current limit active (real-time)");
    }

    /* Optional: Add application-specific actions here, e.g. */
    // if (flag0 & (BQ25188_FLAG0_TSHUT_FAULT | BQ25188_FLAG0_SAFETY_TMR_FAULT)) {
    //     /* Critical fault → disable charging */
    //     gleam_bq25188_write_reg(BQ25188_REG_CHARGECTRL0, 0x00);
    //     LOG_ERR(" → Charging disabled due to critical fault");
    // }
}