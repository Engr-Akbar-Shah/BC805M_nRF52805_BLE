#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "gleam_bq25188.h"

LOG_MODULE_REGISTER(GLEAM_BQ25188);

static const struct i2c_dt_spec bq25188_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(bq25188));

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

	LOG_INF("   BQ25188 REGISTER DUMP COMPLETE");
}
