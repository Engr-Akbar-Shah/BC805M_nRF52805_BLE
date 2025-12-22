#ifndef GLEAM_BQ25188_H
#define GLEAM_BQ25188_H

/* ============================================================
 * BQ25188 I2C DEVICE ADDRESS
 * ============================================================ */
#define BQ25188_I2C_ADDR_7BIT        0x6A    // 7-bit I2C address
#define BQ25188_I2C_ADDR_8BIT        0xD4    // 8-bit I2C address


/* ============================================================
 * 0x00 - STAT0 REGISTER (READ ONLY)
 * ============================================================
 * READ VALUE MEANING:
 * Bit7  : TS Open Status
 * Bit6-5: Charge Status
 * Bit4  : Input Current Limit Active
 * Bit3  : VINDPM Active
 * Bit2  : VDPPM Active
 * Bit1  : Thermal Regulation Active
 * Bit0  : VIN Power Good
 */
#define BQ25188_REG_STAT0            0x00   // R  - Charger real-time status


/* ============================================================
 * 0x01 - STAT1 REGISTER (READ TO CLEAR)
 * ============================================================
 * READ VALUE MEANING:
 * Fault and system status flags that auto-clear after read
 */
#define BQ25188_REG_STAT1            0x01   // RC - Charger & fault status


/* ============================================================
 * 0x02 - FLAG0 REGISTER (READ TO CLEAR)
 * ============================================================
 * READ VALUE MEANING:
 * Latched fault event flags (cleared on read)
 */
#define BQ25188_REG_FLAG0            0x02   // RC - Latched fault flags


/* ============================================================
 * 0x03 - VBAT_CTRL REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * Sets battery regulation voltage (VBATREG)
 * Range: 3.5V to 4.65V in 10mV steps
 *
 * READ VALUE:
 * Returns currently programmed charge voltage
 */
#define BQ25188_REG_VBAT_CTRL        0x03   // R/W - Battery regulation voltage control


/* ============================================================
 * 0x04 - ICHG_CTRL REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * Sets fast charge current ICHG
 * Range: 5mA to 1000mA
 *
 * READ VALUE:
 * Returns programmed fast charge current
 */
#define BQ25188_REG_ICHG_CTRL        0x04   // R/W - Fast charge current control


/* ============================================================
 * 0x05 - CHARGECTRL0 REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * - Charge enable/disable
 * - Termination enable
 * - Recharge enable
 *
 * READ VALUE:
 * Current charge control configuration
 */
#define BQ25188_REG_CHARGECTRL0      0x05   // R/W - Main charging enable/config control


/* ============================================================
 * 0x06 - CHARGECTRL1 REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * - Battery UVLO threshold
 * - Recharge hysteresis
 * - Termination threshold selection
 *
 * READ VALUE:
 * Returns UVLO & recharge configuration
 */
#define BQ25188_REG_CHARGECTRL1      0x06   // R/W - Battery UVLO, termination & recharge control


/* ============================================================
 * 0x07 - IC_CTRL REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * - Watchdog enable/disable
 * - Software reset
 * - Safety timer scaling
 *
 * READ VALUE:
 * Returns IC logic configuration
 */
#define BQ25188_REG_IC_CTRL          0x07   // R/W - Watchdog & IC core control


/* ============================================================
 * 0x08 - TMR_ILIM REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * - Charge safety timer duration
 * - Precharge timer
 * - Input current limit (ILIM)
 *
 * READ VALUE:
 * Returns configured timers and current limit
 */
#define BQ25188_REG_TMR_ILIM         0x08   // R/W - Safety timers & input current limit


/* ============================================================
 * 0x09 - SHIP_RST REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * - Ship mode enter/exit
 * - Push button parameters
 * - Hardware reset control
 *
 * READ VALUE:
 * Returns ship mode & reset configuration
 */
#define BQ25188_REG_SHIP_RST         0x09   // R/W - Ship mode, reset & pushbutton control


/* ============================================================
 * 0x0A - SYS_REG REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * SYS voltage regulation target:
 *  - Battery tracking
 *  - Fixed 4.4V – 5.5V
 *
 * READ VALUE:
 * Returns current SYS regulation mode
 */
#define BQ25188_REG_SYS_REG          0x0A   // R/W - System voltage regulation control


/* ============================================================
 * 0x0B - TS_CONTROL REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * - JEITA thermal thresholds
 * - TS warm & cool behavior
 * - TS voltage scaling
 *
 * READ VALUE:
 * Returns temperature control configuration
 */
#define BQ25188_REG_TS_CONTROL       0x0B   // R/W - NTC / JEITA temperature control


/* ============================================================
 * 0x0C - MASK_ID REGISTER (READ / WRITE)
 * ============================================================
 * WRITE VALUE:
 * - Interrupt mask for all fault events
 *
 * READ VALUE:
 * - Interrupt mask states
 * - Device ID bits
 */
#define BQ25188_REG_MASK_ID          0x0C   // R/W - Interrupt mask & Device ID


/* ============================================================
 * RESERVED REGISTERS
 * ============================================================
 * 0x0D and above are RESERVED
 * Reading returns 0xFF
 * Writing is FORBIDDEN and may lock the IC
 */


int gleam_bq25188_init(void);
void bq25188_dump_all_verbose(void);

#endif // GLEAM_BQ25188_H
