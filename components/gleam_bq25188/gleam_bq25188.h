#ifndef GLEAM_BQ25188_H
#define GLEAM_BQ25188_H

#include <stdbool.h>

extern volatile bool bq25188_intr_occur;


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



/* ============================================================
 * STAT0 REGISTER (0x00) - REAL-TIME STATUS (READ ONLY)
 * ============================================================
 * Bit7 : TS Open Status (1 = TS pin open/disconnected)
 * Bit6-5: Charge Status (00 = Not charging, 01 = Pre-charge, 10 = Fast charge, 11 = Charge complete)
 * Bit4 : Input Current Limit Active (1 = ILIM regulation active)
 * Bit3 : VINDPM Active (1 = Input voltage DPM regulation active)
 * Bit2 : VDPPM Active (1 = Battery voltage DPM regulation active)
 * Bit1 : Thermal Regulation Active (1 = Die temperature regulation active)
 * Bit0 : VIN Power Good (1 = Input power good)
 */
#define BQ25188_STAT0_TS_OPEN_STAT      (1 << 7)        // 1 = TS open detected
#define BQ25188_STAT0_CHG_STAT_MASK     (3 << 5)        // Charge status field (extract with >> 5)
#define BQ25188_STAT0_ILIM_STAT         (1 << 4)        // 1 = Input current limit active
#define BQ25188_STAT0_VINDPM_STAT       (1 << 3)        // 1 = VINDPM regulation active
#define BQ25188_STAT0_VDPPM_STAT        (1 << 2)        // 1 = VDPPM regulation active
#define BQ25188_STAT0_TREG_STAT         (1 << 1)        // 1 = Thermal regulation active
#define BQ25188_STAT0_PG_STAT           (1 << 0)        // 1 = VIN power good

/* ============================================================
 * STAT1 REGISTER (0x01) - FAULT / SYSTEM STATUS (READ TO CLEAR)
 * ============================================================
 * Bit7 : TS Fault Status (1 = Temperature fault active)
 * Bit6 : Battery Fault Status (1 = Battery OCP or short detected)
 * Bit5 : VIN Overvoltage Status (1 = Input overvoltage active)
 * Bit4 : Thermal Shutdown Status (1 = Die temperature shutdown active)
 * Bit3 : Watchdog Status (1 = Watchdog timer expired)
 * Bit2 : Safety Timer Status (1 = Safety timer expired)
 * Bit1 : Input Current Limit Status (1 = ILIM fault active)
 * Bit0 : Reserved
 */
#define BQ25188_STAT1_TS_STAT           (1 << 7)        // 1 = TS fault active
#define BQ25188_STAT1_BAT_FAULT_STAT    (1 << 6)        // 1 = Battery fault active
#define BQ25188_STAT1_VIN_OVP_STAT      (1 << 5)        // 1 = VIN overvoltage active
#define BQ25188_STAT1_TSHUT_STAT        (1 << 4)        // 1 = Thermal shutdown active
#define BQ25188_STAT1_WD_STAT           (1 << 3)        // 1 = Watchdog timer expired
#define BQ25188_STAT1_SAFETY_TMR_STAT   (1 << 2)        // 1 = Safety timer expired
#define BQ25188_STAT1_ILIM_STAT         (1 << 1)        // 1 = Input current limit fault
#define BQ25188_STAT1_RESERVED          (1 << 0)        // Reserved

/* ============================================================
 * FLAG0 REGISTER (0x02) - LATCHED FAULT FLAGS (READ TO CLEAR)
 * ============================================================
 * Bit7 : TS Fault Flag (1 = Temperature fault latched - out of range or open)
 * Bit6 : Safety Timer Fault Flag (1 = Safety timer expired latched)
 * Bit5 : Watchdog Fault Flag (1 = Watchdog timer expired latched)
 * Bit4 : Battery Fault Flag (1 = Battery OCP or short-circuit latched)
 * Bit3 : VIN Overvoltage Fault Flag (1 = Input overvoltage latched)
 * Bit2 : Thermal Shutdown Fault Flag (1 = Die thermal shutdown latched)
 * Bit1 : Thermal Regulation Fault Flag (1 = Thermal regulation active latched)
 * Bit0 : Input Current Limit Fault Flag (1 = ILIM fault latched)
 */
#define BQ25188_FLAG0_TS_FAULT          (1 << 7)        // 1 = TS fault latched
#define BQ25188_FLAG0_SAFETY_TMR_FAULT  (1 << 6)        // 1 = Safety timer fault latched
#define BQ25188_FLAG0_WD_FAULT          (1 << 5)        // 1 = Watchdog fault latched
#define BQ25188_FLAG0_BAT_FAULT         (1 << 4)        // 1 = Battery fault latched
#define BQ25188_FLAG0_VIN_OVP_FAULT     (1 << 3)        // 1 = VIN overvoltage fault latched
#define BQ25188_FLAG0_TSHUT_FAULT       (1 << 2)        // 1 = Thermal shutdown fault latched
#define BQ25188_FLAG0_TREG_FAULT        (1 << 1)        // 1 = Thermal regulation fault latched
#define BQ25188_FLAG0_ILIM_FAULT        (1 << 0)        // 1 = Input current limit fault latched

/* ============================================================
 * MASK_ID REGISTER (0x0C) - INTERRUPT MASKS & DEVICE ID (READ / WRITE)
 * ============================================================
 * Bits 7-4 : Device ID (read-only, upper nibble identifies BQ25188)
 * Bits 3-0 : Interrupt mask bits (1 = masked / disabled, 0 = enabled)
 *   Bit3 : TS interrupt mask
 *   Bit2 : Timer interrupt mask (watchdog & safety timer)
 *   Bit1 : General fault interrupt mask
 *   Bit0 : Status change interrupt mask
 */
#define BQ25188_MASK_ID_DEVICE_ID_MASK  (0xF << 4)      // Device ID (bits 7-4, read-only)
#define BQ25188_MASK_ID_TS_MASK         (1 << 3)        // 1 = Mask TS faults/events
#define BQ25188_MASK_ID_TIMER_MASK      (1 << 2)        // 1 = Mask timer faults (WD/safety)
#define BQ25188_MASK_ID_FAULT_MASK      (1 << 1)        // 1 = Mask general faults (OVP, etc.)
#define BQ25188_MASK_ID_STAT_MASK       (1 << 0)        // 1 = Mask status change events


int gleam_bq25188_init(void);
void bq25188_dump_all_verbose(void);
void gleam_bq25188_read_interrupt_cause(void);


#endif // GLEAM_BQ25188_H
