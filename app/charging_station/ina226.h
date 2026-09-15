/****************************************************************************
 * Contest 2026 team 008 - INA226 power monitor header
 *
 * Two INA226 monitors sit behind the TCA9548A:
 *   CH4 -> total power input current   (2.5 mOhm shunt, A1=A0=GND -> 0x40)
 *   CH5 -> soldering iron heater power (5.0 mOhm shunt, A1=A0=VS  -> 0x46)
 *
 * They are addressed through cs_i2c_transfer()'s channel selection.
 ****************************************************************************/

#ifndef __INA226_H
#define __INA226_H

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CS_INA226_TOTAL         0u   /* TCA9548A CH4 */
#define CS_INA226_IRON          1u   /* TCA9548A CH5 */
#define CS_INA226_COUNT         2u

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ina226_sample_s
{
  uint32_t bus_mv;      /* Bus voltage in mV  (1.25 mV/LSB) */
  int32_t  shunt_uv;    /* Shunt drop in uV   (2.5 uV/LSB, signed) */
  int32_t  current_ma;  /* Current in mA      (signed) */
  uint32_t power_mw;    /* Power in mW        */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Probe and configure both monitors.  Missing devices are reported once and
 * skipped; the call still succeeds when the bus itself is usable. */

int  cs_ina226_init(void);

/* Read one monitor.  Returns -ENODEV without touching the bus when the
 * device is known to be absent. */

int  cs_ina226_read(uint8_t dev, FAR struct ina226_sample_s *out);

/* True when the given monitor answered its last access. */

bool cs_ina226_present(uint8_t dev);

/* Re-probe the monitors currently marked absent.  Call a few seconds apart
 * so a missing board costs almost nothing. */

void cs_ina226_retry(void);

#endif /* __INA226_H */
