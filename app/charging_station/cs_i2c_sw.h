/****************************************************************************
 * Contest 2026 team 008 - software (bit-bang) I2C transport
 *
 * The TCA9548A multiplexer hangs off two plain GPIO pins instead of the
 * hardware I2C4 bus, which is used by the FT5x06 touchscreen.
 *
 * Wiring (Arduino headers):
 *   SCL -> D4 (PK1)
 *   SDA -> D5 (PA8)
 *
 * Both lines are open drain: writing 0 drives the line low, writing 1
 * releases it so the pull-up wins.  External 4.7k pull-ups to 3.3V are
 * required on the multiplexer side; the internal pull-ups are only a
 * fallback.
 *
 * All downstream traffic goes through cs_i2c_transfer(), which selects the
 * TCA9548A channel and performs the transfer under one mutex.
 ****************************************************************************/

#ifndef __CS_I2C_SW_H
#define __CS_I2C_SW_H

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CS_I2C_FREQ             100000u  /* Bus clock, Hz */
#define CS_TCA9548A_ADDR        0x70u    /* A2/A1/A0 tied to GND */

/* TCA9548A downstream channel map */
#define CS_TCA_CH_SW3538_0      0u
#define CS_TCA_CH_SW3538_1      1u
#define CS_TCA_CH_SW3538_2      2u
#define CS_TCA_CH_SW3538_3      3u
#define CS_TCA_CH_INA_TOTAL     4u       /* INA226: total input current */
#define CS_TCA_CH_INA_IRON      5u       /* INA226: iron heater current */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Create the software I2C master.  Idempotent: later calls return the
 * existing instance.  Returns 0 on success, negative errno on failure. */

int cs_i2c_bus_init(void);

/* True once cs_i2c_bus_init() has succeeded. */

bool cs_i2c_bus_ready(void);

/* Probe the TCA9548A by writing 0 to its control register (which also
 * deselects every channel).  Returns OK on ACK, negative errno otherwise. */

int cs_i2c_mux_probe(void);

/* Select channel on the TCA9548A and run the transfer as one atomic
 * operation.  The channel register is rewritten on every call so a bus
 * glitch, a reset or a second master cannot leave a stale selection.
 *
 * msgs[i].frequency may be left zero; it is filled in with CS_I2C_FREQ. */

int cs_i2c_transfer(uint8_t channel, FAR struct i2c_msg_s *msgs, int count);

#endif /* __CS_I2C_SW_H */
