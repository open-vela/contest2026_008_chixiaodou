/****************************************************************************
 * Contest 2026 team 008 - SW3538 driver header
 ****************************************************************************/

#ifndef __SW3538_H
#define __SW3538_H

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sw3538_status_s
{
  uint16_t vout_mv;   /* Output voltage in mV */
  uint16_t ic_ma;     /* Type-C port current in mA */
  uint16_t ia_ma;     /* Type-A port current in mA */
  uint8_t  proto;     /* Fast-charge protocol (see sw3538_proto_name) */
  uint8_t  online;    /* bit1: Type-C, bit0: Type-A device present */
  int16_t  temp_c;    /* NTC temperature in degrees C when temp_valid */
  bool     temp_valid;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Bring up the software I2C bus and probe the TCA9548A. */

int  sw3538_init(void);
int  sw3538_read_status(FAR struct sw3538_status_s *st);
int  sw3538_read_status_channel(uint8_t channel,
                                FAR struct sw3538_status_s *st);

/* Presence check on one multiplexer channel, for hot-plug re-probe. */

int  sw3538_probe_channel(uint8_t channel);

const char *sw3538_proto_name(uint8_t proto);

#endif /* __SW3538_H */
