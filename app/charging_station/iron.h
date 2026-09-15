/****************************************************************************
 * Contest 2026 team 008 - soldering iron module
 *
 * MAX6675 (K-thermocouple, software SPI) + software PWM heater + PID.
 * All GPIO based, no public-tree config changes needed.
 ****************************************************************************/

#ifndef __IRON_H
#define __IRON_H

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Sleep state reported via iron_status_s.sleep_active: the handle is parked
 * in its stand (SLEEP line low) and the heater is held off. */

struct iron_status_s
{
  int   temp_c;       /* Current tip temperature in C (from MAX6675) */
  int   target_c;     /* User target temperature */
  float power_pct;    /* Current PWM output 0..100 */
  bool  heating;      /* Heater enabled */
  bool  sensor_ok;    /* MAX6675 communication / thermocouple OK */
  bool  sleep_active; /* Debounced: iron is resting in the stand */
  int   target_eff_c; /* Effective PID target (equals target_c while awake) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize GPIO pins for MAX6675 + heater + JBC245 SLEEP. Returns 0 on success. */
int iron_init(void);

/* Periodic tick (call every ~100ms): read temp + run PID + update PWM */
void iron_tick(void);

/* Control */
void iron_set_target(int temp_c);
void iron_set_heater(bool on);
void iron_set_pid(float p, float i, float d);

/* Query */
void iron_get_status(FAR struct iron_status_s *st);

#endif /* __IRON_H */
