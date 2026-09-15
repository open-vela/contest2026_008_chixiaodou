/****************************************************************************
 * Contest 2026 team 008 - charging station UI header (unit: display pixels)
 ****************************************************************************/

#ifndef __CHARGING_STATION_UI_H
#define __CHARGING_STATION_UI_H

#include <stdbool.h>
#include <lvgl/lvgl.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  cs_ui_init(lv_obj_t *scr);

/* Data update entry - called periodically to refresh simulated data */
void cs_ui_data_tick(void);

/* Command interfaces (used by AI / local command parsing) */
void cs_ui_set_iron_temp(int temp_c);
int  cs_ui_get_iron_temp(void);
void cs_ui_set_iron_pid(float p, float i, float d);
void cs_ui_get_iron_pid(float *p, float *i, float *d);

/* Real SW3538 data for a selected channel (0..3). */
void cs_ui_set_channel_ports(int channel, float c_volt, float c_amp,
                             float a_volt, float a_amp, const char *proto,
                             int temp_c, bool temp_valid);
void cs_ui_set_channel_online(int channel, bool c_online, bool a_online);

/* Compatibility interfaces for channel 1 (UI index 0). */
void cs_ui_set_ch1_real(float volt, float amp, const char *proto);
void cs_ui_set_ch1_online(bool online);

/* Real SW3538 data for both ports of channel 1.
 * c_amp/a_amp <= 0 means that port is idle.
 */
void cs_ui_set_ch1_ports(float c_volt, float c_amp,
                         float a_volt, float a_amp,
                         const char *proto);

/* Real soldering iron data (from MAX6675 + PID) */
void cs_ui_set_iron_real(int temp_c, float power_pct,
                         bool heating, bool sensor_ok);

/* INA226 measurements.  valid=false falls back to the placeholder text
 * instead of showing a misleading zero. */

void cs_ui_set_input_power(float volt, float amp, float watt, bool valid);
void cs_ui_set_iron_power(float volt, float amp, float watt, bool valid);

/* JBC245 park state: when true the handle rests in its stand (SLEEP line
 * low) and the heater is held off entirely. */
void cs_ui_set_iron_sleep(bool sleep_active, int target_eff);

/* UI -> hardware sync: read what the UI currently wants */
int  cs_ui_get_iron_target(void);
bool cs_ui_get_iron_on(void);

#endif /* __CHARGING_STATION_UI_H */
