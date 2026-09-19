/****************************************************************************
 * Contest 2026 team 008 - charging station main
 *
 * Entry point: initializes LVGL, display and touchscreen, then
 * enters the LVGL event loop with the charging station UI.
 * Reads the SW3538 chargers and INA226 monitors over the software I2C
 * bus (D4/D5) every ~500ms.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include <lvgl/lvgl.h>

#include "charging_station_ui.h"
#include "cs_i2c_sw.h"
#include "sw3538.h"
#include "ina226.h"
#include "iron.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INA226_RETRY_TICKS  50   /* 50 x 100ms = 5s between re-probes */
#define SW3538_RETRY_TICKS  50   /* 50 x 100ms = 5s between hot-plug probes */
#define SW3538_FAIL_LIMIT   5    /* sweeps failed before a channel is dropped */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_sw3538_ok[4];

/* Consecutive read failures per channel.  A module that is unplugged falls
 * back to the slow presence probe instead of failing on every sweep. */

static uint8_t g_sw3538_fail[4];

/* Channel read by the next tick; the four SW3538 modules are polled in
 * rotation so no single tick carries the whole bus sweep. */

static int  g_poll_chan;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sw3538_poll_channel
 *
 * Read one SW3538 and push its data into the matching UI channel card.
 * On failure, mark the channel idle but keep the app running.
 *
 * One channel takes about 23ms (four ADC conversions, each waiting 5ms for
 * the chip to latch a result), so the four channels are spread across four
 * consecutive ticks rather than read back to back.  A full sweep still
 * completes every 400ms, but no single tick blocks the LVGL loop and the
 * touch input timer for ~92ms.
 ****************************************************************************/

static void sw3538_poll_channel(int channel)
{
  struct sw3538_status_s st;
  int ret;

  if (!g_sw3538_ok[channel])
    {
      return;
    }

  ret = sw3538_read_status_channel(channel, &st);
  if (ret < 0)
    {
      cs_ui_set_channel_online(channel, false, false);

      /* Drop a module that has stopped answering back to the slow presence
       * probe, so a card that was unplugged does not log a failure on every
       * sweep and can be picked up again when it is plugged back in. */

      if (++g_sw3538_fail[channel] >= SW3538_FAIL_LIMIT)
        {
          g_sw3538_fail[channel] = 0;
          g_sw3538_ok[channel] = false;
          syslog(LOG_WARNING, "Charging Station: SW3538 CH%d lost\n",
                 channel + 1);
        }

      return;
    }

  g_sw3538_fail[channel] = 0;

  cs_ui_set_channel_ports(channel, (float)st.vout_mv / 1000.0f,
                          (float)st.ic_ma / 1000.0f,
                          (float)st.vout_mv / 1000.0f,
                          (float)st.ia_ma / 1000.0f,
                          sw3538_proto_name(st.proto), st.temp_c,
                          st.temp_valid);
  cs_ui_set_channel_online(channel, (st.online & 0x02) != 0,
                           (st.online & 0x01) != 0);

  syslog(LOG_INFO, "SW3538 CH%d: Vout=%dmV Ic=%dmA Ia=%dmA "
         "temp=%s%dC proto=%d(%s)\n", channel + 1, st.vout_mv,
         st.ic_ma, st.ia_ma, st.temp_valid ? "" : "N/A ", st.temp_c,
         st.proto, sw3538_proto_name(st.proto));
}

/****************************************************************************
 * Name: sw3538_retry_absent
 *
 * Presence-probe every channel that is not currently up, so a module plugged
 * in after boot is picked up without a reset.  Channel 3 is the one that
 * matters in practice: its card sits over the RJ45 jack, so it is left out
 * unless Ethernet is unused, and it may be fitted later while running.
 ****************************************************************************/

static void sw3538_retry_absent(void)
{
  int channel;

  for (channel = 0; channel < 4; channel++)
    {
      if (g_sw3538_ok[channel])
        {
          continue;
        }

      if (sw3538_probe_channel(channel) == OK)
        {
          g_sw3538_ok[channel] = true;
          g_sw3538_fail[channel] = 0;
          syslog(LOG_INFO, "Charging Station: SW3538 CH%d detected "
                 "(hot-plug)\n", channel + 1);
        }
    }
}

/****************************************************************************
 * Name: ina226_poll
 *
 * Read the total input monitor and the soldering iron monitor.
 *
 * The iron heater is switched by a 0.5 Hz software PWM (2s cycle), so a
 * single 35ms conversion window lands either fully on or fully off; the
 * value is a snapshot rather than an average, and a single low sample is
 * not a fault.
 ****************************************************************************/

static void ina226_poll(void)
{
  struct ina226_sample_s s;

  if (cs_ina226_read(CS_INA226_TOTAL, &s) == OK)
    {
      cs_ui_set_input_power((float)s.bus_mv / 1000.0f,
                            (float)s.current_ma / 1000.0f,
                            (float)s.power_mw / 1000.0f, true);

      syslog(LOG_INFO, "INA226 IN : %lumV %ldmA %lumW\n",
             (unsigned long)s.bus_mv, (long)s.current_ma,
             (unsigned long)s.power_mw);
    }
  else
    {
      cs_ui_set_input_power(0.0f, 0.0f, 0.0f, false);
    }

  if (cs_ina226_read(CS_INA226_IRON, &s) == OK)
    {
      cs_ui_set_iron_power((float)s.bus_mv / 1000.0f,
                           (float)s.current_ma / 1000.0f,
                           (float)s.power_mw / 1000.0f, true);

      syslog(LOG_INFO, "INA226 IRON: %lumV %ldmA %lumW\n",
             (unsigned long)s.bus_mv, (long)s.current_ma,
             (unsigned long)s.power_mw);
    }
  else
    {
      cs_ui_set_iron_power(0.0f, 0.0f, 0.0f, false);
    }
}

/****************************************************************************
 * Name: data_timer_cb
 *
 * Periodically refresh simulated data + real SW3538 readings.
 ****************************************************************************/

static void data_timer_cb(lv_timer_t *timer)
{
  static int slow = 0;
  static int retry = 0;
  struct iron_status_s ist;

  cs_ui_data_tick();

  /* Sync UI intent (target temp / heater on / PID) into the iron module */
  float kp;
  float ki;
  float kd;

  iron_set_target(cs_ui_get_iron_target());
  iron_set_heater(cs_ui_get_iron_on());
  cs_ui_get_iron_pid(&kp, &ki, &kd);
  iron_set_pid(kp * 0.2f, ki * 0.4f, kd * 0.2f);

  /* Soldering iron: PID tick every 100ms */
  iron_tick();
  iron_get_status(&ist);
  cs_ui_set_iron_real(ist.temp_c, ist.power_pct, ist.heating, ist.sensor_ok);
  cs_ui_set_iron_sleep(ist.sleep_active,
                       (ist.sleep_active ? ist.target_eff_c : ist.target_c));

  /* SW3538 sweep: one channel per 100ms tick, so the bus work never blocks
   * the LVGL loop for longer than a single channel (~23ms). */
  sw3538_poll_channel(g_poll_chan);
  if (++g_poll_chan >= 4)
    {
      g_poll_chan = 0;
    }

  /* INA226 is cheap (about 2ms for both monitors). */
  if (++slow >= 5)
    {
      slow = 0;
      ina226_poll();
    }

  /* Re-probe devices that were absent at boot: the power monitors, and any
   * SW3538 module that has been plugged in since. */
  if (++retry >= INA226_RETRY_TICKS)
    {
      retry = 0;
      cs_ina226_retry();
      sw3538_retry_absent();
    }

  (void)timer;
}

/****************************************************************************
 * Name: lv_nuttx_loop
 *
 * Standard LVGL poll loop.
 ****************************************************************************/

static void lv_nuttx_loop(void)
{
  while (1)
    {
      uint32_t idle;

      idle = lv_timer_handler();
      idle = idle ? idle : 1;
      usleep(idle * 1000);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  lv_nuttx_dsc_t info;
  lv_nuttx_result_t result;

  syslog(LOG_INFO, "Charging Station starting\n");

  /* Init soldering iron GPIO + MAX6675 probe (non-fatal) */
  iron_init();

  /* Bring up the software I2C bus and probe the TCA9548A plus the four
   * SW3538 modules.  Every step is non-fatal: the UI must still come up
   * with no hardware attached. */
  if (sw3538_init() < 0)
    {
      syslog(LOG_WARNING, "Charging Station: TCA9548A not detected\n");
    }
  else
    {
      int channel;

      for (channel = 0; channel < 4; channel++)
        {
          struct sw3538_status_s st;

          if (sw3538_read_status_channel(channel, &st) == OK)
            {
              g_sw3538_ok[channel] = true;
              syslog(LOG_INFO, "Charging Station: SW3538 CH%d ready\n",
                     channel + 1);
            }
          else
            {
              syslog(LOG_WARNING, "Charging Station: SW3538 CH%d absent\n",
                     channel + 1);
            }
        }
    }

  /* Probe the two INA226 power monitors on TCA channels 4 and 5. */

  if (cs_ina226_init() < 0)
    {
      syslog(LOG_WARNING, "Charging Station: INA226 init failed\n");
    }

  lv_init();

  lv_nuttx_dsc_init(&info);

#ifdef CONFIG_LV_USE_NUTTX_LCD
  info.fb_path = "/dev/lcd0";
#endif

  lv_nuttx_init(&info, &result);

  if (result.disp == NULL)
    {
      syslog(LOG_ERR, "Charging Station: display init failed\n");
      lv_deinit();
      return -1;
    }

  /* Build the charging station UI */
  if (cs_ui_init(lv_screen_active()) < 0)
    {
      syslog(LOG_ERR, "Charging Station: UI init failed\n");
      lv_nuttx_deinit(&result);
      lv_deinit();
      return -1;
    }

  /* Start data simulation timer (100ms) */
  lv_timer_create(data_timer_cb, 100, NULL);

  syslog(LOG_INFO, "Charging Station: display res=%ldx%ld\n",
         (long)lv_display_get_horizontal_resolution(result.disp),
         (long)lv_display_get_vertical_resolution(result.disp));

  if (result.indev)
    {
      /* Speed up touch reading: 20ms poll period (faster response) */
      lv_indev_set_mode(result.indev, LV_INDEV_MODE_TIMER);
      lv_timer_t *t = lv_indev_get_read_timer(result.indev);
      if (t)
        {
          lv_timer_set_period(t, 20);
        }
      syslog(LOG_INFO, "Charging Station: touchscreen registered (20ms poll)\n");
    }
  else
    {
      syslog(LOG_WARNING, "Charging Station: no touchscreen (indev NULL)\n");
    }

  /* Enter the LVGL event loop */
  syslog(LOG_INFO, "Charging Station: entering poll loop\n");
  lv_nuttx_loop();

  lv_nuttx_deinit(&result);
  lv_deinit();
  return 0;
}
