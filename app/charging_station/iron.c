/****************************************************************************
 * Contest 2026 team 008 - soldering iron module
 *
 * MAX6675 K-thermocouple reader over software SPI (bit-bang GPIO),
 * software PWM heater control and a simple PID loop.
 *
 * Wiring (Arduino headers):
 *   MAX6675 SCK -> D8  (PE3)
 *   MAX6675 CS  -> D7  (PI8)
 *   MAX6675 SO  -> D6  (PE6)
 *   Heater PWM  -> D3  (PA6) -> NPN transistor -> MOSFET -> 24V iron
 *   Sleep sense -> D2  (PG3)
 *
 * The remaining Arduino pins are owned by other modules:
 *   D4 (PK1) / D5 (PA8) -> TCA9548A software I2C, see cs_i2c_sw.c
 *   D9 (PH15)           -> board ID input, not used yet
 *
 * MAX6675 protocol (16-bit read):
 *   D15=0(dummy), D14-D3=temp(12bit, 0.25C/bit),
 *   D2=1 thermocouple open, D1=0, D0=tri-state
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "iron.h"

/* GPIO config macros come from chip headers not on the app include
 * path; use explicit values via the generic stm32 API forward decls.
 */
extern int  stm32_configgpio(uint32_t cfgset);
extern void stm32_gpiowrite(uint32_t pinset, bool value);
extern bool stm32_gpioread(uint32_t pinset);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* --- GPIO pin assignments (Arduino D2/D3/D6/D7/D8) ---
 * stm32_gpio.h is not on the app include path, so pinsets are built
 * from the documented bit layout:
 *   MODE bits18-19 (OUTPUT=1<<18), SPEED bits10-11 (50MHz=2<<10),
 *   OUTPUT_SET bit8 (0x100), PORT bits4-7 (A=0,E=4,I=8), PIN bits0-3
 */
#define GPIO_OUTPUT_VAL     (1u << 18)
#define GPIO_SPEED_50MHZ    (2u << 10)
#define GPIO_OUTPUT_SETBIT  (1u << 8)
#define GPIO_PULLUP         (1u << 16)

#define IRON_GPIO_SCK    (GPIO_OUTPUT_VAL | GPIO_SPEED_50MHZ | \
                          (4u << 4) | 3u)              /* D8 = PE3  */
#define IRON_GPIO_CS     (GPIO_OUTPUT_VAL | GPIO_SPEED_50MHZ | \
                          GPIO_OUTPUT_SETBIT | (8u << 4) | 8u)  /* D7 = PI8 */
#define IRON_GPIO_SO     ((4u << 4) | 6u)               /* D6 = PE6 input */
#define IRON_GPIO_HEAT   (GPIO_OUTPUT_VAL | GPIO_SPEED_50MHZ | \
                          (0u << 4) | 6u)              /* D3 = PA6  */
#define IRON_GPIO_SLEEP  (GPIO_PULLUP | (6u << 4) | 3u) /* D2 = PG3, active low */

/* --- Timing (us) --- */
#define IRON_SPI_HALF_PERIOD_US  2    /* ~250kHz bit clock */
#define IRON_TICK_MS             100  /* PID period */

/* --- PID limits --- */
#define IRON_TEMP_MIN     100
#define IRON_TEMP_MAX     480
#define IRON_TEMP_CUTOFF  500
#define IRON_PWR_MAX      100.0f
#define IRON_PID_DT       (IRON_TICK_MS / 1000.0f)
#define IRON_PID_KP_MAX   20.0f
#define IRON_PID_KI_MAX   2.0f
#define IRON_PID_KD_MAX   20.0f
#define IRON_SAMPLE_MAX_AGE  0.6f

/* Software PWM: the iron module ticks every 100ms; the PWM cycle spans
 * IRON_PWM_TICKS ticks => 2s period (0.5Hz). Soldering irons have large
 * thermal inertia, so this is far below their thermal time constant and the
 * resulting ripple is negligible.  power% => the heater stays ON for the
 * first N slices of each cycle, OFF for the rest.
 *
 * 20 slices give 5% resolution.  That matters because an idling JBC245 only
 * needs roughly 6-18% duty to hold temperature, so a coarser scale would
 * quantise the entire operating point away: at 10 slices everything below
 * 9% collapsed to "off" and 10-19% all became 10%, which made the tip hunt
 * around its target instead of settling. */
#define IRON_PWM_TICKS       20   /* 2s cycle = 20 x 100ms slices */

/* Derivative low-pass: the temperature is sampled every 300ms, which makes
 * the raw difference quotient noisy and would kick the D-term hard on every
 * sample.  Clamp the measured rate of change instead; the units are C/s, so
 * with the default Kd the D-term is bounded to 1.5 * 8 = 12% of output. */
#define IRON_D_MAX_SLOPE     8.0f   /* max temperature slope, C/s */

/* JBC245 sleep detection (Arduino D2 = PG3, active low with pull-up).
 * Parking the handle simply stops the heater, so there is no staged
 * cool-down or standby temperature to wait for. */
#define IRON_SLEEP_DEBOUNCE_TICKS 3   /* 300ms bounce filter */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static float g_temp_c      = 25.0f;
static int   g_target_c    = 350;
static bool  g_heating     = false;
static bool  g_sensor_ok   = false;
static float g_kp          = 3.0f;
static float g_ki          = 0.05f;
static float g_kd          = 1.5f;
static float g_power       = 0.0f;   /* 0..100 */

/* PID state */
static float g_integral    = 0.0f;
static float g_last_temp_c = 25.0f;
static bool  g_have_sample = false;
static float g_sample_age  = 0.0f;

/* Software PWM state: counts down every tick */
static int   g_pwm_slice   = 0;

/* JBC245 sleep state: true while the handle rests in its stand, signalled
 * by the SLEEP line being pulled low. */

static bool  g_sleep_pin_deb   = false;
static int   g_sleep_deb_cnt   = 0;
static bool  g_parked          = false;
static int   g_target_eff_c    = 350;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iron_spi_delay
 ****************************************************************************/

static inline void iron_spi_delay(void)
{
  up_udelay(IRON_SPI_HALF_PERIOD_US);
}

/****************************************************************************
 * Name: max6675_read
 *
 * Bit-bang a 16-bit read from the MAX6675.
 * Returns 16-bit raw value (D2 = thermocouple open flag).
 ****************************************************************************/

static uint16_t max6675_read(void)
{
  uint16_t value = 0;
  int i;

  /* CS low to start */
  stm32_gpiowrite(IRON_GPIO_CS, false);
  iron_spi_delay();

  /* Read 16 bits MSB first.  The MAX6675 updates SO after the falling
   * edge; sample each bit while SCK is low and stable. */
  for (i = 0; i < 16; i++)
    {
      /* SCK low -> data valid to read */
      stm32_gpiowrite(IRON_GPIO_SCK, false);
      iron_spi_delay();

      /* Sample SO while SCK is low: the MAX6675 shifts out the next bit on
       * the falling edge, so the line is already stable when the clock is
       * low and we read before driving it high again. */
      value = (value << 1);
      if (stm32_gpioread(IRON_GPIO_SO))
        {
          value |= 1;
        }

      /* SCK high */
      stm32_gpiowrite(IRON_GPIO_SCK, true);
      iron_spi_delay();
    }

  /* CS high to end */
  stm32_gpiowrite(IRON_GPIO_CS, true);
  iron_spi_delay();

  return value;
}

/****************************************************************************
 * Name: max6675_temp
 *
 * Decode a 16-bit MAX6675 frame into Celsius.
 * Returns false (no temperature written) when the thermocouple is open,
 * shorted, or the frame is otherwise unusable.
 ****************************************************************************/

static bool max6675_temp(uint16_t raw, FAR float *temp_c)
{
  uint16_t code;

  if (temp_c == NULL || (raw & 0x8000) != 0 || (raw & 0x0002) != 0 ||
      (raw & 0x0004) != 0)
    {
      return false;
    }

  code = (raw >> 3) & 0x0fff;
  *temp_c = (float)code * 0.25f;
  return *temp_c >= 0.0f && *temp_c <= 1023.75f;
}

/****************************************************************************
 * Name: iron_pid_update
 *
 * Compute heater power % from PID on temperature error.
 ****************************************************************************/

static void iron_pid_update(float dt)
{
  float error;
  float p_term;
  float i_term;
  float d_term;
  float d_slope;
  float candidate;

  if (!g_heating || !g_sensor_ok || !g_have_sample)
    {
      g_power = 0.0f;
      g_integral = 0.0f;
      return;
    }

  error = (float)g_target_eff_c - g_temp_c;
  p_term = g_kp * error;
  d_slope = (g_temp_c - g_last_temp_c) / dt;
  if (d_slope > IRON_D_MAX_SLOPE) d_slope = IRON_D_MAX_SLOPE;
  if (d_slope < -IRON_D_MAX_SLOPE) d_slope = -IRON_D_MAX_SLOPE;
  d_term = -g_kd * d_slope;

  /* Conditional integration: do not integrate farther into saturation. */
  candidate = p_term + g_ki * g_integral + d_term;
  if (!((candidate >= IRON_PWR_MAX && error > 0.0f) ||
        (candidate <= 0.0f && error < 0.0f)))
    {
      g_integral += error * dt;
      if (g_integral > 1000.0f) g_integral = 1000.0f;
      if (g_integral < -1000.0f) g_integral = -1000.0f;
    }

  i_term = g_ki * g_integral;
  g_power = p_term + i_term + d_term;
  if (g_power < 0.0f) g_power = 0.0f;
  if (g_power > IRON_PWR_MAX) g_power = IRON_PWR_MAX;
}

/****************************************************************************
 * Name: iron_pwm_apply
 *
 * Software PWM: turn heater GPIO on/off based on power %.
 * Called once per tick (100ms).  Each tick advances one slice of the
 * IRON_PWM_TICKS-long cycle; the heater is on for the first on_slices of
 * them, so the duty cycle is on_slices / IRON_PWM_TICKS.
 ****************************************************************************/

static void iron_pwm_apply(void)
{
  /* Round rather than truncate: a floor here would bias every duty cycle
   * downwards by up to one full slice. */

  int on_slices = (int)(g_power / 100.0f * IRON_PWM_TICKS + 0.5f);

  if (!g_heating || on_slices <= 0)
    {
      g_pwm_slice = 0;
      stm32_gpiowrite(IRON_GPIO_HEAT, false);
      return;
    }

  if (on_slices >= IRON_PWM_TICKS)
    {
      /* Constant on: restart the phase so a later drop back to a partial
       * duty cycle starts from a clean slice rather than a stale one. */

      g_pwm_slice = 0;
      stm32_gpiowrite(IRON_GPIO_HEAT, true);
      return;
    }

  stm32_gpiowrite(IRON_GPIO_HEAT, g_pwm_slice < on_slices);
  g_pwm_slice++;
  if (g_pwm_slice >= IRON_PWM_TICKS)
    {
      g_pwm_slice = 0;
    }
}

/****************************************************************************
 * Name: iron_sleep_tick
 *
 * Debounce the JBC245 SLEEP pin.  The handle pulls the line low while the
 * iron rests in its stand, so an active-low reading means "parked".
 *
 * Parked irons do not heat at all: the heater is held off for as long as
 * the pin reads low, and the PID state is cleared so that picking the iron
 * back up starts from a clean controller rather than from whatever the
 * integrator had accumulated before it was put down.
 ****************************************************************************/

static void iron_sleep_tick(void)
{
  bool raw;

  raw = !stm32_gpioread(IRON_GPIO_SLEEP);   /* active low: in stand */

  /* Bounce filter: require N consecutive identical samples. */
  if (raw != g_sleep_pin_deb)
    {
      g_sleep_deb_cnt++;
      if (g_sleep_deb_cnt >= IRON_SLEEP_DEBOUNCE_TICKS)
        {
          g_sleep_pin_deb = raw;
          g_sleep_deb_cnt = 0;
        }
    }
  else
    {
      g_sleep_deb_cnt = 0;
    }

  if (g_sleep_pin_deb == g_parked)
    {
      return;   /* no transition */
    }

  if (g_sleep_pin_deb)
    {
      /* Just parked: stop heating and drop the PID history. */
      g_parked = true;
      g_power       = 0.0f;
      g_integral    = 0.0f;
      g_pwm_slice   = 0;
      g_last_temp_c = g_temp_c;
      g_target_eff_c = g_target_c;
      stm32_gpiowrite(IRON_GPIO_HEAT, false);
      syslog(LOG_INFO, "IRON: parked, heater off\n");
    }
  else
    {
      /* Picked back up: resume the user target with a clean controller. */
      g_parked = false;
      g_integral    = 0.0f;
      g_last_temp_c = g_temp_c;
      g_target_eff_c = g_target_c;
      syslog(LOG_INFO, "IRON: picked up, resuming %dC\n", g_target_c);
    }
}

/* Returns true while the iron is parked in its stand (SLEEP line low). */

static bool iron_is_parked(void)
{
  return g_parked;
}


int iron_init(void)
{
  /* Configure GPIO pins */
  stm32_configgpio(IRON_GPIO_SCK);
  stm32_configgpio(IRON_GPIO_CS);
  stm32_configgpio(IRON_GPIO_SO);
  stm32_configgpio(IRON_GPIO_HEAT);
  stm32_configgpio(IRON_GPIO_SLEEP);

  /* Idle states: CS high, clock low, heater off */
  stm32_gpiowrite(IRON_GPIO_CS, true);
  stm32_gpiowrite(IRON_GPIO_SCK, false);
  stm32_gpiowrite(IRON_GPIO_HEAT, false);

  /* Probe MAX6675 */
  {
    uint16_t raw = max6675_read();
    float t;

    if (!max6675_temp(raw, &t))
      {
        syslog(LOG_WARNING, "IRON: invalid MAX6675 frame (raw=0x%04x)\n", raw);
        g_sensor_ok = false;
      }
    else
      {
        g_temp_c = t;
        g_last_temp_c = t;
        g_have_sample = true;
        g_sample_age = 0.0f;
        g_sensor_ok = true;
        syslog(LOG_INFO, "IRON: MAX6675 ok, T=%.1fC (raw=0x%04x)\n", t, raw);
      }
  }

  return 0;
}

void iron_tick(void)
{
  static int read_div = 0;
  uint16_t raw;
  float t;
  bool fresh_sample = false;

  g_sample_age += IRON_PID_DT;

  /* JBC245 sleep detection runs every tick, before sampling. */
  iron_sleep_tick();

  /* CS high starts a conversion.  Read only after three 100ms ticks so the
   * MAX6675 has exceeded its 220ms maximum conversion time. */
  if (++read_div >= 3)
    {
      read_div = 0;
      raw = max6675_read();

      if (max6675_temp(raw, &t))
        {
          if (t >= IRON_TEMP_CUTOFF)
            {
              syslog(LOG_WARNING, "IRON: over-temperature %.1fC\n", t);
              g_sensor_ok = false;
            }
          else
            {
              g_last_temp_c = g_temp_c;
              g_temp_c = t;
              g_sample_age = 0.0f;
              g_have_sample = true;
              g_sensor_ok = true;
              fresh_sample = true;
            }
        }
      else
        {
          syslog(LOG_WARNING, "IRON: invalid MAX6675 frame (raw=0x%04x)\n", raw);
          g_sensor_ok = false;
        }
    }

  if (!g_sensor_ok || g_sample_age > IRON_SAMPLE_MAX_AGE)
    {
      /* A stale or invalid sensor reading must always shut the heater off. */
      g_sensor_ok = false;
      g_power = 0.0f;
      g_integral = 0.0f;
      g_pwm_slice = 0;
      stm32_gpiowrite(IRON_GPIO_HEAT, false);
      return;
    }

  /* Parked in the stand: no heating at all, regardless of the target. */

  if (iron_is_parked())
    {
      g_power = 0.0f;
      g_integral = 0.0f;
      g_pwm_slice = 0;
      g_last_temp_c = g_temp_c;
      stm32_gpiowrite(IRON_GPIO_HEAT, false);
      return;
    }

  if (fresh_sample)
    {
      iron_pid_update(IRON_PID_DT * 3.0f);
    }

  iron_pwm_apply();
}

void iron_set_target(int temp_c)
{
  if (temp_c < IRON_TEMP_MIN) temp_c = IRON_TEMP_MIN;
  if (temp_c > IRON_TEMP_MAX) temp_c = IRON_TEMP_MAX;
  if (temp_c != g_target_c)
    {
      g_integral = 0.0f;
      g_last_temp_c = g_temp_c;
    }

  g_target_c = temp_c;

  /* While parked the target is remembered but not acted on; the effective
   * target is restored when the iron is picked up again. */

  if (!iron_is_parked())
    {
      g_target_eff_c = temp_c;
    }
}

void iron_set_heater(bool on)
{
  g_heating = on;
  if (!on)
    {
      /* Stop the output and drop the controller history.  The park state is
       * deliberately left alone: it tracks the SLEEP pin, which only
       * iron_sleep_tick() reads.  Writing it here from a raw pin sample
       * would bypass the debounce and let a UI action overwrite a physical
       * condition -- turning the heat switch off and on again would then
       * latch "parked" without the iron having moved. */

      g_integral = 0.0f;
      g_last_temp_c = g_temp_c;
      g_power = 0.0f;
      g_pwm_slice = 0;
      stm32_gpiowrite(IRON_GPIO_HEAT, false);
    }
}

void iron_set_pid(float p, float i, float d)
{
  if (!isfinite(p) || !isfinite(i) || !isfinite(d) || p < 0.0f ||
      i < 0.0f || d < 0.0f || p > IRON_PID_KP_MAX ||
      i > IRON_PID_KI_MAX || d > IRON_PID_KD_MAX)
    {
      syslog(LOG_WARNING, "IRON: rejected invalid PID P=%.2f I=%.2f D=%.2f\n", p, i, d);
      return;
    }

  if (p != g_kp || i != g_ki || d != g_kd)
    {
      g_integral = 0.0f;
      g_last_temp_c = g_temp_c;
    }

  g_kp = p;
  g_ki = i;
  g_kd = d;
}

void iron_get_status(FAR struct iron_status_s *st)
{
  if (st == NULL)
    {
      return;
    }

  st->temp_c       = (int)g_temp_c;
  st->target_c     = g_target_c;
  st->power_pct    = g_power;
  st->heating      = g_heating;
  st->sensor_ok    = g_sensor_ok;
  st->sleep_active = g_parked;
  st->target_eff_c = g_target_eff_c;
}
