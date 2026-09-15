/****************************************************************************
 * Contest 2026 team 008 - INA226 power monitor driver
 *
 * Two INA226 shunt monitors behind the TCA9548A (see cs_i2c_sw.h):
 *   CH4 -> total power input current   (address 0x40)
 *   CH5 -> soldering iron heater power (address 0x46)
 *
 * This is a register level driver, not a character device: the application
 * polls both monitors from its own timer and pushes the results to the UI.
 *
 * Register map (datasheet section 7.6):
 *   0x00 Configuration   0x01 Shunt Voltage   0x02 Bus Voltage
 *   0x03 Power           0x04 Current         0x05 Calibration
 *   0xFE Manufacturer ID (0x5449 = 'TI')
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>

#include "cs_i2c_sw.h"
#include "ina226.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Shunt resistor values in micro-ohms.  THESE MUST MATCH THE HARDWARE: an
 * error here scales both the current and the power reading by the same
 * factor.  The driver prints the resulting current LSB at start-up so it
 * can be checked against a bench meter.
 */

#define CS_INA226_TOTAL_SHUNT_UOHM   2500u  /* 2.5 mOhm */
#define CS_INA226_IRON_SHUNT_UOHM    5000u  /* 5.0 mOhm */

/* Target current resolution in micro-amps.  The datasheet suggests picking
 * a round value near Max_Expected_Current / 32768 so the current and power
 * registers come out in convenient units.  Both resistors below divide the
 * calibration formula exactly, so the effective resolution equals the
 * requested one with no truncation error.
 */

#define CS_INA226_TOTAL_LSB_UA       1000u  /* 1 mA   -> +/-32.77 A */
#define CS_INA226_IRON_LSB_UA         500u  /* 0.5 mA -> +/-16.38 A */

/* Slave addresses, set by the A1/A0 pins (datasheet table 2). */

#define CS_INA226_ADDR_TOTAL         0x40   /* A1 = GND, A0 = GND */
#define CS_INA226_ADDR_IRON          0x46   /* A1 = VS,  A0 = VS  */

#define CS_INA226_REG_CONFIG         0x00
#define CS_INA226_REG_SHUNT          0x01
#define CS_INA226_REG_BUS            0x02
#define CS_INA226_REG_POWER          0x03
#define CS_INA226_REG_CURRENT        0x04
#define CS_INA226_REG_CALIBRATION    0x05
#define CS_INA226_REG_MANUFACTURER   0xfe

#define CS_INA226_MANUFACTURER_ID    0x5449   /* 'TI' */

/* Configuration register: AVG = 16 samples, 1.1 ms bus + 1.1 ms shunt
 * conversions, continuous shunt and bus mode (datasheet table 5).
 * One full cycle takes 16 x (1.1 + 1.1) ms = 35.2 ms. */

#define CS_INA226_CFG_AVG_16         (2u << 9)
#define CS_INA226_CFG_VBUSCT_1P1MS   (4u << 6)
#define CS_INA226_CFG_VSHCT_1P1MS    (4u << 3)
#define CS_INA226_CFG_MODE_CONT      (7u << 0)
#define CS_INA226_CFG_DEFAULT        (CS_INA226_CFG_AVG_16 | \
                                      CS_INA226_CFG_VBUSCT_1P1MS | \
                                      CS_INA226_CFG_VSHCT_1P1MS | \
                                      CS_INA226_CFG_MODE_CONT)

#define CS_INA226_CONFIG_RESET       (1u << 15)

/* Mark a device absent after this many consecutive failed reads. */

#define CS_INA226_MAX_ERRORS         3

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cs_ina226_dev_s
{
  uint8_t  channel;      /* TCA9548A downstream channel */
  uint8_t  addr;         /* I2C slave address */
  uint16_t lsb_ua;       /* Requested current LSB, uA */
  uint32_t shunt_uohm;   /* Shunt resistor in micro-ohms */
  uint16_t cal;          /* Value written to the calibration register */
  uint32_t lsb_na;       /* Effective current LSB in nA */
  uint8_t  errors;       /* Consecutive read failures */
  bool     present;      /* Device answered the last access */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cs_ina226_dev_s g_ina226[CS_INA226_COUNT] =
{
  {
    CS_TCA_CH_INA_TOTAL, CS_INA226_ADDR_TOTAL, CS_INA226_TOTAL_LSB_UA,
    CS_INA226_TOTAL_SHUNT_UOHM, 0, 0, 0, false
  },
  {
    CS_TCA_CH_INA_IRON,  CS_INA226_ADDR_IRON,  CS_INA226_IRON_LSB_UA,
    CS_INA226_IRON_SHUNT_UOHM,  0, 0, 0, false
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs_ina226_read_reg
 *
 * Read a 16-bit register using the repeated start shown in the datasheet
 * timing diagram (figure 23).
 ****************************************************************************/

static int cs_ina226_read_reg(FAR struct cs_ina226_dev_s *dev, uint8_t reg,
                              FAR uint16_t *value)
{
  struct i2c_msg_s msg[2];
  uint8_t regaddr = reg;
  uint8_t buf[2];
  int ret;

  msg[0].frequency = CS_I2C_FREQ;
  msg[0].addr      = dev->addr;
  msg[0].flags     = I2C_M_NOSTOP;   /* Keep the bus for the read */
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CS_I2C_FREQ;
  msg[1].addr      = dev->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = 2;

  ret = cs_i2c_transfer(dev->channel, msg, 2);
  if (ret < 0)
    {
      return ret;
    }

  *value = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
  return OK;
}

/****************************************************************************
 * Name: cs_ina226_write_reg
 ****************************************************************************/

static int cs_ina226_write_reg(FAR struct cs_ina226_dev_s *dev, uint8_t reg,
                               uint16_t value)
{
  struct i2c_msg_s msg;
  uint8_t tx[3];

  tx[0] = reg;
  tx[1] = (uint8_t)(value >> 8);
  tx[2] = (uint8_t)(value & 0xff);

  msg.frequency = CS_I2C_FREQ;
  msg.addr      = dev->addr;
  msg.flags     = 0;
  msg.buffer    = tx;
  msg.length    = sizeof(tx);

  return cs_i2c_transfer(dev->channel, &msg, 1);
}

/****************************************************************************
 * Name: cs_ina226_calibrate
 *
 * Program the calibration register from the shunt value.
 *
 *   CAL = 0.00512 / (Current_LSB[A] * R_shunt[ohm])
 *       = 5.12e9   / (Current_LSB[uA] * R_shunt[uohm])
 *
 * The result is truncated to an integer register, so the current LSB is
 * recomputed from the value actually written.  Skipping that step leaves a
 * fixed gain error of up to one part in 65535 on every current reading.
 *
 * Returned Value:
 *   OK, or -ERANGE when the shunt does not yield a usable calibration value.
 ****************************************************************************/

static int cs_ina226_calibrate(FAR struct cs_ina226_dev_s *dev)
{
  uint64_t cal;

  cal = 5120000000ull / ((uint64_t)dev->lsb_ua *
                         (uint64_t)dev->shunt_uohm);
  if (cal < 1 || cal > 65535)
    {
      syslog(LOG_ERR, "INA226 ch%u: %lu uOhm shunt cannot be calibrated "
             "with a %u uA LSB (cal=%llu)\n", dev->channel,
             (unsigned long)dev->shunt_uohm, (unsigned)dev->lsb_ua,
             (unsigned long long)cal);
      return -ERANGE;
    }

  dev->cal    = (uint16_t)cal;
  dev->lsb_na = (uint32_t)(5120000000000ull / (cal * dev->shunt_uohm));
  return OK;
}

/****************************************************************************
 * Name: cs_ina226_probe
 *
 * Identify the device and bring it into continuous conversion mode.
 ****************************************************************************/

static int cs_ina226_probe(FAR struct cs_ina226_dev_s *dev)
{
  uint16_t id;
  int ret;

  ret = cs_ina226_read_reg(dev, CS_INA226_REG_MANUFACTURER, &id);
  if (ret < 0)
    {
      return ret;
    }

  if (id != CS_INA226_MANUFACTURER_ID)
    {
      syslog(LOG_WARNING, "INA226 ch%u addr 0x%02x: bad manufacturer ID "
             "0x%04x (expected 0x%04x)\n", dev->channel, dev->addr, id,
             CS_INA226_MANUFACTURER_ID);
      return -ENODEV;
    }

  if (cs_ina226_calibrate(dev) < 0)
    {
      return -ERANGE;
    }

  /* Reset, then configure for continuous conversion. */

  ret = cs_ina226_write_reg(dev, CS_INA226_REG_CONFIG, CS_INA226_CONFIG_RESET);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(1000);

  ret = cs_ina226_write_reg(dev, CS_INA226_REG_CONFIG, CS_INA226_CFG_DEFAULT);
  if (ret < 0)
    {
      return ret;
    }

  ret = cs_ina226_write_reg(dev, CS_INA226_REG_CALIBRATION, dev->cal);
  if (ret < 0)
    {
      return ret;
    }

  syslog(LOG_INFO, "INA226 ch%u addr 0x%02x: %lu uOhm shunt, cal=%u "
         "(%lu nA/count, +/-%.2f A full scale)\n",
         dev->channel, dev->addr, (unsigned long)dev->shunt_uohm, dev->cal,
         (unsigned long)dev->lsb_na,
         32767.0 * (double)dev->lsb_na / 1.0e9);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cs_ina226_init(void)
{
  unsigned int i;

  if (!cs_i2c_bus_ready())
    {
      int ret = cs_i2c_bus_init();
      if (ret < 0)
        {
          return ret;
        }
    }

  for (i = 0; i < CS_INA226_COUNT; i++)
    {
      if (cs_ina226_probe(&g_ina226[i]) < 0)
        {
          /* Report once, then stay quiet until cs_ina226_retry() finds it. */

          syslog(LOG_WARNING, "INA226: monitor %u not detected on TCA ch%u "
                 "(addr 0x%02x)\n", i, g_ina226[i].channel,
                 g_ina226[i].addr);
          g_ina226[i].present = false;
        }
      else
        {
          g_ina226[i].present = true;
          g_ina226[i].errors  = 0;
        }
    }

  return OK;
}

bool cs_ina226_present(uint8_t dev)
{
  if (dev >= CS_INA226_COUNT)
    {
      return false;
    }

  return g_ina226[dev].present;
}

int cs_ina226_read(uint8_t dev, FAR struct ina226_sample_s *out)
{
  FAR struct cs_ina226_dev_s *d;
  uint16_t raw;
  int ret;

  if (dev >= CS_INA226_COUNT || out == NULL)
    {
      return -EINVAL;
    }

  d = &g_ina226[dev];
  if (!d->present)
    {
      /* Do not burn four ACK timeouts per poll on a monitor that is not
       * plugged in. */

      return -ENODEV;
    }

  ret = cs_ina226_read_reg(d, CS_INA226_REG_BUS, &raw);
  if (ret < 0)
    {
      goto lost;
    }

  /* Bus voltage is unsigned; bit 15 is always zero. */

  out->bus_mv = (uint32_t)(raw & 0x7fff) * 5u / 4u;

  ret = cs_ina226_read_reg(d, CS_INA226_REG_SHUNT, &raw);
  if (ret < 0)
    {
      goto lost;
    }

  out->shunt_uv = (int32_t)(int16_t)raw * 5 / 2;

  ret = cs_ina226_read_reg(d, CS_INA226_REG_CURRENT, &raw);
  if (ret < 0)
    {
      goto lost;
    }

  out->current_ma = (int32_t)(((int64_t)(int16_t)raw * d->lsb_na) / 1000000);

  ret = cs_ina226_read_reg(d, CS_INA226_REG_POWER, &raw);
  if (ret < 0)
    {
      goto lost;
    }

  /* The power LSB is internally fixed at 25 x Current_LSB. */

  out->power_mw = (uint32_t)(((uint64_t)raw * 25ull * d->lsb_na) /
                             1000000ull);

  d->errors = 0;
  return OK;

lost:
  if (++d->errors >= CS_INA226_MAX_ERRORS)
    {
      d->errors  = 0;
      d->present = false;
      syslog(LOG_WARNING, "INA226: monitor %u lost on TCA ch%u\n", dev,
             d->channel);
    }

  return ret;
}

void cs_ina226_retry(void)
{
  unsigned int i;

  if (!cs_i2c_bus_ready())
    {
      return;
    }

  for (i = 0; i < CS_INA226_COUNT; i++)
    {
      FAR struct cs_ina226_dev_s *d = &g_ina226[i];

      if (d->present)
        {
          continue;
        }

      if (cs_ina226_probe(d) == OK)
        {
          d->present = true;
          d->errors  = 0;
          syslog(LOG_INFO, "INA226: monitor %u reappeared on TCA ch%u\n", i,
                 d->channel);
        }
    }
}
