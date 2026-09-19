/****************************************************************************
 * Contest 2026 team 008 - SW3538 fast-charge chip driver (NuttX)
 *
 * Four SW3538 devices share I2C address 0x3c behind a TCA9548A at 0x70.
 * Each transaction selects exactly one TCA downstream channel first.
 *
 * The multiplexer hangs off the software I2C bus (see cs_i2c_sw.c), so it
 * does not share I2C4 with the FT5x06 touchscreen.  All bus access goes
 * through cs_i2c_transfer(), which re-selects the channel on every call and
 * holds a lock across select + transfer.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "cs_i2c_sw.h"
#include "sw3538.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SW3538_ADDR              0x3c
#define SW3538_CHANNEL_COUNT     4

#define SW3538_REG_PROTO         0x09
#define SW3538_REG_ONLINE        0x0d
#define SW3538_REG_I2C_EN        0x10
#define SW3538_REG_ADC_SEL       0x40
#define SW3538_REG_ADC_LO        0x41
#define SW3538_REG_ADC_HI        0x42
#define SW3538_REG_NTC_CURRENT   0x44

/* Reg0x40 channel codes (register list, section 2.23).
 *
 * Channels 1 and 2 are the two output paths of the chip.  Register list
 * 2.8 (Reg0x0A) and 2.9 (Reg0x0D) both state that for the A+C mode used
 * here, path 1 is the Type-C port and path 2 is the Type-A port.
 */

#define ADC_CH_PATH1             1   /* Type-C port current, 2.5mA/bit@5mohm */
#define ADC_CH_PATH2             2   /* Type-A port current, 2.5mA/bit@5mohm */
#define ADC_CH_VOUT              5   /* Output voltage, 6mV/bit */
#define ADC_CH_VIN               6   /* Input voltage, 10mV/bit (ADC off by default) */
#define ADC_CH_NTC               7   /* NTC voltage, 1.2mV/bit */
#define ADC_CH_VOUT_CONV        11   /* Converted output voltage, 1mV/bit */

#define NTC_UV_PER_BIT           1200u
#define NTC_CURRENT_LOW_UA       20u
#define NTC_CURRENT_HIGH_UA      40u

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 104AT 100K NTC nominal resistance table.  The values must be calibrated
 * against the actual module at room temperature before they are relied upon
 * for thermal decisions.  This driver reports invalid data rather than a
 * fabricated temperature for values outside the supported table. */

struct ntc_point_s
{
  int16_t temp_c;
  uint32_t resistance_ohm;
};

static const struct ntc_point_s g_ntc_104at[] =
{
  {  0, 355000 }, { 10, 202000 }, { 20, 125000 }, { 25, 100000 },
  { 30,  80500 }, { 40,  51000 }, { 50,  33500 }, { 60,  22400 },
  { 70,  15300 }, { 80,  10600 }, { 90,   7400 }, {100,   5250 },
  {110,   3800 }, {120,   2800 },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sw3538_read_regs(uint8_t channel, uint8_t reg, uint8_t *buf,
                            int len)
{
  struct i2c_msg_s msg[2];
  uint8_t regaddr = reg;

  msg[0].frequency = CS_I2C_FREQ;
  msg[0].addr = SW3538_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = CS_I2C_FREQ;
  msg[1].addr = SW3538_ADDR;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = buf;
  msg[1].length = len;

  return cs_i2c_transfer(channel, msg, 2);
}

static int sw3538_write_reg(uint8_t channel, uint8_t reg, uint8_t val)
{
  struct i2c_msg_s msg;
  uint8_t tx[2] = { reg, val };

  msg.frequency = CS_I2C_FREQ;
  msg.addr = SW3538_ADDR;
  msg.flags = 0;
  msg.buffer = tx;
  msg.length = sizeof(tx);
  return cs_i2c_transfer(channel, &msg, 1);
}

/****************************************************************************
 * Name: sw3538_unlock
 *
 * Enable I2C writes on one chip.  Register list 2.10 (Reg0x10, bits 7-5)
 * requires the sequence 0x20 -> 0x40 -> 0x80 before any other register may
 * be written to; reads need no unlock.  Every SW3538 sits behind its own
 * multiplexer channel, so the sequence has to be replayed on whichever
 * channel is about to be written, and it must precede every write to
 * Reg0x40 or the ADC channel change is silently ignored.
 ****************************************************************************/

static int sw3538_unlock(uint8_t channel)
{
  static const uint8_t seq[3] = { 0x20, 0x40, 0x80 };
  int i;
  int ret;

  for (i = 0; i < 3; i++)
    {
      ret = sw3538_write_reg(channel, SW3538_REG_I2C_EN, seq[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

static int sw3538_adc_code(uint8_t channel, uint8_t adc, FAR uint16_t *code)
{
  uint8_t lo;
  uint8_t hi;
  int ret;

  if (code == NULL)
    {
      return -EINVAL;
    }

  ret = sw3538_write_reg(channel, SW3538_REG_ADC_SEL, adc);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(5000);

  ret = sw3538_read_regs(channel, SW3538_REG_ADC_LO, &lo, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = sw3538_read_regs(channel, SW3538_REG_ADC_HI, &hi, 1);
  if (ret < 0)
    {
      return ret;
    }

  *code = (uint16_t)(((uint16_t)(hi & 0x0f) << 8) | lo);
  return OK;
}

static bool sw3538_ntc_to_temp(uint16_t code, uint8_t ntc_current,
                               FAR int16_t *temp_c)
{
  uint32_t current_ua;
  uint32_t resistance;
  unsigned int i;

  if (code == 0 || temp_c == NULL)
    {
      return false;
    }

  current_ua = (ntc_current & 0x80) ? NTC_CURRENT_HIGH_UA :
                                      NTC_CURRENT_LOW_UA;
  resistance = ((uint32_t)code * NTC_UV_PER_BIT) / current_ua;

  if (resistance > g_ntc_104at[0].resistance_ohm ||
      resistance < g_ntc_104at[(sizeof(g_ntc_104at) /
                                 sizeof(g_ntc_104at[0])) - 1].resistance_ohm)
    {
      return false;
    }

  for (i = 0; i < (sizeof(g_ntc_104at) / sizeof(g_ntc_104at[0])) - 1; i++)
    {
      const struct ntc_point_s *high = &g_ntc_104at[i];
      const struct ntc_point_s *low = &g_ntc_104at[i + 1];

      if (resistance <= high->resistance_ohm &&
          resistance >= low->resistance_ohm)
        {
          uint32_t span = high->resistance_ohm - low->resistance_ohm;
          uint32_t offset = high->resistance_ohm - resistance;

          *temp_c = (int16_t)(high->temp_c +
                    ((int32_t)offset * (low->temp_c - high->temp_c) / span));
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sw3538_init(void)
{
  int ret;

  ret = cs_i2c_bus_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "SW3538: software I2C bus init failed\n");
      return ret;
    }

  ret = cs_i2c_mux_probe();
  if (ret < 0)
    {
      syslog(LOG_ERR, "SW3538: no TCA9548A ACK at 0x%02x\n", CS_TCA9548A_ADDR);
      return ret;
    }

  syslog(LOG_INFO, "SW3538: TCA9548A ready at 0x%02x on software I2C\n",
         CS_TCA9548A_ADDR);
  return OK;
}

int sw3538_read_status_channel(uint8_t channel,
                                FAR struct sw3538_status_s *st)
{
  uint16_t code;
  uint8_t proto;
  uint8_t online;
  uint8_t ntc_current;
  int ret;

  if (st == NULL || channel >= SW3538_CHANNEL_COUNT)
    {
      return -EINVAL;
    }

  memset(st, 0, sizeof(*st));

  /* Reg0x40 is a write; the chip has to be write-enabled first or the ADC
   * channel selection does not take effect and every field would report the
   * same latched value. */

  ret = sw3538_unlock(channel);
  if (ret < 0)
    {
      return ret;
    }

  ret = sw3538_read_regs(channel, SW3538_REG_PROTO, &proto, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = sw3538_read_regs(channel, SW3538_REG_ONLINE, &online, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = sw3538_adc_code(channel, ADC_CH_VOUT, &code);
  if (ret < 0)
    {
      return ret;
    }
  st->vout_mv = (uint16_t)(code * 6u);

  ret = sw3538_adc_code(channel, ADC_CH_PATH1, &code);
  if (ret < 0)
    {
      return ret;
    }
  st->ic_ma = (uint16_t)(code * 25u / 10u);

  ret = sw3538_adc_code(channel, ADC_CH_PATH2, &code);
  if (ret < 0)
    {
      return ret;
    }
  st->ia_ma = (uint16_t)(code * 25u / 10u);

  ret = sw3538_read_regs(channel, SW3538_REG_NTC_CURRENT, &ntc_current, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = sw3538_adc_code(channel, ADC_CH_NTC, &code);
  if (ret < 0)
    {
      return ret;
    }

  st->temp_valid = sw3538_ntc_to_temp(code, ntc_current, &st->temp_c);
  st->proto = proto & 0x0f;
  st->online = online & 0x03;
  return OK;
}

int sw3538_read_status(FAR struct sw3538_status_s *st)
{
  return sw3538_read_status_channel(0, st);
}

/****************************************************************************
 * Name: sw3538_probe_channel
 *
 * Cheap presence check on one multiplexer channel: write-enable the chip and
 * read back its protocol register.  Used to pick up a module that is plugged
 * in after boot, so it must not rely on any state left by an earlier read.
 ****************************************************************************/

int sw3538_probe_channel(uint8_t channel)
{
  uint8_t proto;

  if (channel >= SW3538_CHANNEL_COUNT)
    {
      return -EINVAL;
    }

  if (sw3538_unlock(channel) < 0)
    {
      return -ENODEV;
    }

  if (sw3538_read_regs(channel, SW3538_REG_PROTO, &proto, 1) < 0)
    {
      return -ENODEV;
    }

  return OK;
}

const char *sw3538_proto_name(uint8_t proto)
{
  switch (proto)
    {
      case 0:  return "IDLE";
      case 1:  return "QC2.0";
      case 2:  return "QC3.0";
      case 3:  return "QC3+";
      case 4:  return "FCP";
      case 5:  return "SCP";
      case 6:  return "PD";
      case 7:  return "PD-PPS";
      case 8:  return "PE1.1";
      case 9:  return "PE2.0";
      case 10: return "VOOC1";
      case 11: return "VOOC4";
      case 13: return "SFCP";
      case 14: return "AFC";
      case 15: return "TFCP";
      default: return "?";
    }
}
