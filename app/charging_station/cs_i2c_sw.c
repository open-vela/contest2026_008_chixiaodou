/****************************************************************************
 * Contest 2026 team 008 - software (bit-bang) I2C transport
 *
 * Wraps the generic NuttX i2c_bitbang engine (CONFIG_I2C_BITBANG) with
 * STM32H7 GPIO operations and adds atomic TCA9548A channel selection.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_bitbang.h>

#include "cs_i2c_sw.h"

/* Chip headers are not on the app include path; the generic GPIO API is
 * declared here and pinsets are built from the documented bit layout:
 *   MODE bits18-19 (OUTPUT=1<<18), SPEED bits10-11 (50MHz=2<<10),
 *   OPEN-DRAIN bit9, OUTPUT_SET bit8, PUPD bits16-17 (PULLUP=1<<16),
 *   PORT bits4-7 (A=0,K=10), PIN bits0-3
 */

extern int  stm32_configgpio(uint32_t cfgset);
extern void stm32_gpiowrite(uint32_t pinset, bool value);
extern bool stm32_gpioread(uint32_t pinset);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_OUTPUT_VAL     (1u << 18)
#define GPIO_SPEED_50MHZ    (2u << 10)
#define GPIO_OUTPUT_SETBIT  (1u << 8)
#define GPIO_OPENDRAIN_VAL  (1u << 9)
#define GPIO_PULLUP         (1u << 16)

#define CS_I2C_SCL_PIN      (GPIO_OUTPUT_VAL | GPIO_SPEED_50MHZ | \
                             GPIO_OPENDRAIN_VAL | GPIO_OUTPUT_SETBIT | \
                             GPIO_PULLUP | (10u << 4) | 1u)  /* D4 = PK1 */
#define CS_I2C_SDA_PIN      (GPIO_OUTPUT_VAL | GPIO_SPEED_50MHZ | \
                             GPIO_OPENDRAIN_VAL | GPIO_OUTPUT_SETBIT | \
                             GPIO_PULLUP | (0u << 4) | 8u)   /* D5 = PA8 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cs_i2c_priv_s
{
  uint32_t scl;    /* SCL pinset */
  uint32_t sda;    /* SDA pinset */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cs_i2c_priv_s g_cs_i2c_priv =
{
  CS_I2C_SCL_PIN,
  CS_I2C_SDA_PIN
};

static struct i2c_bitbang_lower_dev_s g_cs_i2c_lower;
static FAR struct i2c_master_s *g_cs_i2c_bus;
static mutex_t g_cs_i2c_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The bit-bang engine calls these from inside a spinlock, so they must stay
 * trivial: no delays, no logging, no allocation. */

static void cs_i2c_initialize(FAR struct i2c_bitbang_lower_dev_s *lower)
{
  FAR struct cs_i2c_priv_s *priv = lower->priv;

  stm32_configgpio(priv->scl);
  stm32_configgpio(priv->sda);

  /* Release both lines so the bus idles high.  Whether the lines actually
   * came up is checked once by cs_i2c_bus_init(). */

  stm32_gpiowrite(priv->scl, true);
  stm32_gpiowrite(priv->sda, true);
}

static void cs_i2c_set_scl(FAR struct i2c_bitbang_lower_dev_s *lower,
                           bool high)
{
  FAR struct cs_i2c_priv_s *priv = lower->priv;

  stm32_gpiowrite(priv->scl, high);
}

static void cs_i2c_set_sda(FAR struct i2c_bitbang_lower_dev_s *lower,
                           bool high)
{
  FAR struct cs_i2c_priv_s *priv = lower->priv;

  stm32_gpiowrite(priv->sda, high);
}

static bool cs_i2c_get_scl(FAR struct i2c_bitbang_lower_dev_s *lower)
{
  FAR struct cs_i2c_priv_s *priv = lower->priv;

  return stm32_gpioread(priv->scl);
}

static bool cs_i2c_get_sda(FAR struct i2c_bitbang_lower_dev_s *lower)
{
  FAR struct cs_i2c_priv_s *priv = lower->priv;

  return stm32_gpioread(priv->sda);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cs_i2c_bus_init(void)
{
  static const struct i2c_bitbang_lower_ops_s ops =
  {
    cs_i2c_initialize,
    cs_i2c_set_scl,
    cs_i2c_set_sda,
    cs_i2c_get_scl,
    cs_i2c_get_sda
  };

  /* Idempotent: a second i2c_bitbang_initialize() would allocate a second
   * master with its own spinlock driving the same pins. */

  if (g_cs_i2c_bus != NULL)
    {
      return OK;
    }

  g_cs_i2c_lower.ops  = &ops;
  g_cs_i2c_lower.priv = &g_cs_i2c_priv;

  g_cs_i2c_bus = i2c_bitbang_initialize(&g_cs_i2c_lower);
  if (g_cs_i2c_bus == NULL)
    {
      syslog(LOG_ERR, "CS-I2C: bit-bang master init failed\n");
      return -ENODEV;
    }

  /* Lines should idle high; a low reading means no pull-up or bad wiring. */

  if (!stm32_gpioread(g_cs_i2c_priv.scl) ||
      !stm32_gpioread(g_cs_i2c_priv.sda))
    {
      syslog(LOG_WARNING, "CS-I2C: SCL(D4/PK1)=%d SDA(D5/PA8)=%d stuck low - "
             "check pull-ups and wiring\n",
             stm32_gpioread(g_cs_i2c_priv.scl),
             stm32_gpioread(g_cs_i2c_priv.sda));
    }

  syslog(LOG_INFO, "CS-I2C: software bus ready (SCL=D4/PK1, SDA=D5/PA8, "
         "%ukHz)\n", CS_I2C_FREQ / 1000u);
  return OK;
}

bool cs_i2c_bus_ready(void)
{
  return g_cs_i2c_bus != NULL;
}

int cs_i2c_mux_probe(void)
{
  struct i2c_msg_s msg;
  uint8_t value = 0;

  if (g_cs_i2c_bus == NULL)
    {
      return -ENODEV;
    }

  /* A single register device: write 0 to deselect every downstream channel. */

  msg.frequency = CS_I2C_FREQ;
  msg.addr      = CS_TCA9548A_ADDR;
  msg.flags     = 0;
  msg.buffer    = &value;
  msg.length    = 1;

  return I2C_TRANSFER(g_cs_i2c_bus, &msg, 1);
}

int cs_i2c_transfer(uint8_t channel, FAR struct i2c_msg_s *msgs, int count)
{
  struct i2c_msg_s sel;
  uint8_t value;
  int i;
  int ret;

  if (g_cs_i2c_bus == NULL || msgs == NULL || count <= 0 || channel > 7)
    {
      return -EINVAL;
    }

  /* The bit-bang engine divides by msg->frequency; never let it be zero. */

  for (i = 0; i < count; i++)
    {
      if (msgs[i].frequency == 0)
        {
          msgs[i].frequency = CS_I2C_FREQ;
        }
    }

  /* Selecting the channel and talking to the device must be atomic. */

  nxmutex_lock(&g_cs_i2c_lock);

  value         = (uint8_t)(1u << channel);
  sel.frequency = CS_I2C_FREQ;
  sel.addr      = CS_TCA9548A_ADDR;
  sel.flags     = 0;
  sel.buffer    = &value;
  sel.length    = 1;

  ret = I2C_TRANSFER(g_cs_i2c_bus, &sel, 1);
  if (ret >= 0)
    {
      ret = I2C_TRANSFER(g_cs_i2c_bus, msgs, count);
    }

  nxmutex_unlock(&g_cs_i2c_lock);
  return ret;
}
