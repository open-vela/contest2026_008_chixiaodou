#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include <lvgl/lvgl.h>

#include "charging_station_logo.h"
#include "charging_station_ui.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCREEN_W            480
#define SCREEN_H            272
#define TOPBAR_H            30
#define NAV_H               32
#define TAB_AREA_H          (SCREEN_H - TOPBAR_H - NAV_H)  /* 210 */

/* Palette - light theme */
#define COLOR_BG            lv_color_hex(0xFFFFFF)
#define COLOR_CARD          lv_color_hex(0xF3F4F6)
#define COLOR_CARD2         lv_color_hex(0xE5E7EB)
#define COLOR_THEME         lv_color_hex(0x2563EB)
#define COLOR_OK            lv_color_hex(0x16A34A)
#define COLOR_WARN          lv_color_hex(0xEA580C)
#define COLOR_ERR           lv_color_hex(0xDC2626)
#define COLOR_HEAT          lv_color_hex(0xDC2626)
#define COLOR_TEXT          lv_color_hex(0x1F2937)
#define COLOR_DIM           lv_color_hex(0x6B7280)

/* Iron tab palette - light instrument panel, local to this tab */
#define COLOR_IRON_BG       lv_color_hex(0xFFFFFF)
#define COLOR_IRON_PANEL    lv_color_hex(0xF3F7FB)
#define COLOR_IRON_PANEL2   lv_color_hex(0xE4EEF7)
#define COLOR_IRON_RING_BG  lv_color_hex(0xC9D8E6)
#define COLOR_IRON_ACCENT   lv_color_hex(0x168DCA)
#define COLOR_IRON_TEXT     lv_color_hex(0x1F2937)
#define COLOR_IRON_DIM      lv_color_hex(0x64748B)

/* Charging port protocol badges */
#define BADGE_PD            lv_color_hex(0x2563EB)
#define BADGE_QC            lv_color_hex(0x16A34A)
#define BADGE_FCP           lv_color_hex(0xEA580C)
#define BADGE_AFC           lv_color_hex(0x7C3AED)
#define BADGE_IDLE          lv_color_hex(0x9CA3AF)

/* Iron PID defaults */
#define IRON_TEMP_MIN       100
#define IRON_TEMP_MAX       480
#define IRON_TEMP_STEP      5
#define PID_P_MAX           500.0f
#define PID_I_MAX           100.0f
#define PID_D_MAX           100.0f

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One charging channel (a SW3538 controls 1 C port + 1 A port) */
typedef struct
{
  const char *name;
  /* C port */
  const char *c_proto;
  float       c_volt;
  float       c_amp;
  float       c_watt;
  /* A port */
  const char *a_proto;
  float       a_volt;
  float       a_amp;
  float       a_watt;
  int         die_temp;
  bool        temp_valid;
} chan_t;

/****************************************************************************
 * Private Data - simulated charging channels
 ****************************************************************************/

static chan_t g_chan[4] =
{
  { "CH1", "IDLE", 0.0f, 0.0f, 0.0f, "IDLE", 0.0f, 0.0f, 0.0f, 0, false },
  { "CH2", "IDLE", 0.0f, 0.0f, 0.0f, "IDLE", 0.0f, 0.0f, 0.0f, 0, false },
  { "CH3", "IDLE", 0.0f, 0.0f, 0.0f, "IDLE", 0.0f, 0.0f, 0.0f, 0, false },
  { "CH4", "IDLE", 0.0f, 0.0f, 0.0f, "IDLE", 0.0f, 0.0f, 0.0f, 0, false },
};

/* Iron (soldering) state */
static int   g_iron_temp   = 300;
static int   g_iron_target = 350;
/* User-facing PID values.  They are scaled to the conservative control
 * gains in charging_station_main.c before being passed to iron_set_pid(). */
static float g_iron_p      = 15.0f;
static float g_iron_i      = 0.1f;
static float g_iron_d      = 5.0f;
static bool  g_iron_on     = true;
static int   g_iron_power  = 0;      /* watts, simulated from PID */

/* Weather/time (simulated) */
static int   g_clock_hour  = 14;
static int   g_clock_min   = 23;
static int   g_weather_temp = 27;
static int   g_local_temp  = 29;

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

static void ui_build_topbar(lv_obj_t *scr);
static void ui_build_tabview(lv_obj_t *scr);
static void ui_tab_prepare(lv_obj_t *tab);
static void ui_build_charger(lv_obj_t *parent);
static void ui_build_iron(lv_obj_t *parent);
static void ui_build_info(lv_obj_t *parent);
static void ui_pid_open(lv_obj_t *clicked);
static void ui_pid_open_cb(lv_event_t *e);


/* Widget handles */
static lv_obj_t *g_lbl_time;
static lv_obj_t *g_lbl_weather;
static lv_obj_t *g_lbl_localtemp;

/* Charger widgets */
static lv_obj_t *g_chan_c_lbl[4];
static lv_obj_t *g_chan_a_lbl[4];
static lv_obj_t *g_chan_c_proto_lbl[4];
static lv_obj_t *g_chan_a_proto_lbl[4];
static lv_obj_t *g_chan_temp_lbl[4];
static lv_obj_t *g_input_lbl;
static char g_chan_c_proto[4][12];
static char g_chan_a_proto[4][12];

/* Iron widgets */
static lv_obj_t *g_iron_temp_lbl;
static lv_obj_t *g_iron_pwr_lbl;
static lv_obj_t *g_iron_watt_lbl;
static float     g_iron_watt_hold = 0.0f;  /* last non-zero heater reading */
static lv_obj_t *g_iron_status_lbl;
static lv_obj_t *g_iron_slider;
static lv_obj_t *g_iron_slider_lbl;
static bool g_iron_sleep_active = false;
static int  g_iron_target_eff   = 350;
static lv_obj_t *g_iron_pwr_arc;
static lv_obj_t *g_iron_preset_btn[4];
static lv_obj_t *g_iron_heat_btn;
static lv_obj_t *g_iron_heat_btn_lbl;
static lv_obj_t *g_iron_pid_btn;
static float g_pid_backup[3];

/* Info widgets */
static lv_obj_t *g_info_clock_lbl;
static lv_obj_t *g_info_date_lbl;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static lv_color_t proto_color(const char *proto)
{
  if (proto == NULL || strcmp(proto, "IDLE") == 0 ||
      strcmp(proto, "NONE") == 0)
    {
      return BADGE_IDLE;
    }

  if (strncmp(proto, "PD", 2) == 0) return BADGE_PD;
  if (strncmp(proto, "QC", 2) == 0) return BADGE_QC;
  if (strncmp(proto, "FCP", 3) == 0 || strncmp(proto, "SFCP", 4) == 0)
    return BADGE_FCP;
  if (strncmp(proto, "AFC", 3) == 0) return BADGE_AFC;
  return BADGE_IDLE;
}

/****************************************************************************
 * Name: ui_build_topbar
 *
 * Always-visible status bar: logo, time, weather, local temp.
 ****************************************************************************/

static void ui_build_topbar(lv_obj_t *scr)
{
  lv_obj_t *bar = lv_obj_create(scr);
  lv_obj_t *lbl;

  lv_obj_set_size(bar, SCREEN_W, TOPBAR_H);
  lv_obj_set_pos(bar, 0, 0);
  lv_obj_set_style_bg_color(bar, COLOR_CARD, 0);
  lv_obj_set_style_border_width(bar, 0, 0);
  lv_obj_set_style_pad_all(bar, 2, 0);

  /* Logo */
  lv_obj_t *logo = lv_image_create(bar);
  lv_image_set_src(logo, &charging_station_logo);
  lv_obj_set_size(logo, 22, 22);
  lv_image_set_scale(logo, 88); /* 22px display from the complete 64px asset */
  lv_obj_align(logo, LV_ALIGN_LEFT_MID, 3, 0);

  /* Title */
  lbl = lv_label_create(bar);
  lv_label_set_text(lbl, "Charging Station");
  lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 28, 0);
  lv_obj_set_style_text_color(lbl, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);

  /* Time HH:MM */
  g_lbl_time = lv_label_create(bar);
  lv_label_set_text(g_lbl_time, "14:23");
  lv_obj_align(g_lbl_time, LV_ALIGN_RIGHT_MID, -110, 0);
  lv_obj_set_style_text_color(g_lbl_time, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(g_lbl_time, &lv_font_montserrat_16, 0);

  /* Weather */
  g_lbl_weather = lv_label_create(bar);
  lv_label_set_text(g_lbl_weather, "27C");
  lv_obj_align(g_lbl_weather, LV_ALIGN_RIGHT_MID, -70, 0);
  lv_obj_set_style_text_color(g_lbl_weather, COLOR_WARN, 0);
  lv_obj_set_style_text_font(g_lbl_weather, &lv_font_montserrat_16, 0);

  /* Local temp */
  g_lbl_localtemp = lv_label_create(bar);
  lv_label_set_text(g_lbl_localtemp, "29C");
  lv_obj_align(g_lbl_localtemp, LV_ALIGN_RIGHT_MID, -8, 0);
  lv_obj_set_style_text_color(g_lbl_localtemp, COLOR_OK, 0);
  lv_obj_set_style_text_font(g_lbl_localtemp, &lv_font_montserrat_16, 0);
}

/****************************************************************************
 * Name: ui_charger_badge_create
 *
 * Create a compact protocol badge used by a USB-C or USB-A port row.
 ****************************************************************************/

static lv_obj_t *ui_charger_badge_create(lv_obj_t *parent, int x, int y,
                                         const char *proto)
{
  lv_obj_t *badge = lv_label_create(parent);

  lv_label_set_text(badge, proto);
  lv_obj_set_pos(badge, x, y);
  lv_obj_set_style_text_color(badge, lv_color_white(), 0);
  lv_obj_set_style_text_font(badge, &lv_font_montserrat_14, 0);
  lv_obj_set_style_bg_color(badge, proto_color(proto), 0);
  lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(badge, 5, 0);
  lv_obj_set_style_pad_hor(badge, 4, 0);
  lv_obj_set_style_pad_ver(badge, 1, 0);

  return badge;
}

/****************************************************************************
 * Name: ui_build_charger
 *
 * Tab1 is a four-row output monitor. Each row shows a channel with complete
 * USB-C and USB-A protocol, voltage, current, and power readings.
 ****************************************************************************/

static void ui_build_charger(lv_obj_t *parent)
{
  const int gap = 8;
  const int card_w = (SCREEN_W - 3 * gap) / 2;
  const int card_h = 74;
  lv_obj_t *lbl;
  int i;

  lv_obj_set_style_bg_color(parent, COLOR_IRON_BG, 0);

  /* Top banner doubles as the total-input readout once the INA226 on TCA
   * channel 4 answers; until then it keeps the plain section title. */

  g_input_lbl = lv_label_create(parent);
  lv_label_set_text(g_input_lbl, "CHARGING OUTPUTS");
  lv_obj_set_pos(g_input_lbl, 10, 2);
  lv_obj_set_style_text_color(g_input_lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(g_input_lbl, &lv_font_montserrat_14, 0);

  for (i = 0; i < 4; i++)
    {
      int x = gap + (i % 2) * (card_w + gap);
      int y = 20 + (i / 2) * (card_h + gap);
      lv_obj_t *card = lv_obj_create(parent);

      lv_obj_set_size(card, card_w, card_h);
      lv_obj_set_pos(card, x, y);
      lv_obj_set_style_bg_color(card, COLOR_IRON_PANEL, 0);
      lv_obj_set_style_radius(card, 10, 0);
      lv_obj_set_style_border_width(card, 0, 0);
      lv_obj_set_style_pad_all(card, 0, 0);
      lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);

      lbl = lv_label_create(card);
      lv_label_set_text(lbl, g_chan[i].name);
      lv_obj_set_pos(lbl, 8, 5);
      lv_obj_set_style_text_color(lbl, COLOR_IRON_TEXT, 0);
      lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);

      g_chan_temp_lbl[i] = lv_label_create(card);
      lv_label_set_text(g_chan_temp_lbl[i], "TEMP N/A");
      lv_obj_align(g_chan_temp_lbl[i], LV_ALIGN_TOP_RIGHT, -7, 5);
      lv_obj_set_style_text_color(g_chan_temp_lbl[i], COLOR_IRON_DIM, 0);
      lv_obj_set_style_text_font(g_chan_temp_lbl[i], &lv_font_montserrat_14, 0);

      lbl = lv_label_create(card);
      lv_label_set_text(lbl, "USB-C");
      lv_obj_set_pos(lbl, 8, 29);
      lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
      lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
      /* The fast-charge protocol is shown once in the channel header so it
       * does not overlap the USB-C/USB-A labels below. */
      g_chan_c_proto_lbl[i] = ui_charger_badge_create(card, 43, 4, "IDLE");
      g_chan_c_lbl[i] = lv_label_create(card);
      lv_label_set_text(g_chan_c_lbl[i], "--.-V --.--A --.-W");
      lv_obj_set_pos(g_chan_c_lbl[i], 87, 29);
      lv_obj_set_style_text_color(g_chan_c_lbl[i], COLOR_IRON_DIM, 0);
      lv_obj_set_style_text_font(g_chan_c_lbl[i], &lv_font_montserrat_14, 0);

      lbl = lv_label_create(card);
      lv_label_set_text(lbl, "USB-A");
      lv_obj_set_pos(lbl, 8, 50);
      lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
      lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
      /* Channel-level protocol shares the header badge; keep this handle
       * for the existing per-port update path without adding another badge. */
      g_chan_a_proto_lbl[i] = g_chan_c_proto_lbl[i];
      g_chan_a_lbl[i] = lv_label_create(card);
      lv_label_set_text(g_chan_a_lbl[i], "--.-V --.--A --.-W");
      lv_obj_set_pos(g_chan_a_lbl[i], 87, 50);
      lv_obj_set_style_text_color(g_chan_a_lbl[i], COLOR_IRON_DIM, 0);
      lv_obj_set_style_text_font(g_chan_a_lbl[i], &lv_font_montserrat_14, 0);
    }
}

/****************************************************************************
 * Name: ui_iron_slider_cb
 *
 * Temperature slider change callback.
 ****************************************************************************/

static void ui_iron_update_heat_button(void)
{
  if (g_iron_heat_btn == NULL || g_iron_heat_btn_lbl == NULL)
    {
      return;
    }

  if (g_iron_on)
    {
      lv_label_set_text(g_iron_heat_btn_lbl, "HEAT ON");
      lv_obj_set_style_bg_color(g_iron_heat_btn, COLOR_IRON_ACCENT, 0);
      lv_obj_set_style_text_color(g_iron_heat_btn_lbl, COLOR_IRON_BG, 0);
    }
  else
    {
      lv_label_set_text(g_iron_heat_btn_lbl, "HEAT OFF");
      lv_obj_set_style_bg_color(g_iron_heat_btn, COLOR_IRON_PANEL2, 0);
      lv_obj_set_style_text_color(g_iron_heat_btn_lbl, COLOR_IRON_TEXT, 0);
    }
}

static void ui_iron_heat_toggle_cb(lv_event_t *e)
{
  g_iron_on = !g_iron_on;
  ui_iron_update_heat_button();
  (void)e;
}

static void ui_iron_update_preset_style(void)
{
  const int presets[4] = { 300, 350, 380, 420 };

  for (int i = 0; i < 4; i++)
    {
      if (g_iron_preset_btn[i] == NULL)
        {
          continue;
        }

      if (g_iron_target == presets[i])
        {
          lv_obj_set_style_bg_color(g_iron_preset_btn[i], COLOR_IRON_ACCENT, 0);
          lv_obj_set_style_text_color(lv_obj_get_child(g_iron_preset_btn[i], 0),
                                      COLOR_IRON_BG, 0);
        }
      else
        {
          lv_obj_set_style_bg_color(g_iron_preset_btn[i], COLOR_IRON_PANEL2, 0);
          lv_obj_set_style_text_color(lv_obj_get_child(g_iron_preset_btn[i], 0),
                                      COLOR_IRON_TEXT, 0);
        }
    }
}

static void ui_iron_set_target(int target)
{
  target = ((target + IRON_TEMP_STEP / 2) / IRON_TEMP_STEP) * IRON_TEMP_STEP;
  if (target > IRON_TEMP_MAX) target = IRON_TEMP_MAX;
  if (target < IRON_TEMP_MIN) target = IRON_TEMP_MIN;

  g_iron_target = target;

  if (g_iron_slider_lbl != NULL)
    {
      lv_label_set_text_fmt(g_iron_slider_lbl, "TARGET %d°C", target);
    }

  ui_iron_update_preset_style();

  if (g_iron_slider != NULL)
    {
      lv_slider_set_value(g_iron_slider, target, LV_ANIM_OFF);
    }
}

static void ui_iron_adjust_cb(lv_event_t *e)
{
  int delta = (int)(intptr_t)lv_event_get_user_data(e);

  ui_iron_set_target(g_iron_target + delta);
}

static void ui_iron_slider_cb(lv_event_t *e)
{
  lv_obj_t *slider = lv_event_get_target(e);

  ui_iron_set_target((int)lv_slider_get_value(slider));
}

/****************************************************************************
 * Name: ui_iron_preset_cb
 *
 * Preset temperature button: set target from user data.
 ****************************************************************************/

static void ui_iron_preset_cb(lv_event_t *e)
{
  int preset = (int)(intptr_t)lv_event_get_user_data(e);

  if (preset < IRON_TEMP_MIN) preset = IRON_TEMP_MIN;
  if (preset > IRON_TEMP_MAX) preset = IRON_TEMP_MAX;

  ui_iron_set_target(preset);
}

/****************************************************************************
 * Name: ui_build_iron
 *
 * Tab2 is a 480 x 177 dark instrument panel. The left side holds the
 * measured/target temperature and adjustment controls. The right side holds
 * a full power dial, with presets directly below it.
 ****************************************************************************/

static void ui_build_iron(lv_obj_t *parent)
{
  const int presets[4] = { 300, 350, 380, 420 };
  lv_obj_t *btn;
  lv_obj_t *dial;
  lv_obj_t *lbl;

  lv_obj_set_style_bg_color(parent, COLOR_IRON_BG, 0);

  /* Left control panel */
  lv_obj_t *panel = lv_obj_create(parent);
  lv_obj_set_size(panel, 248, 165);
  lv_obj_set_pos(panel, 7, 6);
  lv_obj_set_style_bg_color(panel, COLOR_IRON_PANEL, 0);
  lv_obj_set_style_radius(panel, 12, 0);
  lv_obj_set_style_border_width(panel, 0, 0);
  lv_obj_set_style_pad_all(panel, 0, 0);
  lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

  lbl = lv_label_create(panel);
  lv_label_set_text(lbl, "SOLDERING IRON");
  lv_obj_set_pos(lbl, 13, 11);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  g_iron_pid_btn = lv_button_create(panel);
  lv_obj_set_size(g_iron_pid_btn, 80, 24);

  /* Top-right of the panel: the title on the left runs to about x=141, so
   * a fixed x=78 would sit on top of it. */

  lv_obj_align(g_iron_pid_btn, LV_ALIGN_TOP_RIGHT, -6, 7);
  lv_obj_set_style_bg_color(g_iron_pid_btn, COLOR_IRON_PANEL2, 0);
  lv_obj_add_event_cb(g_iron_pid_btn, ui_pid_open_cb, LV_EVENT_CLICKED, NULL);
  lbl = lv_label_create(g_iron_pid_btn);
  lv_label_set_text(lbl, "PID");
  lv_obj_center(lbl);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  g_iron_status_lbl = lv_label_create(panel);
  lv_label_set_text(g_iron_status_lbl, "HEATING");
  lv_obj_set_pos(g_iron_status_lbl, 168, 10);
  lv_obj_set_style_text_color(g_iron_status_lbl, COLOR_HEAT, 0);
  lv_obj_set_style_text_font(g_iron_status_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_bg_color(g_iron_status_lbl, COLOR_IRON_PANEL2, 0);
  lv_obj_set_style_bg_opa(g_iron_status_lbl, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(g_iron_status_lbl, 6, 0);
  lv_obj_set_style_pad_hor(g_iron_status_lbl, 5, 0);
  lv_obj_set_style_pad_ver(g_iron_status_lbl, 2, 0);

  g_iron_temp_lbl = lv_label_create(panel);
  lv_label_set_text_fmt(g_iron_temp_lbl, "%d", g_iron_temp);
  lv_obj_set_pos(g_iron_temp_lbl, 13, 37);
  lv_obj_set_style_text_color(g_iron_temp_lbl, COLOR_IRON_ACCENT, 0);
  lv_obj_set_style_text_font(g_iron_temp_lbl, &lv_font_montserrat_28, 0);

  lbl = lv_label_create(panel);
  lv_label_set_text(lbl, "°C");
  lv_obj_set_pos(lbl, 89, 51);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);

  lbl = lv_label_create(panel);
  lv_label_set_text(lbl, "CURRENT");
  lv_obj_set_pos(lbl, 14, 76);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  g_iron_slider_lbl = lv_label_create(panel);
  lv_label_set_text_fmt(g_iron_slider_lbl, "TARGET %d°C", g_iron_target);
  lv_obj_set_pos(g_iron_slider_lbl, 122, 45);
  lv_obj_set_style_text_color(g_iron_slider_lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(g_iron_slider_lbl, &lv_font_montserrat_16, 0);

  g_iron_slider = lv_slider_create(panel);
  lv_obj_set_size(g_iron_slider, 164, 12);
  lv_obj_set_pos(g_iron_slider, 13, 105);
  lv_slider_set_range(g_iron_slider, IRON_TEMP_MIN, IRON_TEMP_MAX);
  lv_slider_set_value(g_iron_slider, g_iron_target, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(g_iron_slider, COLOR_IRON_ACCENT, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(g_iron_slider, COLOR_IRON_RING_BG, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_iron_slider, COLOR_IRON_TEXT, LV_PART_KNOB);
  lv_obj_add_event_cb(g_iron_slider, ui_iron_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

  lbl = lv_label_create(panel);
  lv_label_set_text(lbl, "100°C");
  lv_obj_set_pos(lbl, 13, 124);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  lbl = lv_label_create(panel);
  lv_label_set_text(lbl, "480°C");
  lv_obj_set_pos(lbl, 128, 124);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  btn = lv_button_create(panel);
  lv_obj_set_size(btn, 28, 28);
  lv_obj_set_pos(btn, 180, 96);
  lv_obj_set_style_bg_color(btn, COLOR_IRON_PANEL2, 0);
  lv_obj_set_style_radius(btn, 7, 0);
  lv_obj_add_event_cb(btn, ui_iron_adjust_cb, LV_EVENT_CLICKED,
                      (void *)(intptr_t)-IRON_TEMP_STEP);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "-");
  lv_obj_center(lbl);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);

  btn = lv_button_create(panel);
  lv_obj_set_size(btn, 28, 28);
  lv_obj_set_pos(btn, 216, 96);
  lv_obj_set_style_bg_color(btn, COLOR_IRON_ACCENT, 0);
  lv_obj_set_style_radius(btn, 7, 0);
  lv_obj_add_event_cb(btn, ui_iron_adjust_cb, LV_EVENT_CLICKED,
                      (void *)(intptr_t)IRON_TEMP_STEP);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "+");
  lv_obj_center(lbl);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_BG, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);

  /* User intent switch; the existing 100 ms task applies this state to
   * iron_set_heater(), while the status badge continues to show feedback. */
  g_iron_heat_btn = lv_button_create(panel);
  lv_obj_set_size(g_iron_heat_btn, 80, 24);
  lv_obj_set_pos(g_iron_heat_btn, 158, 137);
  lv_obj_set_style_radius(g_iron_heat_btn, 7, 0);
  lv_obj_add_event_cb(g_iron_heat_btn, ui_iron_heat_toggle_cb,
                      LV_EVENT_CLICKED, NULL);
  g_iron_heat_btn_lbl = lv_label_create(g_iron_heat_btn);
  lv_obj_center(g_iron_heat_btn_lbl);
  lv_obj_set_style_text_font(g_iron_heat_btn_lbl, &lv_font_montserrat_14, 0);
  ui_iron_update_heat_button();

  /* Right complete power dial */
  dial = lv_obj_create(parent);
  lv_obj_set_size(dial, 112, 112);
  lv_obj_set_pos(dial, 345, 4);
  lv_obj_set_style_bg_color(dial, COLOR_IRON_PANEL, 0);
  lv_obj_set_style_radius(dial, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(dial, 0, 0);
  lv_obj_set_style_pad_all(dial, 0, 0);
  lv_obj_remove_flag(dial, LV_OBJ_FLAG_SCROLLABLE);

  g_iron_pwr_arc = lv_arc_create(dial);
  lv_obj_set_size(g_iron_pwr_arc, 100, 100);
  lv_obj_center(g_iron_pwr_arc);
  lv_arc_set_rotation(g_iron_pwr_arc, 270);
  lv_arc_set_bg_angles(g_iron_pwr_arc, 0, 360);
  lv_arc_set_range(g_iron_pwr_arc, 0, 100);
  lv_arc_set_value(g_iron_pwr_arc, 0);
  lv_obj_set_style_arc_color(g_iron_pwr_arc, COLOR_IRON_ACCENT,
                             LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(g_iron_pwr_arc, 9, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(g_iron_pwr_arc, COLOR_IRON_RING_BG, LV_PART_MAIN);
  lv_obj_set_style_arc_width(g_iron_pwr_arc, 9, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(g_iron_pwr_arc, true, LV_PART_INDICATOR);
  lv_obj_set_style_opa(g_iron_pwr_arc, LV_OPA_TRANSP, LV_PART_KNOB);
  lv_obj_remove_flag(g_iron_pwr_arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(g_iron_pwr_arc, LV_OBJ_FLAG_SCROLLABLE);

  g_iron_pwr_lbl = lv_label_create(dial);
  lv_label_set_text(g_iron_pwr_lbl, "0%");
  lv_obj_align(g_iron_pwr_lbl, LV_ALIGN_CENTER, 0, -8);
  lv_obj_set_style_text_color(g_iron_pwr_lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(g_iron_pwr_lbl, &lv_font_montserrat_20, 0);

  lbl = lv_label_create(dial);
  lv_label_set_text(lbl, "POWER");
  lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 16);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  /* Real heater power measured by the INA226 on TCA channel 5.  The duty
   * percentage above is what the PID asked for; this is what the heater
   * actually drew. */

  g_iron_watt_lbl = lv_label_create(dial);
  lv_label_set_text(g_iron_watt_lbl, "--W");
  lv_obj_align(g_iron_watt_lbl, LV_ALIGN_CENTER, 0, 34);
  lv_obj_set_style_text_color(g_iron_watt_lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(g_iron_watt_lbl, &lv_font_montserrat_14, 0);

  /* Right presets directly below the dial */
  for (int i = 0; i < 4; i++)
    {
      int px = 270 + (i % 2) * 96;
      int py = 122 + (i / 2) * 24;

      btn = lv_button_create(parent);
      lv_obj_set_size(btn, 88, 21);
      lv_obj_set_pos(btn, px, py);
      g_iron_preset_btn[i] = btn;
      lv_obj_set_style_bg_color(btn, COLOR_IRON_PANEL2, 0);
      lv_obj_set_style_radius(btn, 6, 0);
      lv_obj_set_style_border_width(btn, 1, 0);
      lv_obj_set_style_border_color(btn, COLOR_IRON_RING_BG, 0);
      lv_obj_add_event_cb(btn, ui_iron_preset_cb, LV_EVENT_CLICKED,
                          (void *)(intptr_t)presets[i]);
      lbl = lv_label_create(btn);
      lv_label_set_text_fmt(lbl, "%d°C", presets[i]);
      lv_obj_center(lbl);
      lv_obj_set_style_text_color(lbl, COLOR_IRON_TEXT, 0);
      lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    }

  ui_iron_update_preset_style();
}

/****************************************************************************
 * Name: ui_build_info
 *
 * Tab3 is a 480 x 177 environment dashboard: time and date on the left,
 * with weather, local temperature, and humidity cards on the right.
 ****************************************************************************/

static void ui_build_info(lv_obj_t *parent)
{
  lv_obj_t *card;
  lv_obj_t *lbl;

  lv_obj_set_style_bg_color(parent, COLOR_IRON_BG, 0);

  /* Main time and date card */
  card = lv_obj_create(parent);
  lv_obj_set_size(card, 224, 165);
  lv_obj_set_pos(card, 8, 6);
  lv_obj_set_style_bg_color(card, COLOR_IRON_PANEL, 0);
  lv_obj_set_style_radius(card, 12, 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_style_pad_all(card, 0, 0);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lbl = lv_label_create(card);
  lv_label_set_text(lbl, "ENVIRONMENT");
  lv_obj_set_pos(lbl, 14, 12);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  lbl = lv_label_create(card);
  lv_label_set_text(lbl, "LOCAL TIME");
  lv_obj_set_pos(lbl, 14, 42);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  g_info_clock_lbl = lv_label_create(card);
  lv_label_set_text_fmt(g_info_clock_lbl, "%02d:%02d", g_clock_hour,
                        g_clock_min);
  lv_obj_set_pos(g_info_clock_lbl, 14, 61);
  lv_obj_set_style_text_color(g_info_clock_lbl, COLOR_IRON_ACCENT, 0);
  lv_obj_set_style_text_font(g_info_clock_lbl, &lv_font_montserrat_28, 0);

  lbl = lv_label_create(card);
  lv_label_set_text(lbl, "DATE");
  lv_obj_set_pos(lbl, 14, 111);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  g_info_date_lbl = lv_label_create(card);
  lv_label_set_text(g_info_date_lbl, "2026-08-31  MONDAY");
  lv_obj_set_pos(g_info_date_lbl, 14, 132);
  lv_obj_set_style_text_color(g_info_date_lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(g_info_date_lbl, &lv_font_montserrat_14, 0);

  /* Weather card */
  card = lv_obj_create(parent);
  lv_obj_set_size(card, 232, 78);
  lv_obj_set_pos(card, 240, 6);
  lv_obj_set_style_bg_color(card, COLOR_IRON_PANEL, 0);
  lv_obj_set_style_radius(card, 12, 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_style_pad_all(card, 0, 0);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lbl = lv_label_create(card);
  lv_label_set_text(lbl, "WEATHER");
  lv_obj_set_pos(lbl, 12, 10);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  lbl = lv_label_create(card);
  lv_label_set_text(lbl, "SUNNY / NINGBO");
  lv_obj_set_pos(lbl, 12, 34);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  lbl = lv_label_create(card);
  lv_label_set_text_fmt(lbl, "%d°C", g_weather_temp);
  lv_obj_set_pos(lbl, 160, 30);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_ACCENT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);

  /* Local temperature card */
  card = lv_obj_create(parent);
  lv_obj_set_size(card, 112, 79);
  lv_obj_set_pos(card, 240, 92);
  lv_obj_set_style_bg_color(card, COLOR_IRON_PANEL2, 0);
  lv_obj_set_style_radius(card, 10, 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_style_pad_all(card, 0, 0);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lbl = lv_label_create(card);
  lv_label_set_text(lbl, "LOCAL TEMP");
  lv_obj_set_pos(lbl, 10, 10);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  lbl = lv_label_create(card);
  lv_label_set_text_fmt(lbl, "%d°C", g_local_temp);
  lv_obj_set_pos(lbl, 10, 37);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);

  /* Humidity card */
  card = lv_obj_create(parent);
  lv_obj_set_size(card, 112, 79);
  lv_obj_set_pos(card, 360, 92);
  lv_obj_set_style_bg_color(card, COLOR_IRON_PANEL2, 0);
  lv_obj_set_style_radius(card, 10, 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_style_pad_all(card, 0, 0);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lbl = lv_label_create(card);
  lv_label_set_text(lbl, "HUMIDITY");
  lv_obj_set_pos(lbl, 10, 10);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_DIM, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  lbl = lv_label_create(card);
  lv_label_set_text(lbl, "62%");
  lv_obj_set_pos(lbl, 10, 37);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
}

/****************************************************************************
 * Name: ui_pid_spinbox_cb
 *
 * PID spinbox value changes.
 ****************************************************************************/

struct ui_pid_value_s
{
  lv_obj_t *label;
  int value;
};

struct ui_pid_step_s
{
  lv_obj_t *label;
  int row;
  bool increment;
};

static struct ui_pid_value_s g_pid_value[3];
static struct ui_pid_step_s g_pid_step[6];

static void ui_pid_value_refresh(int row)
{
  if (g_pid_value[row].label == NULL) return;
  if (row == 1)
    lv_label_set_text_fmt(g_pid_value[row].label, "%d.%d",
                          g_pid_value[row].value / 10,
                          g_pid_value[row].value % 10);
  else
    lv_label_set_text_fmt(g_pid_value[row].label, "%d",
                          g_pid_value[row].value);
}

static void ui_pid_step_cb(lv_event_t *e)
{
  struct ui_pid_step_s *step = lv_event_get_user_data(e);
  int max;
  int delta;

  if (step == NULL || step->label == NULL) return;

  switch (step->row)
    {
      case 0: /* P: 0..100 step 5 */
        max = 100; delta = 5;
        break;
      case 1: /* I: 0.0..5.0 step 0.1 */
        max = 50; delta = 1;
        break;
      default: /* D: 0..100 step 5 */
        max = 100; delta = 5;
        break;
    }

  g_pid_value[step->row].value += step->increment ? delta : -delta;
  if (g_pid_value[step->row].value < 0)
    g_pid_value[step->row].value = 0;
  if (g_pid_value[step->row].value > max) g_pid_value[step->row].value = max;
  if (step->row == 0) g_iron_p = (float)g_pid_value[0].value;
  else if (step->row == 1) g_iron_i = (float)g_pid_value[1].value / 10.0f;
  else g_iron_d = (float)g_pid_value[2].value;
  ui_pid_value_refresh(step->row);
}

/****************************************************************************
 * Name: ui_pid_spin_make
 *
 * Create one PID spinbox row.
 ****************************************************************************/

static lv_obj_t *ui_pid_spin_make(lv_obj_t *parent, const char *name,
                                  float def, int x, int y, int row)
{
  lv_obj_t *title = lv_label_create(parent);
  lv_obj_t *value_label = lv_label_create(parent);
  lv_obj_t *btn;
  int raw = row == 1 ? (int)(def * 10.0f + 0.5f) : (int)(def + 0.5f);
  int max = row == 1 ? 50 : 100;

  g_pid_value[row].label = value_label;
  g_pid_value[row].value = raw < 0 ? 0 : (raw > max ? max : raw);
  lv_label_set_text(title, name);
  lv_obj_set_pos(title, x, y);
  lv_obj_set_style_text_color(title, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
  lv_obj_set_size(value_label, 90, 24);
  lv_obj_set_pos(value_label, x + 58, y - 2);
  lv_obj_set_style_bg_color(value_label, COLOR_CARD2, 0);
  lv_obj_set_style_bg_opa(value_label, LV_OPA_COVER, 0);
  lv_obj_set_style_text_align(value_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(value_label, &lv_font_montserrat_16, 0);
  ui_pid_value_refresh(row);

  btn = lv_button_create(parent);
  lv_obj_set_size(btn, 24, 24); lv_obj_set_pos(btn, x + 30, y - 2);
  lv_label_set_text(lv_label_create(btn), "+"); lv_obj_center(lv_obj_get_child(btn, 0));
  g_pid_step[row * 2] = (struct ui_pid_step_s){ value_label, row, true };
  lv_obj_add_event_cb(btn, ui_pid_step_cb, LV_EVENT_CLICKED,
                      &g_pid_step[row * 2]);
  lv_obj_add_event_cb(btn, ui_pid_step_cb, LV_EVENT_LONG_PRESSED_REPEAT,
                      &g_pid_step[row * 2]);

  btn = lv_button_create(parent);
  lv_obj_set_size(btn, 24, 24); lv_obj_set_pos(btn, x + 152, y - 2);
  lv_label_set_text(lv_label_create(btn), "-"); lv_obj_center(lv_obj_get_child(btn, 0));
  g_pid_step[row * 2 + 1] = (struct ui_pid_step_s){ value_label, row, false };
  lv_obj_add_event_cb(btn, ui_pid_step_cb, LV_EVENT_CLICKED,
                      &g_pid_step[row * 2 + 1]);
  lv_obj_add_event_cb(btn, ui_pid_step_cb, LV_EVENT_LONG_PRESSED_REPEAT,
                      &g_pid_step[row * 2 + 1]);
  return value_label;
}

static void ui_pid_apply_cb(lv_event_t *e)
{
  lv_obj_t *mbox = lv_event_get_user_data(e);
  lv_obj_add_flag(mbox, LV_OBJ_FLAG_HIDDEN);
}

static void ui_pid_cancel_cb(lv_event_t *e)
{
  lv_obj_t *mbox = lv_event_get_user_data(e);
  g_iron_p = g_pid_backup[0];
  g_iron_i = g_pid_backup[1];
  g_iron_d = g_pid_backup[2];
  lv_obj_add_flag(mbox, LV_OBJ_FLAG_HIDDEN);
}

static void ui_pid_open_cb(lv_event_t *e)
{
  ui_pid_open(lv_event_get_target(e));
}

static void ui_pid_open(lv_obj_t *clicked)
{
  lv_obj_t *mbox;
  lv_obj_t *content;
  lv_obj_t *btn;
  lv_obj_t *lbl;

  g_pid_backup[0] = g_iron_p;
  g_pid_backup[1] = g_iron_i;
  g_pid_backup[2] = g_iron_d;

  mbox = lv_obj_create(lv_scr_act());
  lv_obj_set_size(mbox, 300, 200);
  lv_obj_center(mbox);
  lv_obj_set_style_bg_color(mbox, COLOR_BG, 0);
  lv_obj_set_style_border_color(mbox, COLOR_IRON_ACCENT, 0);
  lv_obj_set_style_border_width(mbox, 2, 0);
  lv_obj_set_style_radius(mbox, 12, 0);
  lv_obj_set_style_pad_all(mbox, 8, 0);
  lv_obj_add_flag(mbox, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(mbox, LV_OBJ_FLAG_SCROLLABLE);

  lbl = lv_label_create(mbox);
  lv_label_set_text(lbl, "PID PARAMETERS");
  lv_obj_set_pos(lbl, 8, 4);
  lv_obj_set_style_text_color(lbl, COLOR_IRON_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);

  content = lv_obj_create(mbox);
  lv_obj_set_size(content, 270, 138);
  lv_obj_set_pos(content, 8, 30);
  lv_obj_set_style_bg_opa(content, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(content, 0, 0);
  lv_obj_set_style_pad_all(content, 0, 0);
  lv_obj_remove_flag(content, LV_OBJ_FLAG_SCROLLABLE);

  ui_pid_spin_make(content, "P", g_iron_p, 8, 6, 0);
  ui_pid_spin_make(content, "I", g_iron_i, 8, 40, 1);
  ui_pid_spin_make(content, "D", g_iron_d, 8, 74, 2);

  btn = lv_button_create(mbox);
  lv_obj_set_size(btn, 80, 24);
  lv_obj_set_pos(btn, 105, 167);
  lv_obj_set_style_bg_color(btn, COLOR_IRON_PANEL2, 0);
  lv_obj_add_event_cb(btn, ui_pid_cancel_cb, LV_EVENT_CLICKED, mbox);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "CANCEL");
  lv_obj_center(lbl);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);

  btn = lv_button_create(mbox);
  lv_obj_set_size(btn, 80, 24);
  lv_obj_set_pos(btn, 195, 167);
  lv_obj_set_style_bg_color(btn, COLOR_IRON_ACCENT, 0);
  lv_obj_add_event_cb(btn, ui_pid_apply_cb, LV_EVENT_CLICKED, mbox);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "APPLY");
  lv_obj_center(lbl);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  (void)clicked;
}


/****************************************************************************
 * Name: cs_ui_set_ch1_real
 *
 * Feed real SW3538 readings into channel 1 display.
 * Volt in volts, amp in amps. If online is false, port shows idle.
 ****************************************************************************/

void cs_ui_set_channel_ports(int channel, float c_volt, float c_amp,
                             float a_volt, float a_amp, const char *proto,
                             int temp_c, bool temp_valid)
{
  chan_t *chan;

  if (channel < 0 || channel >= 4)
    {
      return;
    }

  chan = &g_chan[channel];
  snprintf(g_chan_c_proto[channel], sizeof(g_chan_c_proto[channel]), "%s",
           proto != NULL ? proto : "IDLE");
  snprintf(g_chan_a_proto[channel], sizeof(g_chan_a_proto[channel]), "%s",
           proto != NULL ? proto : "IDLE");

  if (c_amp > 0.01f)
    {
      chan->c_proto = g_chan_c_proto[channel];
      chan->c_volt = c_volt;
      chan->c_amp = c_amp;
      chan->c_watt = c_volt * c_amp;
    }
  else
    {
      chan->c_proto = "IDLE";
      chan->c_volt = 0.0f;
      chan->c_amp = 0.0f;
      chan->c_watt = 0.0f;
    }

  if (a_amp > 0.01f)
    {
      chan->a_proto = g_chan_a_proto[channel];
      chan->a_volt = a_volt;
      chan->a_amp = a_amp;
      chan->a_watt = a_volt * a_amp;
    }
  else
    {
      chan->a_proto = "IDLE";
      chan->a_volt = 0.0f;
      chan->a_amp = 0.0f;
      chan->a_watt = 0.0f;
    }

  chan->die_temp = temp_c;
  chan->temp_valid = temp_valid;
}

void cs_ui_set_channel_online(int channel, bool c_online, bool a_online)
{
  if (channel < 0 || channel >= 4)
    {
      return;
    }

  if (!c_online)
    {
      g_chan[channel].c_proto = "IDLE";
      g_chan[channel].c_volt = 0.0f;
      g_chan[channel].c_amp = 0.0f;
      g_chan[channel].c_watt = 0.0f;
    }

  if (!a_online)
    {
      g_chan[channel].a_proto = "IDLE";
      g_chan[channel].a_volt = 0.0f;
      g_chan[channel].a_amp = 0.0f;
      g_chan[channel].a_watt = 0.0f;
    }
}

void cs_ui_set_ch1_real(float volt, float amp, const char *proto)
{
  cs_ui_set_channel_ports(0, volt, amp, 0.0f, 0.0f, proto, 0, false);
}

void cs_ui_set_ch1_ports(float c_volt, float c_amp,
                         float a_volt, float a_amp,
                         const char *proto)
{
  cs_ui_set_channel_ports(0, c_volt, c_amp, a_volt, a_amp, proto, 0, false);
}

/****************************************************************************
 * Name: cs_ui_set_ch1_online
 *
 * Mark channel 1 as online/offline.
 ****************************************************************************/

void cs_ui_set_ch1_online(bool online)
{
  cs_ui_set_channel_online(0, online, online);
}

/****************************************************************************
 * Name: cs_ui_data_tick
 *
 * Simulate data changes. Called periodically.
 ****************************************************************************/

void cs_ui_data_tick(void)
{
  static int tick = 0;
  int i;
  int power;

  tick++;

  /* Clock: advance 1 min every 60 ticks (tick = 1s) */
  g_clock_min++;
  if (g_clock_min >= 60)
    {
      g_clock_min = 0;
      g_clock_hour = (g_clock_hour + 1) % 24;
    }

  lv_label_set_text_fmt(g_lbl_time, "%02d:%02d", g_clock_hour, g_clock_min);
  lv_label_set_text_fmt(g_info_clock_lbl, "%02d:%02d", g_clock_hour, g_clock_min);

  /* Update charger port labels.
   * Only channel 0 C-port carries real SW3538 data (fed by main).
   * No simulated fluctuation: everything not backed by real data
   * shows "N/A".
   */
  for (i = 0; i < 4; i++)
    {
      if (strcmp(g_chan[i].c_proto, "IDLE") != 0)
        {
          lv_label_set_text_fmt(g_chan_c_lbl[i], "%.1fV %.2fA %.1fW",
                                g_chan[i].c_volt, g_chan[i].c_amp,
                                g_chan[i].c_watt);
          lv_obj_set_style_text_color(g_chan_c_lbl[i], COLOR_IRON_TEXT, 0);
        }
      else
        {
          lv_label_set_text(g_chan_c_lbl[i], "--.-V  --.--A  --.-W");
          lv_obj_set_style_text_color(g_chan_c_lbl[i], COLOR_IRON_DIM, 0);
        }

      lv_label_set_text(g_chan_c_proto_lbl[i],
                        strcmp(g_chan[i].c_proto, "IDLE") != 0 ?
                        g_chan[i].c_proto : g_chan[i].a_proto);
      lv_obj_set_style_bg_color(g_chan_c_proto_lbl[i],
                                proto_color(strcmp(g_chan[i].c_proto,
                                                   "IDLE") != 0 ?
                                            g_chan[i].c_proto :
                                            g_chan[i].a_proto), 0);

      if (strcmp(g_chan[i].a_proto, "IDLE") != 0)
        {
          lv_label_set_text_fmt(g_chan_a_lbl[i], "%.1fV %.2fA %.1fW",
                                g_chan[i].a_volt, g_chan[i].a_amp,
                                g_chan[i].a_watt);
          lv_obj_set_style_text_color(g_chan_a_lbl[i], COLOR_IRON_TEXT, 0);
        }
      else
        {
          lv_label_set_text(g_chan_a_lbl[i], "--.-V  --.--A  --.-W");
          lv_obj_set_style_text_color(g_chan_a_lbl[i], COLOR_IRON_DIM, 0);
        }

      if (g_chan[i].temp_valid)
        {
          lv_label_set_text_fmt(g_chan_temp_lbl[i], "TEMP %d°C",
                                g_chan[i].die_temp);
          lv_obj_set_style_text_color(g_chan_temp_lbl[i], COLOR_WARN, 0);
        }
      else
        {
          lv_label_set_text(g_chan_temp_lbl[i], "TEMP N/A");
          lv_obj_set_style_text_color(g_chan_temp_lbl[i], COLOR_IRON_DIM, 0);
        }
    }

  /* Iron display: values updated from real hardware via
   * cs_ui_set_iron_real(). Here we only render + advance charts.
   */
  lv_label_set_text_fmt(g_iron_temp_lbl, "%d", g_iron_temp);
  lv_label_set_text_fmt(g_iron_pwr_lbl, "%.0f%%", (float)g_iron_power);

  /* Status: the parked iron does not heat at all, so it reports SLEEP
   * (orange) even when the heat switch is on.  Otherwise: OFF (gray),
   * READY (green) when within ~5C of target, HEATING (red) otherwise. */
  if (g_iron_sleep_active)
    {
      lv_label_set_text(g_iron_status_lbl, "SLEEP");
      lv_obj_set_style_text_color(g_iron_status_lbl, COLOR_WARN, 0);
    }
  else if (!g_iron_on)
    {
      lv_label_set_text(g_iron_status_lbl, "OFF");
      lv_obj_set_style_text_color(g_iron_status_lbl, COLOR_DIM, 0);
    }
  else if (abs(g_iron_target_eff - g_iron_temp) <= 5)
    {
      lv_label_set_text(g_iron_status_lbl, "READY");
      lv_obj_set_style_text_color(g_iron_status_lbl, COLOR_OK, 0);
    }
  else
    {
      lv_label_set_text(g_iron_status_lbl, "HEATING");
      lv_obj_set_style_text_color(g_iron_status_lbl, COLOR_HEAT, 0);
    }

  ui_iron_update_heat_button();

  /* Target readout.  ui_iron_set_target() also writes it on user input;
   * refreshing it here keeps it correct across park/pickup transitions. */

  lv_label_set_text_fmt(g_iron_slider_lbl, "TARGET %d°C", g_iron_target_eff);

  /* Power ring: ring shows power %, clamped to its configured range. */
  power = g_iron_power;
  if (power < 0) power = 0;
  if (power > 100) power = 100;
  lv_arc_set_value(g_iron_pwr_arc, power);
  lv_label_set_text_fmt(g_iron_pwr_lbl, "%d%%", power);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs_ui_set_iron_temp
 ****************************************************************************/

void cs_ui_set_iron_temp(int temp_c)
{
  if (temp_c < IRON_TEMP_MIN) temp_c = IRON_TEMP_MIN;
  if (temp_c > IRON_TEMP_MAX) temp_c = IRON_TEMP_MAX;
  g_iron_target = temp_c;
}

/****************************************************************************
 * Name: cs_ui_set_iron_real
 *
 * Feed real soldering iron readings (MAX6675 temp + PID power) to the
 * UI. power_pct is a percentage; display as W-ish for simplicity.
 ****************************************************************************/

void cs_ui_set_iron_real(int temp_c, float power_pct,
                         bool heating, bool sensor_ok)
{
  g_iron_temp  = temp_c;
  g_iron_power = (int)power_pct;
  g_iron_on    = heating;
}

/****************************************************************************
 * Name: cs_ui_set_input_power
 *
 * Total input rail measured by the INA226 behind TCA9548A channel 4.
 * valid=false means the monitor is absent or its last read failed, so the
 * banner falls back to the plain section title instead of showing a fake
 * 0 W.
 ****************************************************************************/

void cs_ui_set_input_power(float volt, float amp, float watt, bool valid)
{
  if (g_input_lbl == NULL)
    {
      return;
    }

  if (!valid)
    {
      lv_label_set_text(g_input_lbl, "CHARGING OUTPUTS");
      lv_obj_set_style_text_color(g_input_lbl, COLOR_IRON_DIM, 0);
      return;
    }

  lv_label_set_text_fmt(g_input_lbl, "INPUT  %.1fV  %.2fA  %.0fW",
                        (double)volt, (double)amp, (double)watt);
  lv_obj_set_style_text_color(g_input_lbl,
                              (amp > 0.05f) ? COLOR_OK : COLOR_IRON_DIM, 0);
}

/****************************************************************************
 * Name: cs_ui_set_iron_power
 *
 * Heater power measured by the INA226 on TCA9548A channel 5.  The heater is
 * switched by a 0.5 Hz software PWM, so successive samples toggle between
 * the on and off state; that is why this sits next to the PID duty
 * percentage rather than replacing it.
 ****************************************************************************/

void cs_ui_set_iron_power(float volt, float amp, float watt, bool valid)
{
  if (g_iron_watt_lbl == NULL)
    {
      return;
    }

  if (!valid)
    {
      g_iron_watt_hold = 0.0f;
      lv_label_set_text(g_iron_watt_lbl, "--W");
      lv_obj_set_style_text_color(g_iron_watt_lbl, COLOR_IRON_DIM, 0);
      return;
    }

  /* The heater runs on a 0.5 Hz software PWM (2s cycle) while the monitor is
   * polled every 500ms, so the sampling phase lands on the on and off halves
   * of the cycle in turn.  Showing the raw sample would make the readout
   * alternate between the real wattage and 0 W.  While the PID is asking
   * for heat, hold the last non-zero reading; when it stops asking, the
   * held value is dropped so a genuinely idle tip reads 0 W.
   *
   * Note this therefore shows the power drawn while the heater is ON, not
   * the cycle average -- at a 5% duty the average is 1/20th of what is
   * displayed.  The duty percentage next to it is the other half of the
   * picture. */

  if (watt > 0.5f)
    {
      g_iron_watt_hold = watt;
    }
  else if (g_iron_power <= 0)
    {
      g_iron_watt_hold = 0.0f;
    }

  lv_label_set_text_fmt(g_iron_watt_lbl, "%.0fW", (double)g_iron_watt_hold);
  lv_obj_set_style_text_color(g_iron_watt_lbl,
                              (g_iron_watt_hold > 0.5f) ? COLOR_IRON_ACCENT :
                                                          COLOR_IRON_DIM, 0);
}

/****************************************************************************
 * Name: cs_ui_get_iron_temp
 ****************************************************************************/

int cs_ui_get_iron_temp(void)
{
  return g_iron_target;
}

/****************************************************************************
 * Name: cs_ui_get_iron_target
 ****************************************************************************/

int cs_ui_get_iron_target(void)
{
  return g_iron_target;
}

/****************************************************************************
 * Name: cs_ui_get_iron_on
 ****************************************************************************/

bool cs_ui_get_iron_on(void)
{
  return g_iron_on;
}

/****************************************************************************
 * Name: cs_ui_set_iron_pid
 ****************************************************************************/

void cs_ui_set_iron_pid(float p, float i, float d)
{
  if (p < 0.0f) p = 0.0f;
  if (i < 0.0f) i = 0.0f;
  if (d < 0.0f) d = 0.0f;
  if (p > 100.0f) p = 100.0f;
  if (i > 5.0f) i = 5.0f;
  if (d > 100.0f) d = 100.0f;
  g_iron_p = p;
  g_iron_i = i;
  g_iron_d = d;
}

void cs_ui_get_iron_pid(float *p, float *i, float *d)
{
  if (p != NULL) *p = g_iron_p;
  if (i != NULL) *i = g_iron_i;
  if (d != NULL) *d = g_iron_d;
}

/****************************************************************************
 * Name: cs_ui_set_iron_sleep
 ****************************************************************************/

void cs_ui_set_iron_sleep(bool sleep_active, int target_eff)
{
  g_iron_sleep_active = sleep_active;

  /* Always record the effective target.  The caller passes target_c while
   * awake and target_eff_c while parked, and the READY/HEATING decision
   * compares against it.  Keeping a stale value across a park/pickup
   * transition would make a tip that has reached its target report
   * HEATING forever. */

  g_iron_target_eff = target_eff;
}

/****************************************************************************
 * Name: cs_ui_init
 ****************************************************************************/

int cs_ui_init(lv_obj_t *scr)
{
  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);

  ui_build_topbar(scr);
  ui_build_tabview(scr);

  /* Update clock once at startup */
  lv_label_set_text_fmt(g_lbl_time, "%02d:%02d", g_clock_hour, g_clock_min);

  return 0;
}

/****************************************************************************
 * Name: ui_build_tabview
 ****************************************************************************/

static void ui_build_tabview(lv_obj_t *scr)
{
  lv_obj_t *tv = lv_tabview_create(scr);
  lv_obj_t *tab;

  lv_obj_set_size(tv, SCREEN_W, SCREEN_H - TOPBAR_H);
  lv_obj_set_pos(tv, 0, TOPBAR_H);
  lv_obj_set_style_bg_color(tv, COLOR_BG, 0);

  /* Charger tab */
  tab = lv_tabview_add_tab(tv, "Charger");
  ui_tab_prepare(tab);
  ui_build_charger(tab);

  /* Iron tab */
  tab = lv_tabview_add_tab(tv, "Iron");
  ui_tab_prepare(tab);
  ui_build_iron(tab);

  /* Info tab */
  tab = lv_tabview_add_tab(tv, "Info");
  ui_tab_prepare(tab);
  ui_build_info(tab);
}

/* Helper: strip default obj padding/border so tab content uses the
 * full available area without unintended scroll. Call on each tab page.
 */
static void ui_tab_prepare(lv_obj_t *tab)
{
  lv_obj_set_style_pad_all(tab, 0, 0);
  lv_obj_set_style_border_width(tab, 0, 0);
  lv_obj_remove_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
}
