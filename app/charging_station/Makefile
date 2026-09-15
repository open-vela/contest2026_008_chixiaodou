############################################################################
# Contest 2026 team 008 charging station - Makefile
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = charging_station
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = 32768
MODULE    = $(CONFIG_LVX_USE_DEMO_CONTEST2026_008_CHARGING_STATION)

MAINSRC = charging_station_main.c
CSRCS   = charging_station_ui.c sw3538.c iron.c cs_i2c_sw.c ina226.c

include $(APPDIR)/Application.mk
