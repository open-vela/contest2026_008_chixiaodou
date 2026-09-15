# Contest 2026 team 008 charging station
if(CONFIG_LVX_USE_DEMO_CONTEST2026_008_CHARGING_STATION)
  nuttx_add_application(
    NAME charging_station
    SRCS charging_station_main.c charging_station_ui.c sw3538.c iron.c
         cs_i2c_sw.c ina226.c
    STACKSIZE 32768)
endif()
