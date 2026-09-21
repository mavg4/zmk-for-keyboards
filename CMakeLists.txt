# Sources are added to ZMK's `app` target rather than a library of their own so
# they inherit app's private include dirs (drivers/behavior.h, zmk/display.h).
target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_DISPLAY_POWER app PRIVATE src/behaviors/behavior_display_power.c)
