/*
 * External power control that brings the OLED back with it.
 *
 * ZMK's &ext_power cuts VCC to the panel, but nothing re-initializes the
 * SSD1306 when the rail returns, so the screen stays dark until a reset.
 * See https://github.com/zmkfirmware/zmk/issues/674 (open since 2021).
 *
 * Re-sending the init sequence on its own is not enough: while the panel is
 * unpowered it drags the bus down and the nRF TWI peripheral latches an error,
 * so the first write fails. The controller has to be torn down and re-inited
 * first. Zephyr's PM hooks for i2c_nrfx_twi do exactly that via public API.
 */

#define DT_DRV_COMPAT zmk_behavior_display_power

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>

#include <drivers/behavior.h>
#include <drivers/ext_power.h>
#include <dt-bindings/zmk/ext_power.h>

#include <lvgl.h>
#include <zmk/display.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define DISP DT_CHOSEN(zephyr_display)
#define EXT_PWR DT_INST(0, zmk_ext_power_generic)

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DISP, solomon_ssd1306fb),
             "This behavior only knows how to re-initialize a solomon,ssd1306fb panel");
BUILD_ASSERT(DT_ON_BUS(DISP, i2c), "This behavior only supports an I2C-attached panel");

static const struct device *const display = DEVICE_DT_GET(DISP);
static const struct i2c_dt_spec panel = I2C_DT_SPEC_GET(DISP);
static const struct device *const ext_power = DEVICE_DT_GET(EXT_PWR);

/* Control byte saying every following byte is a command, not pixel data. */
#define SSD1306_CMD_STREAM 0x00
#define SSD1306_PUMP_VOLTAGE 0x33

/*
 * Mirrors ssd1306_init_device() in the Zephyr driver, which is static and so
 * unreachable from here. Every value is read from the same devicetree
 * properties the driver itself reads, so the two cannot drift apart.
 */
static int ssd1306_reinit(void) {
    uint8_t cmds[] = {
        0xAE,                                            /* display off */
        0xD5, (0x8 << 4) | 0x0,                          /* clock divide / osc freq */
        0xD9, DT_PROP_OR(DISP, prechargep, 0x22),        /* pre-charge period */
        0xDB, 0x20,                                      /* VCOMH deselect level */
        0x40,                                            /* start line 0 */
        0xD3, DT_PROP_OR(DISP, display_offset, 0),       /* display offset */
        0xDA, DT_PROP(DISP, com_sequential) ? 0x02 : 0x12,
        0xA8, DT_PROP_OR(DISP, multiplex_ratio, 63),
        DT_PROP(DISP, segment_remap) ? 0xA1 : 0xA0,
        DT_PROP(DISP, com_invdir) ? 0xC8 : 0xC0,
        0x8D, 0x14, SSD1306_PUMP_VOLTAGE,                /* charge pump on */
        0xA4,                                            /* resume display from RAM */
        DT_PROP(DISP, inversion_on) ? 0xA7 : 0xA6,
        0x81, CONFIG_SSD1306_DEFAULT_CONTRAST,
        0xAF,                                            /* display on */
    };

    return i2c_burst_write_dt(&panel, SSD1306_CMD_STREAM, cmds, sizeof(cmds));
}

static void display_power_on_cb(struct k_work *work) {
    int rc = ext_power_enable(ext_power);
    if (rc) {
        LOG_ERR("Failed to enable ext power: %d", rc);
        return;
    }

    k_msleep(CONFIG_ZMK_BEHAVIOR_DISPLAY_POWER_RAIL_SETTLE_MS);

    /* Clear the error the unpowered panel latched into the I2C controller. */
    rc = pm_device_action_run(panel.bus, PM_DEVICE_ACTION_SUSPEND);
    if (rc && rc != -EALREADY) {
        LOG_WRN("Failed to suspend %s: %d", panel.bus->name, rc);
    }
    k_msleep(CONFIG_ZMK_BEHAVIOR_DISPLAY_POWER_BUS_RESET_MS);

    rc = pm_device_action_run(panel.bus, PM_DEVICE_ACTION_RESUME);
    if (rc && rc != -EALREADY) {
        LOG_ERR("Failed to resume %s: %d", panel.bus->name, rc);
        return;
    }
    k_msleep(CONFIG_ZMK_BEHAVIOR_DISPLAY_POWER_BUS_RESET_MS);

    rc = ssd1306_reinit();
    if (rc) {
        LOG_ERR("Failed to re-initialize the panel: %d", rc);
        return;
    }

    display_blanking_off(display);

    if (zmk_display_is_initialized()) {
        /* GDDRAM went away with the rail, so nothing on screen is still valid. */
        lv_obj_invalidate(lv_scr_act());
    } else {
        LOG_WRN("Display never initialized at boot; panel is lit but has no screen to draw");
    }
}

static void display_power_off_cb(struct k_work *work) {
    /* Park the panel before the rail drops out from under it. */
    display_blanking_on(display);

    int rc = ext_power_disable(ext_power);
    if (rc) {
        LOG_ERR("Failed to disable ext power: %d", rc);
    }
}

/*
 * Both run on the display work queue, which is also where lv_task_handler()
 * runs, so they cannot race an LVGL flush mid-sequence.
 */
K_WORK_DEFINE(display_power_on_work, display_power_on_cb);
K_WORK_DEFINE(display_power_off_work, display_power_off_cb);

static int on_keymap_binding_convert_central_state_dependent_params(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    if (binding->param1 == EXT_POWER_TOGGLE_CMD) {
        binding->param1 = ext_power_get(ext_power) > 0 ? EXT_POWER_OFF_CMD : EXT_POWER_ON_CMD;
    }

    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    struct k_work *work;

    switch (binding->param1) {
    case EXT_POWER_OFF_CMD:
        work = &display_power_off_work;
        break;
    case EXT_POWER_ON_CMD:
        work = &display_power_on_work;
        break;
    case EXT_POWER_TOGGLE_CMD:
        work = ext_power_get(ext_power) > 0 ? &display_power_off_work : &display_power_on_work;
        break;
    default:
        LOG_ERR("Unknown ext_power command: %d", binding->param1);
        return -ENOTSUP;
    }

    k_work_submit_to_queue(zmk_display_work_q(), work);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_display_power_driver_api = {
    .binding_convert_central_state_dependent_params =
        on_keymap_binding_convert_central_state_dependent_params,
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_display_power_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
