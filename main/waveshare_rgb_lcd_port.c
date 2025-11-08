/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "waveshare_rgb_lcd_port.h"
#include <math.h>

// VSYNC event callback function
IRAM_ATTR static bool rgb_lcd_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    return lvgl_port_notify_rgb_vsync();
}

#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911
/**
 * @brief I2C master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    // Configure I2C parameters
    i2c_param_config(i2c_master_port, &i2c_conf);

    // Install I2C driver
    return i2c_driver_install(i2c_master_port, i2c_conf.mode, 0, 0, 0);
}

// GPIO initialization
void gpio_init(void)
{
    // Zero-initialize the config structure
    gpio_config_t io_conf = {};
    // Disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // Set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    gpio_config(&io_conf);
}

// Reset the touch screen
void waveshare_esp32_s3_touch_reset()
{
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // Reset the touch screen. It is recommended to reset the touch screen before using it.
    write_buf = 0x2C;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    esp_rom_delay_us(100 * 1000);
    gpio_set_level(GPIO_INPUT_IO_4, 0);
    esp_rom_delay_us(100 * 1000);
    write_buf = 0x2E;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    esp_rom_delay_us(200 * 1000);
}

#endif

// Initialize RGB LCD
esp_err_t waveshare_esp32_s3_rgb_lcd_init()
{
    ESP_LOGI(TAG, "Install RGB LCD panel driver"); // Log the start of the RGB LCD panel driver installation
    esp_lcd_panel_handle_t panel_handle = NULL; // Declare a handle for the LCD panel
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT, // Set the clock source for the panel
        .timings =  {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ, // Pixel clock frequency
            .h_res = EXAMPLE_LCD_H_RES, // Horizontal resolution
            .v_res = EXAMPLE_LCD_V_RES, // Vertical resolution
            .hsync_pulse_width = 4, // Horizontal sync pulse width
            .hsync_back_porch = 8, // Horizontal back porch
            .hsync_front_porch = 8, // Horizontal front porch
            .vsync_pulse_width = 4, // Vertical sync pulse width
            .vsync_back_porch = 8, // Vertical back porch
            .vsync_front_porch = 8, // Vertical front porch
            .flags = {
                .pclk_active_neg = 1, // Active low pixel clock
            },
        },
        .data_width = EXAMPLE_RGB_DATA_WIDTH, // Data width for RGB
        .bits_per_pixel = EXAMPLE_RGB_BIT_PER_PIXEL, // Bits per pixel
        .num_fbs = LVGL_PORT_LCD_RGB_BUFFER_NUMS, // Number of frame buffers
        .bounce_buffer_size_px = EXAMPLE_RGB_BOUNCE_BUFFER_SIZE, // Bounce buffer size in pixels
        .sram_trans_align = 4, // SRAM transaction alignment
        .psram_trans_align = 64, // PSRAM transaction alignment
        .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC, // GPIO number for horizontal sync
        .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC, // GPIO number for vertical sync
        .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE, // GPIO number for data enable
        .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK, // GPIO number for pixel clock
        .disp_gpio_num = EXAMPLE_LCD_IO_RGB_DISP, // GPIO number for display
        .data_gpio_nums = {
            EXAMPLE_LCD_IO_RGB_DATA0,
            EXAMPLE_LCD_IO_RGB_DATA1,
            EXAMPLE_LCD_IO_RGB_DATA2,
            EXAMPLE_LCD_IO_RGB_DATA3,
            EXAMPLE_LCD_IO_RGB_DATA4,
            EXAMPLE_LCD_IO_RGB_DATA5,
            EXAMPLE_LCD_IO_RGB_DATA6,
            EXAMPLE_LCD_IO_RGB_DATA7,
            EXAMPLE_LCD_IO_RGB_DATA8,
            EXAMPLE_LCD_IO_RGB_DATA9,
            EXAMPLE_LCD_IO_RGB_DATA10,
            EXAMPLE_LCD_IO_RGB_DATA11,
            EXAMPLE_LCD_IO_RGB_DATA12,
            EXAMPLE_LCD_IO_RGB_DATA13,
            EXAMPLE_LCD_IO_RGB_DATA14,
            EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .flags = {
            .fb_in_psram = 1, // Use PSRAM for framebuffer
        },
    };

    // Create a new RGB panel with the specified configuration
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Initialize RGB LCD panel"); // Log the initialization of the RGB LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle)); // Initialize the LCD panel

    esp_lcd_touch_handle_t tp_handle = NULL; // Declare a handle for the touch panel
#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911
    ESP_LOGI(TAG, "Initialize I2C bus"); // Log the initialization of the I2C bus
    i2c_master_init(); // Initialize the I2C master
    ESP_LOGI(TAG, "Initialize GPIO"); // Log GPIO initialization
    gpio_init(); // Initialize GPIO pins
    ESP_LOGI(TAG, "Initialize Touch LCD"); // Log touch LCD initialization
    waveshare_esp32_s3_touch_reset(); // Reset the touch panel

    esp_lcd_panel_io_handle_t tp_io_handle = NULL; // Declare a handle for touch panel I/O
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG(); // Configure I2C for GT911 touch controller

    ESP_LOGI(TAG, "Initialize I2C panel IO"); // Log I2C panel I/O initialization
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle)); // Create new I2C panel I/O

    ESP_LOGI(TAG, "Initialize touch controller GT911"); // Log touch controller initialization
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES, // Set maximum X coordinate
        .y_max = EXAMPLE_LCD_V_RES, // Set maximum Y coordinate
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST, // GPIO number for reset
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT, // GPIO number for interrupt
        .levels = {
            .reset = 0, // Reset level
            .interrupt = 0, // Interrupt level
        },
        .flags = {
            .swap_xy = 0, // No swap of X and Y
            .mirror_x = 0, // No mirroring of X
            .mirror_y = 0, // No mirroring of Y
        },
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp_handle)); // Create new I2C GT911 touch controller
#endif // CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911

    ESP_ERROR_CHECK(lvgl_port_init(panel_handle, tp_handle)); // Initialize LVGL with the panel and touch handles

    // Register callbacks for RGB panel events
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
#if EXAMPLE_RGB_BOUNCE_BUFFER_SIZE > 0
        .on_bounce_frame_finish = rgb_lcd_on_vsync_event, // Callback for bounce frame finish
#else
        .on_vsync = rgb_lcd_on_vsync_event, // Callback for vertical sync
#endif
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL)); // Register event callbacks

    return ESP_OK; // Return success 
}

/******************************* Turn on the screen backlight **************************************/
esp_err_t wavesahre_rgb_lcd_bl_on()
{
    //Configure CH422G to output mode 
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    //Pull the backlight pin high to light the screen backlight 
    write_buf = 0x1E;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ESP_OK;
}

/******************************* Turn off the screen backlight **************************************/
esp_err_t wavesahre_rgb_lcd_bl_off()
{
    //Configure CH422G to output mode 
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    //Turn off the screen backlight by pulling the backlight pin low 
    write_buf = 0x1A;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ESP_OK;
}

/******************************* RAW data streaming demo **************************************/
#define RAW_DATA_TIMER_PERIOD_SENSOR_MS 10
#define RAW_DATA_TIMER_PERIOD_CHART_MS 100
#define RAW_DATA_MAX_ABS_VALUE 2.0f
#define RAW_DATA_CHART_POINT_COUNT 50
#define RAW_DATA_CHART_SCALE 1000

typedef struct
{
    float accel[3];
    float gyro[3];
} sensor_data_t;

typedef struct
{
    lv_obj_t *chart;
    lv_chart_series_t *accel_series[3];
    lv_chart_series_t *gyro_series[3];
    lv_obj_t *accel_labels[3];
    lv_obj_t *gyro_labels[3];
} raw_data_ui_ctx_t;

static sensor_data_t s_sensor_data;
static raw_data_ui_ctx_t s_raw_ui_ctx;

static inline float clampf_range(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static inline lv_coord_t raw_to_chart_coord(float value)
{
    return (lv_coord_t)(value * RAW_DATA_CHART_SCALE);
}

static float noise_component(float magnitude)
{
    return ((float)lv_rand(-100, 100) / 1000.0f) * magnitude;
}

static void emulate_sensor_task(lv_timer_t *timer)
{
{
    lv_obj_t *chart;
    lv_chart_series_t *accel_series[3];
    lv_chart_series_t *gyro_series[3];
    lv_obj_t *accel_labels[3];
    lv_obj_t *gyro_labels[3];
} raw_data_ui_ctx_t;

static sensor_data_t s_sensor_data;
static raw_data_ui_ctx_t s_raw_ui_ctx;

static inline float clampf_range(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static inline lv_coord_t raw_to_chart_coord(float value)
{
    return (lv_coord_t)(value * RAW_DATA_CHART_SCALE);
}

static float noise_component(float magnitude)
{
    return ((float)lv_rand(-100, 100) / 1000.0f) * magnitude;
}

static void emulate_sensor_task(lv_timer_t *timer)
{
    (void)timer;
    static float time_s = 0.0f;
    time_s += RAW_DATA_TIMER_PERIOD_SENSOR_MS / 1000.0f;

    const float two_pi = 6.28318530718f;

    s_sensor_data.accel[0] = clampf_range(1.2f * sinf(two_pi * 0.35f * time_s) + noise_component(0.05f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.accel[1] = clampf_range(1.0f * sinf(two_pi * 0.22f * time_s + 0.8f) + noise_component(0.05f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.accel[2] = clampf_range(0.9f * sinf(two_pi * 0.28f * time_s + 1.6f) + noise_component(0.05f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);

    s_sensor_data.gyro[0] = clampf_range(1.4f * sinf(two_pi * 0.18f * time_s + 0.3f) + noise_component(0.06f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.gyro[1] = clampf_range(1.1f * sinf(two_pi * 0.30f * time_s + 1.1f) + noise_component(0.06f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.gyro[2] = clampf_range(1.0f * sinf(two_pi * 0.24f * time_s + 2.0f) + noise_component(0.06f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.accel[0] = clampf_range(1.0f * sinf(two_pi * 0.35f * time_s) + 0.3f * cosf(two_pi * 0.15f * time_s) + noise_component(0.08f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.accel[1] = clampf_range(0.8f * sinf(two_pi * 0.25f * time_s + 0.5f) + noise_component(0.1f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.accel[2] = clampf_range(0.6f * cosf(two_pi * 0.18f * time_s - 0.2f) + 0.4f * sinf(two_pi * 0.05f * time_s) + noise_component(0.05f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);

    s_sensor_data.gyro[0] = clampf_range(1.3f * sinf(two_pi * 0.12f * time_s) + 0.4f * cosf(two_pi * 0.32f * time_s) + noise_component(0.12f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.gyro[1] = clampf_range(1.0f * cosf(two_pi * 0.22f * time_s + 0.7f) + noise_component(0.1f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
    s_sensor_data.gyro[2] = clampf_range(0.9f * sinf(two_pi * 0.28f * time_s - 0.4f) + 0.5f * cosf(two_pi * 0.08f * time_s) + noise_component(0.09f), -RAW_DATA_MAX_ABS_VALUE, RAW_DATA_MAX_ABS_VALUE);
}

static void update_chart_and_labels(lv_timer_t *timer)
{
    raw_data_ui_ctx_t *ctx = timer->user_data;
    if (ctx == NULL || ctx->chart == NULL)
    {
        return;
    }

    for (int i = 0; i < 3; i++)
    {
        lv_chart_set_next_value(ctx->chart, ctx->accel_series[i], raw_to_chart_coord(s_sensor_data.accel[i]));
        lv_chart_set_next_value(ctx->chart, ctx->gyro_series[i], raw_to_chart_coord(s_sensor_data.gyro[i]));
    }

    lv_label_set_text_fmt(ctx->accel_labels[0], "Acc x: %.2f", s_sensor_data.accel[0]);
    lv_label_set_text_fmt(ctx->accel_labels[1], "Acc y: %.2f", s_sensor_data.accel[1]);
    lv_label_set_text_fmt(ctx->accel_labels[2], "Acc z: %.2f", s_sensor_data.accel[2]);

    lv_label_set_text_fmt(ctx->gyro_labels[0], "Gyro roll: %.2f", s_sensor_data.gyro[0]);
    lv_label_set_text_fmt(ctx->gyro_labels[1], "Gyro Pitch: %.2f", s_sensor_data.gyro[1]);
    lv_label_set_text_fmt(ctx->gyro_labels[2], "Gyro yaw: %.2f", s_sensor_data.gyro[2]);
    lv_label_set_text_fmt(ctx->accel_labels[0], "Gyorsulásmérő X: %.2f", s_sensor_data.accel[0]);
    lv_label_set_text_fmt(ctx->accel_labels[1], "Gyorsulásmérő Y: %.2f", s_sensor_data.accel[1]);
    lv_label_set_text_fmt(ctx->accel_labels[2], "Gyorsulásmérő Z: %.2f", s_sensor_data.accel[2]);

    lv_label_set_text_fmt(ctx->gyro_labels[0], "Giroszkóp Roll: %.2f", s_sensor_data.gyro[0]);
    lv_label_set_text_fmt(ctx->gyro_labels[1], "Giroszkóp Pitch: %.2f", s_sensor_data.gyro[1]);
    lv_label_set_text_fmt(ctx->gyro_labels[2], "Giroszkóp Yaw: %.2f", s_sensor_data.gyro[2]);
}

void raw_data_demo_ui(void)
{
    lv_memset_00(&s_sensor_data, sizeof(s_sensor_data));
    lv_memset_00(&s_raw_ui_ctx, sizeof(s_raw_ui_ctx));

    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *status_panel = lv_obj_create(scr);
    lv_obj_set_size(status_panel, 300, 300);
    lv_obj_align(status_panel, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_set_style_bg_color(status_panel, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_bg_opa(status_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(status_panel, 20, 0);
    lv_obj_set_style_border_width(status_panel, 0, 0);
    lv_obj_set_style_pad_all(status_panel, 0, 0);

    lv_obj_t *warning_label = lv_label_create(scr);
    lv_label_set_text(warning_label, "Wrong move");
    lv_label_set_text(warning_label, "Rossz végrehajtás");
    lv_obj_align_to(warning_label, status_panel, LV_ALIGN_OUT_TOP_MID, 0, -10);

    lv_obj_t *raw_values_container = lv_obj_create(scr);
    lv_obj_remove_style_all(raw_values_container);
    lv_obj_set_size(raw_values_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(raw_values_container, LV_ALIGN_TOP_RIGHT, -20, 20);
    lv_obj_set_style_pad_all(raw_values_container, 0, 0);
    lv_obj_set_style_pad_row(raw_values_container, 0, 0);
    lv_obj_set_style_pad_column(raw_values_container, 24, 0);
    lv_obj_set_layout(raw_values_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(raw_values_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_size(raw_values_container, 400, LV_SIZE_CONTENT);
    lv_obj_align(raw_values_container, LV_ALIGN_TOP_RIGHT, -20, 20);
    lv_obj_set_style_pad_all(raw_values_container, 0, 0);
    lv_obj_set_style_pad_row(raw_values_container, 10, 0);
    lv_obj_set_layout(raw_values_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(raw_values_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(raw_values_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *accel_container = lv_obj_create(raw_values_container);
    lv_obj_remove_style_all(accel_container);
    lv_obj_set_size(accel_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_layout(accel_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(accel_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(accel_container, 6, 0);

    const lv_color_t accel_colors[3] = {
        lv_palette_main(LV_PALETTE_RED),
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_GREEN),
    };
    const char *accel_axis_labels[3] = {"Acc x", "Acc y", "Acc z"};
    for (int i = 0; i < 3; i++)
    {
        s_raw_ui_ctx.accel_labels[i] = lv_label_create(accel_container);
        lv_label_set_text_fmt(s_raw_ui_ctx.accel_labels[i], "%s: %.2f", accel_axis_labels[i], 0.0f);
        lv_obj_set_style_text_color(s_raw_ui_ctx.accel_labels[i], accel_colors[i], 0);
    lv_obj_set_style_pad_row(accel_container, 4, 0);

    lv_obj_t *accel_title = lv_label_create(accel_container);
    lv_label_set_text(accel_title, "Gyorsulásmérő");
    const char *accel_axis_labels[3] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; i++)
    {
        s_raw_ui_ctx.accel_labels[i] = lv_label_create(accel_container);
        lv_label_set_text_fmt(s_raw_ui_ctx.accel_labels[i], "Gyorsulásmérő %s: %.2f", accel_axis_labels[i], 0.0f);
    }

    lv_obj_t *gyro_container = lv_obj_create(raw_values_container);
    lv_obj_remove_style_all(gyro_container);
    lv_obj_set_size(gyro_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_layout(gyro_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(gyro_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(gyro_container, 6, 0);

    const lv_color_t gyro_colors[3] = {
        lv_palette_main(LV_PALETTE_ORANGE),
        lv_palette_main(LV_PALETTE_PURPLE),
        lv_palette_main(LV_PALETTE_TEAL),
    };
    const char *gyro_axis_labels[3] = {"Gyro roll", "Gyro Pitch", "Gyro yaw"};
    for (int i = 0; i < 3; i++)
    {
        s_raw_ui_ctx.gyro_labels[i] = lv_label_create(gyro_container);
        lv_label_set_text_fmt(s_raw_ui_ctx.gyro_labels[i], "%s: %.2f", gyro_axis_labels[i], 0.0f);
        lv_obj_set_style_text_color(s_raw_ui_ctx.gyro_labels[i], gyro_colors[i], 0);
    lv_obj_set_style_pad_row(gyro_container, 4, 0);

    lv_obj_t *gyro_title = lv_label_create(gyro_container);
    lv_label_set_text(gyro_title, "Giroszkóp");
    const char *gyro_axis_labels[3] = {"Roll", "Pitch", "Yaw"};
    for (int i = 0; i < 3; i++)
    {
        s_raw_ui_ctx.gyro_labels[i] = lv_label_create(gyro_container);
        lv_label_set_text_fmt(s_raw_ui_ctx.gyro_labels[i], "Giroszkóp %s: %.2f", gyro_axis_labels[i], 0.0f);
    }

    s_raw_ui_ctx.chart = lv_chart_create(scr);
    lv_obj_set_size(s_raw_ui_ctx.chart, 400, 300);
    lv_obj_align(s_raw_ui_ctx.chart, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_chart_set_type(s_raw_ui_ctx.chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_raw_ui_ctx.chart, RAW_DATA_CHART_POINT_COUNT);
    lv_chart_set_update_mode(s_raw_ui_ctx.chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_range(s_raw_ui_ctx.chart, LV_CHART_AXIS_PRIMARY_X, 0, 5000);
    lv_chart_set_range(s_raw_ui_ctx.chart, LV_CHART_AXIS_PRIMARY_Y, (lv_coord_t)(-RAW_DATA_CHART_SCALE * RAW_DATA_MAX_ABS_VALUE), (lv_coord_t)(RAW_DATA_CHART_SCALE * RAW_DATA_MAX_ABS_VALUE));
    lv_chart_set_axis_tick(s_raw_ui_ctx.chart, LV_CHART_AXIS_PRIMARY_X, 10, 5, 6, 5, true, 40);
    lv_chart_set_axis_tick(s_raw_ui_ctx.chart, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 5, 4, true, 50);
    lv_chart_set_div_line_count(s_raw_ui_ctx.chart, 5, 6);

    lv_obj_set_style_bg_opa(s_raw_ui_ctx.chart, LV_OPA_30, 0);
    lv_obj_set_style_pad_all(s_raw_ui_ctx.chart, 8, 0);
    lv_obj_set_style_line_width(s_raw_ui_ctx.chart, 2, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_size(s_raw_ui_ctx.chart, 4, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_t *chart_title = lv_label_create(scr);
    lv_label_set_text(chart_title, "RAW DATA");
    lv_obj_align_to(chart_title, s_raw_ui_ctx.chart, LV_ALIGN_OUT_TOP_MID, 0, -10);

    s_raw_ui_ctx.accel_series[0] = lv_chart_add_series(s_raw_ui_ctx.chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    s_raw_ui_ctx.accel_series[1] = lv_chart_add_series(s_raw_ui_ctx.chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    s_raw_ui_ctx.accel_series[2] = lv_chart_add_series(s_raw_ui_ctx.chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);

    s_raw_ui_ctx.gyro_series[0] = lv_chart_add_series(s_raw_ui_ctx.chart, lv_palette_main(LV_PALETTE_ORANGE), LV_CHART_AXIS_PRIMARY_Y);
    s_raw_ui_ctx.gyro_series[1] = lv_chart_add_series(s_raw_ui_ctx.chart, lv_palette_main(LV_PALETTE_PURPLE), LV_CHART_AXIS_PRIMARY_Y);
    s_raw_ui_ctx.gyro_series[2] = lv_chart_add_series(s_raw_ui_ctx.chart, lv_palette_main(LV_PALETTE_TEAL), LV_CHART_AXIS_PRIMARY_Y);

    for (int i = 0; i < 3; i++)
    {
        lv_chart_set_all_value(s_raw_ui_ctx.chart, s_raw_ui_ctx.accel_series[i], 0);
        lv_chart_set_all_value(s_raw_ui_ctx.chart, s_raw_ui_ctx.gyro_series[i], 0);
    }

    lv_timer_create(emulate_sensor_task, RAW_DATA_TIMER_PERIOD_SENSOR_MS, NULL);
    lv_timer_create(update_chart_and_labels, RAW_DATA_TIMER_PERIOD_CHART_MS, &s_raw_ui_ctx);
}
