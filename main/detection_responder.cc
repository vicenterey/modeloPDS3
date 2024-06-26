#include <stdio.h>
#include "detection_responder.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "driver/gpio.h"  // Incluir esta línea para controlar el GPIO
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_main.h"

#if DISPLAY_SUPPORT
#include "image_provider.h"
#include "bsp/esp-bsp.h"

// Camera definition is always initialized to match the trained detection model: 96x96 pix
// That is too small for LCD displays, so we extrapolate the image to 192x192 pix
#define IMG_WD (96 * 2)
#define IMG_HT (96 * 2)

static lv_obj_t *camera_canvas = NULL;
static lv_obj_t *animal_indicator = NULL;
static lv_obj_t *label = NULL;

static void create_gui(void)
{
  bsp_display_start();
  bsp_display_backlight_on(); // Set display brightness to 100%
  bsp_display_lock(0);
  camera_canvas = lv_canvas_create(lv_scr_act());
  assert(camera_canvas);
  lv_obj_align(camera_canvas, LV_ALIGN_TOP_MID, 0, 0);

  animal_indicator = lv_led_create(lv_scr_act());
  assert(animal_indicator);
  lv_obj_align(animal_indicator, LV_ALIGN_BOTTOM_MID, -70, 0);
  lv_led_set_color(animal_indicator, lv_palette_main(LV_PALETTE_GREEN));

  label = lv_label_create(lv_scr_act());
  assert(label);
  lv_label_set_text_static(label, "Animal detected");
  lv_obj_align_to(label, animal_indicator, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
  bsp_display_unlock();
}
#endif // DISPLAY_SUPPORT
#define LED_GPIO_PIN GPIO_NUM_4  // Ajusta esto según tu configuración de hardware

#define SERVO_MIN_PULSEWIDTH 1000 // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 // Maximum angle in degree upto which servo can rotate

static uint32_t targetAngle = 0;

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 13);    // Set GPIO 13 as PWM0A, to which servo is connected
}
void init_gpio() {
    gpio_reset_pin(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void mcpwm_example_servo_control(void *arg)
{
    uint32_t angle;
    // 1. MCPWM GPIO initialization
    mcpwm_example_gpio_initialize();

    // 2. Initial MCPWM configuration
    printf("Configuring Initial Parameters of MCPWM......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    // Configure PWM0A & PWM0B with above settings

    uint32_t currentAngle = 0;
    while (1) {
        if (targetAngle != currentAngle) {
            angle = servo_per_degree_init(targetAngle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            currentAngle = targetAngle;
        }
        vTaskDelay(10);     // Add delay, since it takes time for servo to rotate, generally 100ms/60 degree rotation at 5V
    }
}
void move_servo_to_90_degrees()
{
    targetAngle = 0;
}
void move_servo_to_0_degrees()
{
    targetAngle = 180;
}

void RespondToDetection(float dog_score) {
    xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
    // Mueve el servo a uno de sus extremos (por ejemplo, 0 grados)
    // Wait for 2 seconds

    // Move servo back to -90 degrees // Wait for 2 seconds

    // Asegúrate de que dog_score está entre 0 y 1
    if (dog_score > 0.85 ) {
      
        MicroPrintf("The animal is a Dog");
        gpio_set_level(LED_GPIO_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO_PIN, 0);
        move_servo_to_90_degrees();
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Enciende el LED si es un perro
    } else if (dog_score < 0.4) {
        MicroPrintf("The animal is a Cat");
        gpio_set_level(LED_GPIO_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO_PIN, 0);
        move_servo_to_0_degrees();
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Apaga el LED si es un gato
    } else {
        MicroPrintf("The animal is neither a Dog nor a Cat");
         // Apaga el LED si no es un perro ni un gato
    }

    MicroPrintf("%f", dog_score);

#if DISPLAY_SUPPORT
    if (!camera_canvas) {
        create_gui();
    }

    uint16_t *buf = (uint16_t *) image_provider_get_display_buf();

    bsp_display_lock(0);
    if (dog_score < 0.9) {
        lv_led_off(animal_indicator);
    } else {
        lv_led_on(animal_indicator);
    }
    lv_canvas_set_buffer(camera_canvas, buf, IMG_WD, IMG_HT, LV_IMG_CF_TRUE_COLOR);
    bsp_display_unlock();
#endif // DISPLAY_SUPPORT
}
