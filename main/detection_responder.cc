/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

/*
 * SPDX-FileCopyrightText: 2019-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "detection_responder.h"
#include "tensorflow/lite/micro/micro_log.h"

#include "esp_main.h"

extern "C" {
  #include <stdio.h>
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "driver/gpio.h"
  #include "driver/ledc.h"
}

#define SERVO_PINX GPIO_NUM_13     // Pin de señal del servo
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_FREQ 50              // Frecuencia PWM en Hz (50Hz para servos)

#define LED_PIN GPIO_NUM_2
#define BUTTON_PIN GPIO_NUM_15

#define SERVO_MIN_PULSEWIDTH 1600
#define SERVO_MAX_PULSEWIDTH 8000
#define SERVO_MAX_DEGREE     180

void setup_pwm(uint8_t SERVO_PIN) {
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_16_BIT,
    .timer_num        = LEDC_TIMER,
    .freq_hz          = LEDC_FREQ,
    .clk_cfg          = LEDC_AUTO_CLK
};
  ledc_timer_config(&ledc_timer);
  
  ledc_channel_config_t ledc_channel = {
    .gpio_num       = SERVO_PIN,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER,
    .duty           = 0,
    .hpoint         = 0,
    .flags          = 0
  };
  ledc_channel_config(&ledc_channel);
}

void set_servo_angle(int angle) {
  int duty = SERVO_MIN_PULSEWIDTH + ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / SERVO_MAX_DEGREE;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
}


#if DISPLAY_SUPPORT
#include "image_provider.h"
#include "bsp/esp-bsp.h"


// Camera definition is always initialized to match the trained detection model: 96x96 pix
// That is too small for LCD displays, so we extrapolate the image to 192x192 pix
#define IMG_WD (96 * 2)
#define IMG_HT (96 * 2)

static lv_obj_t *camera_canvas = NULL;
static lv_obj_t *person_indicator = NULL;
static lv_obj_t *label = NULL;

static void create_gui(void)
{
  bsp_display_start();
  bsp_display_backlight_on(); // Set display brightness to 100%
  bsp_display_lock(0);
  camera_canvas = lv_canvas_create(lv_scr_act());
  assert(camera_canvas);
  lv_obj_align(camera_canvas, LV_ALIGN_TOP_MID, 0, 0);

  person_indicator = lv_led_create(lv_scr_act());
  assert(person_indicator);
  lv_obj_align(person_indicator, LV_ALIGN_BOTTOM_MID, -70, 0);
  lv_led_set_color(person_indicator, lv_palette_main(LV_PALETTE_GREEN));

  label = lv_label_create(lv_scr_act());
  assert(label);
  lv_label_set_text_static(label, "Person detected");
  lv_obj_align_to(label, person_indicator, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
  bsp_display_unlock();
}
#endif // DISPLAY_SUPPORT

void RespondToDetection(float* sign_score, const char* kCategoryLabels[]) {

  gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_4, 0);

  float max_score = 0;
  int max_score_index = 0;
  for (int i = 0; i < 6; ++i) {
    if (sign_score[i] > max_score) {
      max_score = sign_score[i];
      max_score_index = i;
    }
  }

  // Log the detected sign.
  if (max_score > 0.5) {
    gpio_set_level(GPIO_NUM_4, 1);

    setup_pwm(SERVO_PINX);
    set_servo_angle(0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    set_servo_angle(180);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    MicroPrintf("Detected sign: %s", kCategoryLabels[max_score_index]);
  } else {
    gpio_set_level(GPIO_NUM_4, 0);

    setup_pwm(SERVO_PINX);
    set_servo_angle(0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    set_servo_angle(180);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    MicroPrintf("No sign detected");
  }
  MicroPrintf("C: %f, L: %f, Puno: %f, Cruzados: %f, Rock: %f, Palma: %f", sign_score[0], sign_score[1], sign_score[2], sign_score[3], sign_score[4], sign_score[5]);
}