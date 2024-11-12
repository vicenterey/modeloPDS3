#include "detection_responder.h"
#include "tensorflow/lite/micro/micro_log.h"
#include <cstdio>
#include <cstring>  // Incluir cstring para strlen
#include "esp_main.h"
#include "image_provider.h"
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_system.h"
  #include "esp_log.h"
  #include "driver/uart.h"
  #include "string.h"
  #include "driver/gpio.h"
}

#define UART_NUM UART_NUM_1
#define TXD_PIN (GPIO_NUM_14)
#define RXD_PIN (GPIO_NUM_15)
#define FLASH_PIN (GPIO_NUM_4)
#define UART_BAUD_RATE 115200
static const int RX_BUF_SIZE = 1024;

void uart_init(void)
{
  const uart_config_t uart_config = {
    .baud_rate = UART_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };
  uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM, &uart_config);
  uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Configurar el pin del flash como salida
  esp_rom_gpio_pad_select_gpio(FLASH_PIN);
  gpio_set_direction(FLASH_PIN, GPIO_MODE_OUTPUT);
}

void uart_send_string(const char* str)
{
  const int len = strlen(str);
  const int txBytes = uart_write_bytes(UART_NUM, str, len);
  ESP_LOGI("UART", "Wrote %d bytes", txBytes);
}

void uart_receive_string(char* buffer, int max_len) {
  int rxBytes = uart_read_bytes(UART_NUM, buffer, max_len - 1, portMAX_DELAY);
  if (rxBytes > 0) {
    buffer[rxBytes] = '\0';  // Null-terminate the received string
    ESP_LOGI("UART", "Received %d bytes: '%s'", rxBytes, buffer);
  }
}

int RespondToDetection(float* sign_score, const char* kCategoryLabels[]) {
  // Encender el flash durante un milisegundo
  gpio_set_level(FLASH_PIN, 1);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gpio_set_level(FLASH_PIN, 0);

  float max_score = 0;
  int max_score_index = 0;
  for (int i = 0; i < 6; ++i) {
    if (sign_score[i] > max_score) {
      max_score = sign_score[i];
      max_score_index = i;
    }
  }

  if (max_score > 0.5) {
    MicroPrintf("Detected sign: %s", kCategoryLabels[max_score_index]);
  } else {
    MicroPrintf("No sign detected");
  }
  MicroPrintf("C: %f, L: %f, Puno: %f, Cruzados: %f, Rock: %f, Palma: %f", sign_score[0], sign_score[1], sign_score[2], sign_score[3], sign_score[4], sign_score[5]);

  return max_score_index;
}