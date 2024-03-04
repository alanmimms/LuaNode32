#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "uart_handler.h"
#include "lua.h"
#include "utils.h"

#define DO_UNITTEST	0

#define OUR_UART UART_NUM_0
#define TXD_PIN GPIO_NUM_43
#define RXD_PIN GPIO_NUM_44

#define UARTTASK_STACK_SIZE	4096
#define RX_BUF_SIZE 		1024 /* MUST be power of two */
#define RX_MOD_MASK		(RX_BUF_SIZE - 1)
#if (RX_BUF_SIZE & (RX_BUF_SIZE - 1)) != 0
#error "RX_BUF_SIZE must be a power of two!"
#endif

static const char *TAG = "uart";

static QueueHandle_t uartQ;


// UART receive ring buffer, initialized statically to be empty.  When
// rxHead == rxTail the buffer is empty. The rxHead and rxTail values
// are indices into this ring, modulo RX_BUF_SIZE. New characters
// received are inserted at rxTail, which is post-incremented so it
// always points to the next unused byte in the ring. Characters are
// extracted and sent to Lua from rxHead, which is post-incremented so
// it always points to the oldest used byte in the ring.
//
// 0
// 1
// 2=A	H
// 3=B
// 4=C
// 5	T
// 6
static uint8_t rxRingBuf[RX_BUF_SIZE];
static unsigned rxHead = RX_BUF_SIZE / 2;
static unsigned rxTail = RX_BUF_SIZE / 2;


static inline int min(int a, int b) {
    return a < b ? a : b;
}


static inline unsigned rxRingUsedSpace(void) {
  return ((rxTail - rxHead) & RX_MOD_MASK);
}


static inline unsigned rxRingUnusedSpace(void) {
  return RX_BUF_SIZE - 1 - rxRingUsedSpace();
}


static uint8_t rxRingGet(void) {
  // SHOULD NEVER BE CALLED WHEN EMPTY
  if (rxRingUsedSpace() == 0) {
    ESP_LOGE(TAG, "Attempt to rxRingGet() when ring is empty!");
    return 0;
  }

  uint8_t ch = rxRingBuf[rxHead++];
  if (rxHead >= RX_BUF_SIZE) rxHead = 0;
  return ch;
}


static void rxRingPut(uint8_t ch) {
  // SHOULD NEVER BE CALLED WHEN FULL
  if (rxRingUnusedSpace() == 0) {
    ESP_LOGE(TAG, "Attempt to rxRingPut(0x%02X) when ring is full!", ch);
    return;
  }

  rxRingBuf[rxTail++] = ch;
  if (rxTail >= RX_BUF_SIZE) rxTail = 0;
}


// This depends on being called only syncronously from the
// `lua_handle_input()` function, which is called synchronously inside
// the UART task and therefore eliminates the need to synchronize with
// the UART task.
bool uart_getc(char *c) {
  if (rxRingUsedSpace() == 0) return false;
  *c = rxRingGet();
  return true;
}


static void uartTask(void *arg)
{
  ESP_LOGI(TAG, "uartTask");
  uart_flush(OUR_UART);

  while(1) {
    uart_event_t event;
    uint8_t tempBuf[16];

    if (!xQueueReceive(uartQ, &event, portMAX_DELAY)) continue;

    switch (event.type) {
    case UART_DATA:

      do {
	int readLen = min(sizeof(tempBuf), min(event.size, rxRingUnusedSpace()));

	if (readLen > rxRingUnusedSpace()) {
	  ESP_LOGW(TAG, "UART dropping %d bytes", readLen - rxRingUnusedSpace());
	}

	int len = uart_read_bytes(OUR_UART, tempBuf, readLen, 20 / portTICK_PERIOD_MS);

	if (len > 0) {

	  // Echo the bytes we read
	  //	  uart_write_bytes(OUR_UART, tempBuf, len);

	  // Copy the bytes into our ring
	  for (int k = 0; k < len; ++k) rxRingPut(tempBuf[k]);

	  lua_handle_input(false);
	  event.size -= len;
	} else {
	  ESP_LOGI(TAG, "UART_DATA uart_read_bytes weird len=%d", len);
	}
      } while (event.size > 0);

      break;

    default:
      ESP_LOGI(TAG, "%d event", (int) event.type);
      break;
    }
  }
}


#if DO_UNITTEST
static void unitTest(void) {
  ESP_LOGI(TAG, "unit test");

  assert(rxRingUsedSpace() == 0);
  assert(rxRingUnusedSpace() == RX_BUF_SIZE - 1);

  rxRingPut(0x01);
  assert(rxRingUsedSpace() == 1);
  assert(rxRingUnusedSpace() == RX_BUF_SIZE - 2);

  rxRingPut(0x02);
  assert(rxRingUsedSpace() == 2);
  assert(rxRingUnusedSpace() == RX_BUF_SIZE - 3);

  rxRingPut(0x03);
  assert(rxRingUsedSpace() == 3);
  assert(rxRingUnusedSpace() == RX_BUF_SIZE - 4);

  assert(rxRingGet() == 0x01);
  assert(rxRingUsedSpace() == 2);
  assert(rxRingUnusedSpace() == RX_BUF_SIZE - 3);

  assert(rxRingGet() == 0x02);
  assert(rxRingUsedSpace() == 1);
  assert(rxRingUnusedSpace() == RX_BUF_SIZE - 2);

  assert(rxRingGet() == 0x03);
  assert(rxRingUsedSpace() == 0);
  assert(rxRingUnusedSpace() == RX_BUF_SIZE - 1);

  // Fill the empty ring
  for (int k=0; rxRingUsedSpace() < RX_BUF_SIZE - 1; ++k) rxRingPut((uint8_t) k);
  assert(rxRingUsedSpace() == RX_BUF_SIZE - 1);
  assert(rxRingUnusedSpace() == 0);

  // Over-fill the ring
  ESP_LOGI(TAG, "Note next five 'ring is full' errors are expected");
  rxRingPut((uint8_t) 0xFF);
  rxRingPut((uint8_t) 0xFF);
  rxRingPut((uint8_t) 0xFF);
  rxRingPut((uint8_t) 0xFF);
  rxRingPut((uint8_t) 0xFF);

  // Empty the ring and check bytes as we go
  for (int k=0; rxRingUsedSpace() != 0; ++k) assert(rxRingGet() == (uint8_t) k);
}
#endif


void my_uart_init(void) {
  ESP_LOGI(TAG, "My uart init");
  uart_flush(OUR_UART);

  const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK(uart_driver_install(OUR_UART, RX_BUF_SIZE * 2, 0, 20, &uartQ, 0));
  ESP_ERROR_CHECK(uart_param_config(OUR_UART, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(OUR_UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_rts(OUR_UART, 1));
  vTaskDelay(300 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "");

#if DO_UNITTEST
  unitTest();
#endif

  xTaskCreate(uartTask, "uartTask", UARTTASK_STACK_SIZE, NULL, 10, NULL);
}
