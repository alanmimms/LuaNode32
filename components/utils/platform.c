#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "platform.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"

#define TAG		"platform"

#if SOC_RTC_FAST_MEM_SUPPORTED
static RTC_DATA_ATTR struct timeval sleep_enter_time;
#else
static struct timeval sleep_enter_time;
#endif

void lua_node_restart()
{
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
	esp_restart();
#elif (CURRENT_PLATFORM == NODE_PLATFORM_ESP8266)
	system_restart();
#else
	ESP_LOGE(TAG, "Not define CURRENT_PLATFORM!");
#endif
}

static void deep_sleep_task(void *args)
{

    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        } 
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

#if CONFIG_IDF_TARGET_ESP32
    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
#endif
}

static void deep_sleep_register_rtc_timer_wakeup(uint64_t us)
{
    //printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(us));
	xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);
	// enter deep sleep
    esp_deep_sleep_start();
}

void lua_node_sleep(uint64_t us)
{
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
	//system_deep_sleep (us);
	deep_sleep_register_rtc_timer_wakeup(us);
#else
	ESP_LOGE(TAG, "Not define CURRENT_PLATFORM!");
#endif
}

uint32_t lua_node_get_heap_size(void)
{
	uint32_t heap_size = 0;
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
	heap_size = esp_get_minimum_free_heap_size();
#else
	ESP_LOGE(TAG, "Not define CURRENT_PLATFORM!");
#endif
	return heap_size;
}

void lua_node_system_restore(void)
{
	
}

//////////////////////////////////////////////////
#define GPIO_OUTPUT_PIN_SEL(x)  ((1ULL<<x))

int platform_gpio_mode(int pin, int mode, int type)
{
	gpio_config_t io_conf = {};
    //disable interrupt 
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //default set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL(pin); 
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
	
	switch(mode) {
		case OUTPUT: {
			io_conf.mode = GPIO_MODE_OUTPUT;
		}
		break;
		case INPUT: {
			io_conf.mode = GPIO_MODE_INPUT;
		}
		break;
		default:
		ESP_LOGE(TAG, "Unknown gpio mode");
		break;
	}
	gpio_config(&io_conf);
	
	return ESP_OK;
}

void platform_gpio_isr_uninstall(void)
{
	
}

uint8_t platform_gpio_read(int pin)
{
	if (pin >= GPIO_NUM_MAX) {
		return -1;
	}
	uint8_t level = gpio_get_level(pin);
	return level;
}

void platform_gpio_write(int pin, int level)
{
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
	gpio_set_level(pin, level);
#else
	ESP_LOGE(TAG, "Not support platform");
#endif
}