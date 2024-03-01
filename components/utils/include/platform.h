#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#define NODE_PLATFORM_ESP32		1001
#define NODE_PLATFORM_ESP8266	1002
#define NODE_PLATFORM_STM32		1003
#define NODE_PLATFORM_W800		1004


#define CURRENT_PLATFORM		NODE_PLATFORM_ESP32
#define BUILD_SPIFFS


#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
#define OUTPUT 				GPIO_MODE_OUTPUT
#define INPUT 				GPIO_MODE_INPUT
#define PULLUP 				GPIO_PULLUP_ONLY
#define FLOAT 				GPIO_FLOATING
#define INOUT 				GPIO_MODE_INPUT_OUTPUT
#define PLATFORM_INTERRUPT 	GPIO_INTR_POSEDGE
#define HIGH 				1
#define LOW 				0
#define GPIO_PIN_NUM 		GPIO_NUM_MAX
#else
#define PULLUP PLATFORM_GPIO_PULLUP
#define FLOAT PLATFORM_GPIO_FLOAT
#define OUTPUT PLATFORM_GPIO_OUTPUT
#define INPUT PLATFORM_GPIO_INPUT
#define INOUT PLATFORM_GPIO_INOUT
#define PLATFORM_INTERRUPT PLATFORM_GPIO_INT
#define HIGH PLATFORM_GPIO_HIGH
#define LOW PLATFORM_GPIO_LOW
#endif

void lua_node_restart(void);
void lua_node_sleep(uint64_t us);
uint32_t lua_node_get_heap_size(void);
void lua_node_system_restore(void);

int platform_gpio_mode(int pin, int mode, int type);
void platform_gpio_isr_uninstall(void);
uint8_t platform_gpio_read(int pin); 
void platform_gpio_write(int pin, int level);



#endif
