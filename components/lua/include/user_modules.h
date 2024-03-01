#ifndef __USER_MODULES_H__
#define __USER_MODULES_H__

#define LUA_USE_BUILTIN_STRING		// for string.xxx()
#define LUA_USE_BUILTIN_TABLE		// for table.xxx()
#define LUA_USE_BUILTIN_COROUTINE	// for coroutine.xxx()
#define LUA_USE_BUILTIN_MATH		// for math.xxx(), partially work
// #define LUA_USE_BUILTIN_IO 			// for io.xxx(), partially work

// #define LUA_USE_BUILTIN_OS			// for os.xxx(), not work
// #define LUA_USE_BUILTIN_DEBUG
#define LUA_USE_BUILTIN_DEBUG_MINIMAL // for debug.getregistry() and debug.traceback()


#define USE_NODE_MODULE
#define USE_FILE_MODULE
#define USE_GPIO_MODULE
#define USE_UTILS_MODULE
#define USE_TMR_MODULE
/*#define USE_LPEG_MODULE
#define USE_UART_MODULE
#define USE_MQTT_MODULE
#define USE_PWM_MODULE
#define USE_I2C_MODULE
#define USE_WIFI_MODULE
#define USE_NET_MODULE
//#define USE_THREAD_MODULE
#define USE_NVS_MODULE 
*/



#endif	/* __USER_MODULES_H__ */
