/*guys, srsly, turn on warnings in the makefile*/
#if defined(__GNUC__)
#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Wextra"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

/*-------------------------------------
NEW TIMER API
---------------------------------------

tmr.wdclr() -- not changed
tmr.now()   -- not changed
tmr.time()  -- not changed
tmr.delay() -- not changed
tmr.alarm() -- not changed
tmr.stop()  -- changed, see below. use tmr.unregister for old functionality

tmr.register(id, interval, mode, function)
	bind function with timer and set the interval in ms
	the mode can be:
		tmr.ALARM_SINGLE for a single run alarm
		tmr.ALARM_SEMI for a multiple single run alarm
		tmr.ALARM_AUTO for a repating alarm
	tmr.register does NOT start the timer
	tmr.alarm is a tmr.register & tmr.start macro
tmr.unregister(id)
	stop alarm, unbind function and clean up memory
	not needed for ALARM_SINGLE, as it unregisters itself
tmr.start(id)
	ret: bool
	start a alarm, returns true on success
tmr.stop(id)
	ret: bool
	stops a alarm, returns true on success
	this call dose not free any memory, to do so use tmr.unregister
	stopped alarms can be started with start
tmr.interval(id, interval)
	set alarm interval, running alarm will be restarted
tmr.state(id)
	ret: (bool, int) or nil
	returns alarm status (true=started/false=stopped) and mode
	nil if timer is unregistered
tmr.softwd(int)
	set a negative value to stop the timer
	any other value starts the timer, when the
	countdown reaches zero, the device restarts
	the timer units are seconds
*/


#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "lrodefs.h"
#include "c_types.h"
#include "lualib.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define TAG "tmr"
#define NUM_TMR	5

#define TIMER_MODE_OFF 3
#define TIMER_MODE_SINGLE 0
#define TIMER_MODE_PERIODIC 2
#define TIMER_IDLE_FLAG (1<<7) 


//extern uint32_t system_get_time();
//extern uint32_t system_rtc_clock_cali_proc();
//extern uint32_t system_get_rtc_time();
extern void uart_sendStr(const char *str);
extern void ets_delay_us(uint32_t us);

//in fact lua_State is constant, it's pointless to pass it around
//but hey, whatever, I'll just pass it, still we waste 28B here
typedef struct{
	//os_timer_t os;
	lua_State* L;
	sint32_t lua_ref;
	uint32_t interval;
	uint8_t mode;
	esp_timer_handle_t timer_handle;
}timer_struct_t;
typedef timer_struct_t* my_timer_t;


//everybody just love unions! riiiiight?
static union {
	uint64_t block;
	uint32_t part[2];
} rtc_time;
static sint32_t soft_watchdog  = -1;
static timer_struct_t alarm_timers[NUM_TMR];
static SemaphoreHandle_t sem;

static void my_timer_cb(void *p)
{
    //SemaphoreHandle_t sem = (SemaphoreHandle_t)p;
    //xSemaphoreGive(sem);
	my_timer_t tmr = (my_timer_t)p;
	int timer_id = -1;
	for (int i = 0; i < NUM_TMR; i++) {
		if (tmr == &alarm_timers[i]) {
			timer_id = i;
			break;
		}
	}
	//ESP_LOGI(TAG, "timer %d, times up", timer_id);
	if (tmr->lua_ref != LUA_NOREF) {
		lua_rawgeti(tmr->L, LUA_REGISTRYINDEX, tmr->lua_ref);
		lua_pushinteger(tmr->L, timer_id);
		lua_call(tmr->L, 1, 0);
		uart_sendStr("\r\n");
		uart_sendStr("> ");
	}
	if (tmr->mode == TIMER_MODE_SINGLE) {
		esp_timer_delete(tmr->timer_handle);
		tmr->mode = TIMER_MODE_OFF;
		tmr->timer_handle = NULL;
	}
}


void tmr_delay_msec(int ms) {
	vTaskDelay(ms/portTICK_PERIOD_MS); 
}

// Lua: tmr.delay( s )	delay seconds
static int tmr_delay_s( lua_State *L ) {
	uint32_t period = luaL_checkinteger(L, 1);
	vTaskDelay(period*(1000/portTICK_PERIOD_MS));
	return 0;
}

// Lua: tmr.delay_ms( ms )
static int tmr_delay_ms( lua_State *L ) {
	uint32_t period = luaL_checkinteger(L, 1);
	tmr_delay_msec(period);
	return 0;
}

// Lua: tmr.delay_us( us )
static int tmr_delay( lua_State* L ){
	sint32_t us = luaL_checkinteger(L, 1);
	if(us <= 0) {
		return luaL_error(L, "wrong arg range");
	}
	while(us >= 1000000){
		us -= 1000000;
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
	uint32_t remain = (uint32_t)(us/1000);
	if(remain > 0) {
		us = us % 1000;
		vTaskDelay(remain/portTICK_PERIOD_MS);
	}
	if(us > 0) {
		ets_delay_us(us);
	}
	return 0; 
}

// Lua: tmr.now() , return system timer in us
static int tmr_now(lua_State* L){
	uint32_t now = xTaskGetTickCount(); //0x7FFFFFFF & system_get_time();
	lua_pushinteger(L, now);
	return 1; 
}


// Lua: tmr.register( id, interval, mode, function )
// unit for interval is us(microsecond)
static int tmr_register(lua_State* L){
	uint32_t id = luaL_checkinteger(L, 1);
	if (id >= NUM_TMR) {
		ESP_LOGE(TAG, "Error timer id");
		return 0;
	}
	sint32_t interval = luaL_checkinteger(L, 2);
	uint8_t mode = luaL_checkinteger(L, 3);
	//validate arguments
	uint8_t args_valid = interval <= 0
		|| (mode != TIMER_MODE_SINGLE && mode != TIMER_MODE_PERIODIC && mode != TIMER_MODE_OFF)
		|| (lua_type(L, 4) != LUA_TFUNCTION && lua_type(L, 4) != LUA_TLIGHTFUNCTION);
	if(args_valid)
		return luaL_error(L, "wrong arg range");
	//get the lua function reference
	lua_pushvalue(L, 4);
	sint32_t ref = luaL_ref(L, LUA_REGISTRYINDEX);
	my_timer_t tmr = &alarm_timers[id];

	//there was a bug in this part, the second part of the following condition was missing
	if(tmr->lua_ref != LUA_NOREF && tmr->lua_ref != ref) {
		luaL_unref(L, LUA_REGISTRYINDEX, tmr->lua_ref);
	}
	tmr->lua_ref = ref;
	tmr->mode = mode;
	tmr->interval = interval;
	tmr->L = L; 
	
	esp_timer_create_args_t timer_args = {
        .callback = my_timer_cb,
        .arg = tmr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "test_timer",
    };
	esp_err_t err = esp_timer_create(&timer_args, &(tmr->timer_handle));
	if (ESP_OK != err) {
		return luaL_error(L, "create timer failed");
	}
	//esp_timer_start_once(tmr->timer_handle, interval);
	//xSemaphoreTake(sem, portMAX_DELAY);
	
	return ESP_OK;  
}

// Lua: tmr.start( id )
static int tmr_start(lua_State* L){
	uint8_t id = luaL_checkinteger(L, 1);
	if (id >= NUM_TMR) {
		ESP_LOGE(TAG, "Error timer id");
		return 0;
	}
	my_timer_t tmr = &alarm_timers[id];
	//we return false if the timer is not idle
	if(tmr->mode == TIMER_MODE_SINGLE) {
		esp_timer_start_once(tmr->timer_handle, tmr->interval);
	} else if (tmr->mode == TIMER_MODE_PERIODIC) {
		esp_timer_start_periodic(tmr->timer_handle, tmr->interval);
	} else if (tmr->mode == TIMER_MODE_OFF) {
		//esp_timer_stop(tmr->timer_handle);
		ESP_LOGW(TAG, "timer %d is already stop", id);
	} else {
		ESP_LOGE(TAG, "Unknown tmer mode");
	}
	return 0;
}

// Lua: tmr.alarm( id, interval, repeat, function )
static int tmr_alarm(lua_State* L){
	tmr_register(L);
	return tmr_start(L);
}

// Lua: tmr.stop( id )
static int tmr_stop(lua_State* L){
	uint8_t id = luaL_checkinteger(L, 1);
	if (id > NUM_TMR) {
		ESP_LOGE(TAG, "Error timer id");
		return 0;
	}
	my_timer_t tmr = &alarm_timers[id];
	if (tmr->mode == TIMER_MODE_OFF) {
		ESP_LOGW(TAG, "The timer is alread stop");
		return 0;
	}
	tmr->mode = TIMER_MODE_OFF;
	esp_timer_stop(tmr->timer_handle);
	return 0;  
}

// Lua: tmr.unregister( id )
static int tmr_unregister(lua_State* L){
	uint8_t id = luaL_checkinteger(L, 1);
	if (id > NUM_TMR) {
		ESP_LOGE(TAG, "Invalid timer id");
		return 0;
	}
	my_timer_t tmr = &alarm_timers[id];
	if (tmr->timer_handle == NULL) {
		ESP_LOGW(TAG, "The timer is alread unregister");
		return 0;
	}
	if(tmr->lua_ref != LUA_NOREF)
		luaL_unref(L, LUA_REGISTRYINDEX, tmr->lua_ref);
	tmr->lua_ref = LUA_NOREF;
	tmr->mode = TIMER_MODE_OFF; 
	esp_timer_stop(tmr->timer_handle);
	esp_timer_delete(tmr->timer_handle);
	tmr->timer_handle = NULL;
	return 0;
}

// Lua: tmr.interval( id, interval )
static int tmr_interval(lua_State* L){
	uint8_t id = luaL_checkinteger(L, 1);
	if (id > NUM_TMR) {
		ESP_LOGE(TAG, "Error timer id");
		return 0;
	}
	my_timer_t tmr = &alarm_timers[id];
	sint32_t interval = luaL_checkinteger(L, 2);
	if(interval <= 0)
		return luaL_error(L, "wrong arg range");
	if(tmr->mode != TIMER_MODE_OFF && tmr->timer_handle != NULL){	
		tmr->interval = interval;
		esp_timer_stop(tmr->timer_handle);
	}
	if(tmr->mode == TIMER_MODE_SINGLE) {
		esp_timer_start_once(tmr->timer_handle, tmr->interval); 
	} else if (tmr->mode == TIMER_MODE_PERIODIC) {
		esp_timer_start_periodic(tmr->timer_handle, tmr->interval);
	}
	return 0;
}

// Lua: tmr.state( id )
static int tmr_state(lua_State* L){
	uint8_t id = luaL_checkinteger(L, 1);
	if (id >= NUM_TMR) {
		ESP_LOGE(TAG, "Error timer id");
		return 0;
	}
	my_timer_t tmr = &alarm_timers[id];
	if(tmr->mode == TIMER_MODE_OFF){
		lua_pushstring(L, "off");
	} else if (tmr->mode == TIMER_MODE_PERIODIC) {
		lua_pushinteger(L, "loop");
	} else if (tmr->mode == TIMER_MODE_SINGLE) {
		lua_pushinteger(L, "once");
	} else {
		lua_pushinteger(L, "unknown");
	}
	//lua_pushinteger(L, tmr->mode&(~TIMER_IDLE_FLAG));
	return 1;
}


//system_rtc_clock_cali_proc() returns
//a fixed point value (12 bit fraction part)
//it tells how many rtc clock ticks represent 1us.
//the high 64 bits of the uint64_t multiplication
//are unnedded (I did the math)
static uint32_t rtc2sec(uint64_t rtc){
	//uint64_t aku = system_get_rtc_time();
	//aku *= rtc;
	//return (aku>>12)/1000000;
	uint32_t expired = xTaskGetTickCount();
	return expired;
}

//the following function workes, I just wrote it and didn't use it.
/*static uint64_t sec2rtc(uint32_t sec){
	uint64_t aku = (1<<20)/system_rtc_clock_cali_proc();
	aku *= sec;
	return (aku>>8)*1000000;
}*/

inline static void rtc_timer_update(){
	uint32_t current = xTaskGetTickCount(); //system_get_rtc_time();
	if(rtc_time.part[0] > current) //overflow check
		rtc_time.part[1]++;
	rtc_time.part[0] = current;
}

void rtc_callback(void *arg){
	rtc_timer_update();
	if(soft_watchdog > 0){
		soft_watchdog--;
		if(soft_watchdog == 0)
			esp_restart();
	}
}

// Lua: tmr.time() , return rtc time in second
static int tmr_time( lua_State* L ){
	rtc_timer_update();
	lua_pushinteger(L, rtc2sec(rtc_time.block));
	return 1; 
}

// Lua: tmr.softwd( value )
static int tmr_softwd( lua_State* L ){
	soft_watchdog = luaL_checkinteger(L, 1);
	return 0; 
}

// Module function map

static void setup_const(lua_State *L)
{
	lua_pushinteger(L, TIMER_MODE_SINGLE);
	lua_setfield(L, -2, "ALARM_SINGLE");
	lua_pushinteger(L, TIMER_MODE_PERIODIC);
	lua_setfield(L, -2, "ALARM_PERIODIC");
	lua_pushinteger(L, TIMER_MODE_OFF);
	lua_setfield(L, -2, "ALARM_OFF");
}

const LUA_REG_TYPE tmr_map[] = {
	{ LSTRKEY( "delay_us" ), LFUNCVAL( tmr_delay ) },
	{ LSTRKEY( "delay_ms" ), LFUNCVAL( tmr_delay_ms ) },
	{ LSTRKEY( "delay" ), LFUNCVAL( tmr_delay_s ) },
	{ LSTRKEY( "now" ), LFUNCVAL( tmr_now ) },
	//{ LSTRKEY( "softwd" ), LFUNCVAL( tmr_softwd ) },
	{ LSTRKEY( "time" ), LFUNCVAL( tmr_time ) },
	{ LSTRKEY( "register" ), LFUNCVAL ( tmr_register ) },
	{ LSTRKEY( "alarm" ), LFUNCVAL( tmr_alarm ) },
	{ LSTRKEY( "start" ), LFUNCVAL ( tmr_start ) },
	{ LSTRKEY( "stop" ), LFUNCVAL ( tmr_stop ) },
	{ LSTRKEY( "unregister" ), LFUNCVAL ( tmr_unregister ) },
	{ LSTRKEY( "state" ), LFUNCVAL ( tmr_state ) },
	{ LSTRKEY( "interval" ), LFUNCVAL ( tmr_interval) }, 
	{ LNILKEY, LNILVAL }
};

void tmr_init(void) {
	esp_timer_init();
	// timer task init
	sem = xSemaphoreCreateCounting(1, 1); //xSemaphoreCreateCounting(NUM_TMR, 0);
	if (sem == NULL) {
		ESP_LOGE(TAG, "Create semaphore failed!");
	}
	for (int i = 0; i < NUM_TMR; i++) {
		alarm_timers[i].mode = TIMER_MODE_OFF;
		alarm_timers[i].timer_handle = NULL;
	}
}

LUALIB_API int luaopen_tmr(lua_State *L)
{
  luaL_register( L, LUA_TMRLIBNAME, tmr_map );
  setup_const(L);
  tmr_init();
  return 1;

}



