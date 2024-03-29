// Module for interfacing with system
#include <stdio.h>
//#include "modules.h"
#include "lauxlib.h"

#include "ldebug.h"
#include "ldo.h"
#include "lfunc.h"
#include "lmem.h"
#include "lobject.h"
#include "lstate.h"
#include "lualib.h"

#include "lopcodes.h"
#include "lstring.h"
#include "lundump.h"

#include "platform.h"
#include "lrotable.h"
#include "lrodefs.h"

#include "c_types.h"
//#include "romfs.h"
#include "c_string.h"
#include "rom/uart.h"
//#include "user_interface.h"
//#include "flash_api.h"
//#include "flash_fs.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_log.h"

#define CPU80MHZ 80
#define CPU160MHZ 160

#define TAG	"node"

#define FS_NAME_MAX_LENGTH 255

// Lua: restart()
static int node_restart( lua_State* L )
{
  lua_node_restart();
  return 0;
}

// Lua: dsleep( us, option )
static int node_dsleep( lua_State* L )
{
  uint64_t us = luaL_optinteger (L, 1, 0);
  lua_node_sleep(us);
  return 0;
}


// Lua: dsleep_set_options
// Combined to dsleep( us, option )
// static int node_deepsleep_setoption( lua_State* L )
// {
//   s32 option;
//   option = luaL_checkinteger( L, 1 );
//   if ( option < 0 || option > 4)
//     return luaL_error( L, "wrong arg range" );
//   else
//    deep_sleep_set_option( option );
//   return 0;
// }
// Lua: info()


static int node_info( lua_State* L )
{
	//uint32_t fs = system_get_flash_size();
	uint32_t fs = 2;
	uint8 id = 0;
	//bool succeed = system_get_chip_id(&id);
	uint32_t hs = lua_node_get_heap_size();
	lua_getglobal(L, "print");
	switch(fs) {
	case 0:
		lua_pushfstring(L, "flash_size=%s, chip_id=%d, heap_size=%d", "1MB", id, hs);
		break;
	case 1:
		lua_pushfstring(L, "flash_size=%s, chip_id=%d, heap_size=%d", "2MB", id, hs);
		break;
	case 2:
		lua_pushfstring(L, "flash_size=%s, chip_id=%d, heap_size=%d", "4MB", id, hs);
		break;
	case 3:
		lua_pushfstring(L, "flash_size=%s, chip_id=%d, heap_size=%d", "8MB", id, hs);
		break;
	case 4:
		lua_pushfstring(L, "flash_size=%s, chip_id=%d, heap_size=%d", "16MB", id, hs);
		break;
	default:
		lua_pushfstring(L, "flash_size=%s, chip_id=%d, heap_size=%d", "32MB", id, hs);
		break;
	}
	int err = lua_pcall(L, 1, 1, 0);
	if (err != 0) { ESP_LOGE(TAG, "lua_pcall failed:%s", lua_tostring(L, -1)); }
	lua_pop(L, 1) ;
	return 0;
}

// Lua: chipid()
static int node_chipid( lua_State* L )
{
  uint8 id = 0;
  //bool succeed = system_get_chip_id(&id);
  lua_getglobal(L, "print");
  lua_pushinteger(L, id);
  int err = lua_pcall(L, 1, 1, 0);
  if (err != 0) { ESP_LOGE(TAG, "lua_pcall failed:%s", lua_tostring(L, -1)); }
  lua_pop(L, 1) ;
  return 0;
}

// deprecated, moved to adc module
// Lua: readvdd33()
// static int node_readvdd33( lua_State* L )
// {
//   uint32_t vdd33 = readvdd33();
//   lua_pushinteger(L, vdd33);
//   return 1;
// }

// Lua: flashid()
static int node_flashid( lua_State* L )
{
  //uint32_t id = spi_flash_get_id();
  //lua_pushinteger( L, id );
  lua_getglobal(L, "print");
  lua_pushinteger(L, 0);
  int err = lua_pcall(L, 1, 1, 0);
  if (err != 0) { ESP_LOGE(TAG, "lua_pcall failed:%s", lua_tostring(L, -1)); }
  lua_pop(L, 1) ;
  return 0;
}

// Lua: flashsize()
static int node_flashsize( lua_State* L )
{
  //uint32_t sz = system_get_flash_size();
  uint32_t sz = 2;
  lua_getglobal(L, "print");
  switch(sz) {
	case 0:
		lua_pushstring(L, "1MB");
		break;
	case 1:
		lua_pushstring(L, "2MB");
		break;
	case 2:
		lua_pushstring(L, "4MB");
		break;
	case 3:
		lua_pushstring(L, "8MB");
		break;
	case 4:
		lua_pushstring(L, "16MB");
		break;
	default:
		lua_pushstring(L, "32MB");
		break;
  }
  //lua_pushinteger(L, sz);
  int err = lua_pcall(L, 1, 1, 0);
  if (err != 0) { ESP_LOGE(TAG, "lua_pcall failed:%s", lua_tostring(L, -1)); }
  lua_pop(L, 1) ;
  return 0;
}

// Lua: heap()
static int node_heap( lua_State* L )
{
  uint32_t sz = lua_node_get_heap_size();
  lua_getglobal(L, "print");
  lua_pushinteger(L, sz);
  int err = lua_pcall(L, 1, 1, 0);
  if (err != 0) { ESP_LOGE(TAG, "lua_pcall failed:%s", lua_tostring(L, -1)); }
  lua_pop(L, 1) ;
  return 0;
}

static int node_memusage( lua_State* L )
{
  uint32_t usage = lua_gc(L, LUA_GCCOUNT, 0);
  lua_getglobal(L, "print");
  lua_pushinteger(L, usage);
  int err = lua_pcall(L, 1, 1, 0);
  if (err != 0) { ESP_LOGE(TAG, "lua_pcall failed:%s", lua_tostring(L, -1)); }
  lua_pop(L, 1) ;
  return 0;
}

static lua_State *gL = NULL;

#ifdef DEVKIT_VERSION_0_9
static int led_high_count = LED_HIGH_COUNT_DEFAULT;
static int led_low_count = LED_LOW_COUNT_DEFAULT;
static int led_count = 0;
static int key_press_count = 0;
static bool key_short_pressed = false;
static bool key_long_pressed = false;
static os_timer_t keyled_timer;
static int long_key_ref = LUA_NOREF;
static int short_key_ref = LUA_NOREF;

static void default_long_press(void *arg) { 
  if (led_high_count == 12 && led_low_count == 12) {
    led_low_count = led_high_count = 6;
  } else {
    led_low_count = led_high_count = 12;
  }
  // led_high_count = 1000 / READLINE_INTERVAL;
  // led_low_count = 1000 / READLINE_INTERVAL;
  // NODE_DBG("default_long_press is called. hc: %d, lc: %d\n", led_high_count, led_low_count);
}

static void default_short_press(void *arg) {
  system_restart();
}

static void key_long_press(void *arg) {
  ESP_LOGI(TAG, "key_long_press is called.");
  if (long_key_ref == LUA_NOREF) {
    default_long_press(arg);
    return;
  }
  if (!gL)
    return;
  lua_rawgeti(gL, LUA_REGISTRYINDEX, long_key_ref);
  lua_call(gL, 0, 0);
}

static void key_short_press(void *arg) {
  ESP_LOGI(TAG, "key_short_press is called.");
  if (short_key_ref == LUA_NOREF) {
    default_short_press(arg);
    return;
  }
  if (!gL)
    return;
  lua_rawgeti(gL, LUA_REGISTRYINDEX, short_key_ref);
  lua_call(gL, 0, 0);
}

static void update_key_led (void *p)
{
  (void)p;
  uint8_t temp = 1, level = 1;
  led_count++;
  if(led_count>led_low_count+led_high_count){
    led_count = 0;    // reset led_count, the level still high
  } else if(led_count>led_low_count && led_count <=led_high_count+led_low_count){
    level = 1;    // output high level
  } else if(led_count<=led_low_count){
    level = 0;    // output low level
  }
  temp = platform_key_led(level);
  if(temp == 0){      // key is pressed
    key_press_count++;
    if(key_press_count>=KEY_LONG_COUNT){
      // key_long_press(NULL);
      key_long_pressed = true;
      key_short_pressed = false;
      // key_press_count = 0;
    } else if(key_press_count>=KEY_SHORT_COUNT){    // < KEY_LONG_COUNT
      // key_short_press(NULL);
      key_short_pressed = true;
    }
  }else{  // key is released
    key_press_count = 0;
    if(key_long_pressed){
      key_long_press(NULL);
      key_long_pressed = false;
    }
    if(key_short_pressed){
      key_short_press(NULL);
      key_short_pressed = false;
    }
  }
}

static void prime_keyled_timer (void)
{
  os_timer_disarm (&keyled_timer);
  os_timer_setfn (&keyled_timer, update_key_led, 0);
  os_timer_arm (&keyled_timer, KEYLED_INTERVAL, 1);
}

// Lua: led(low, high)
static int node_led( lua_State* L )
{
  int low, high;
  if ( lua_isnumber(L, 1) )
  {
    low = lua_tointeger(L, 1);
    if ( low < 0 ) {
      return luaL_error( L, "wrong arg type" );
    }
  } else {
    low = LED_LOW_COUNT_DEFAULT; // default to LED_LOW_COUNT_DEFAULT
  }
  if ( lua_isnumber(L, 2) )
  {
    high = lua_tointeger(L, 2);
    if ( high < 0 ) {
      return luaL_error( L, "wrong arg type" );
    }
  } else {
    high = LED_HIGH_COUNT_DEFAULT; // default to LED_HIGH_COUNT_DEFAULT
  }
  led_high_count = (uint32_t)high / READLINE_INTERVAL;
  led_low_count = (uint32_t)low / READLINE_INTERVAL;
  prime_keyled_timer();
  return 0;
}

// Lua: key(type, function)
static int node_key( lua_State* L )
{
  int *ref = NULL;
  size_t sl;

  const char *str = luaL_checklstring( L, 1, &sl );
  if (str == NULL)
    return luaL_error( L, "wrong arg type" );

  if (sl == 5 && c_strcmp(str, "short") == 0) {
    ref = &short_key_ref;
  } else if (sl == 4 && c_strcmp(str, "long") == 0) {
    ref = &long_key_ref;
  } else {
    ref = &short_key_ref;
  }
  gL = L;
  // luaL_checkanyfunction(L, 2);
  if (lua_type(L, 2) == LUA_TFUNCTION || lua_type(L, 2) == LUA_TLIGHTFUNCTION) {
    lua_pushvalue(L, 2);  // copy argument (func) to the top of stack
    if (*ref != LUA_NOREF)
      luaL_unref(L, LUA_REGISTRYINDEX, *ref);
    *ref = luaL_ref(L, LUA_REGISTRYINDEX);
  } else {    // unref the key press function
    if (*ref != LUA_NOREF)
      luaL_unref(L, LUA_REGISTRYINDEX, *ref);
    *ref = LUA_NOREF;
  }

  prime_keyled_timer();
  return 0;
}
#endif

extern lua_Load gLoad;
// Lua: input("string")
static int node_input( lua_State* L )
{
  size_t l = 0;
  const char *s = luaL_checklstring(L, 1, &l);
  if (s != NULL && l > 0 && l < LUA_MAXINPUT - 1)
  {
    lua_Load *load = &gLoad;
    if (load->line_position == 0) {
      c_memcpy(load->line, s, l);
      load->line[l + 1] = '\0';
      load->line_position = c_strlen(load->line) + 1;
      load->done = 1;
      ESP_LOGI(TAG, "Get command:\n");
      ESP_LOGI(TAG, "%s", load->line); // buggy here
      ESP_LOGI(TAG, "\nResult(if any):\n");
      //system_os_post (LUA_TASK_PRIO, LUA_PROCESS_LINE_SIG, 0);
    }
  }
  return 0;
}

static int output_redir_ref = LUA_NOREF;
static int serial_debug = 1;
void output_redirect(const char *str) {
  // if(c_strlen(str)>=TX_BUFF_SIZE){
  //   NODE_ERR("output too long.\n");
  //   return;
  // }

  if (output_redir_ref == LUA_NOREF || !gL) {
    printf(str);
    return;
  }

  if (serial_debug != 0) {
    printf(str);
  }

  lua_rawgeti(gL, LUA_REGISTRYINDEX, output_redir_ref);
  lua_pushstring(gL, str);
  lua_call(gL, 1, 0);   // this call back function should never user output.
}

// Lua: output(function(c), debug)
static int node_output( lua_State* L )
{
  gL = L;
  // luaL_checkanyfunction(L, 1);
  if (lua_type(L, 1) == LUA_TFUNCTION || lua_type(L, 1) == LUA_TLIGHTFUNCTION) {
    lua_pushvalue(L, 1);  // copy argument (func) to the top of stack
    if (output_redir_ref != LUA_NOREF)
      luaL_unref(L, LUA_REGISTRYINDEX, output_redir_ref);
    output_redir_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  } else {    // unref the key press function
    if (output_redir_ref != LUA_NOREF)
      luaL_unref(L, LUA_REGISTRYINDEX, output_redir_ref);
    output_redir_ref = LUA_NOREF;
    serial_debug = 1;
    return 0;
  }

  if ( lua_isnumber(L, 2) )
  {
    serial_debug = lua_tointeger(L, 2);
    if (serial_debug != 0)
      serial_debug = 1;
  } else {
    serial_debug = 1; // default to 1
  }

  return 0;
}

static int writer(lua_State* L, const void* p, size_t size, void* u)
{
  UNUSED(L);
  FILE* file_fd = (FILE *)u;
  if (NULL == file_fd) {
    return 1;
  }
  ESP_LOGI(TAG, "get fd:%p,size:%d\n", file_fd, size);

  if (size != 0 && (size != fwrite((const char *)p, 1, size, file_fd)) ) {
    return 1;
  }
  ESP_LOGI(TAG, "write fd:%p,size:%d\n", file_fd, size);
  return 0;
}

#define toproto(L,i) (clvalue(L->top+(i))->l.p)
// Lua: compile(filename) -- compile lua file into lua bytecode, and save to .lc
static int node_compile( lua_State* L )
{
  Proto* f;
  FILE* file_fd = NULL;
  size_t len;
  const char *fname = luaL_checklstring( L, 1, &len );
  if ( len > FS_NAME_MAX_LENGTH ) {
    return luaL_error(L, "filename too long");
  }

  char output[FS_NAME_MAX_LENGTH];
  c_strcpy(output, fname);
  // check here that filename end with ".lua".
  if (len < 4 || (c_strcmp( output + len - 4, ".lua") != 0) ) {
    return luaL_error(L, "not a .lua file");
  }

  output[c_strlen(output) - 2] = 'c';
  output[c_strlen(output) - 1] = '\0';
  ESP_LOGI(TAG, "%s", output);
  ESP_LOGI(TAG, "\n");
  if (luaL_loadfsfile(L, fname) != 0) {
    return luaL_error(L, lua_tostring(L, -1));
  }

  f = toproto(L, -1);

  int stripping = 1;      /* strip debug information? */

  file_fd = fopen(output, "w+");
  if (file_fd == NULL) {
    return luaL_error(L, "cannot open/write to file");
  }

  lua_lock(L);
  int result = luaU_dump(L, f, writer, file_fd, stripping);
  lua_unlock(L);

  if (fflush(file_fd) < 0) {   // result codes aren't propagated by flash_fs.h
    // overwrite Lua error, like writer() does in case of a file io error
    result = 1;
  }
  fclose(file_fd);
  file_fd = NULL;

  if (result == LUA_ERR_CC_INTOVERFLOW) {
    return luaL_error(L, "value too big or small for target integer type");
  }
  if (result == LUA_ERR_CC_NOTINTEGER) {
    return luaL_error(L, "target lua_Number is integral but fractional value found");
  }
  if (result == 1) {    // result status generated by writer() or fs_flush() fail
    return luaL_error(L, "writing to file failed");
  }

  return 0;
}

#if 0
// Lua: setcpufreq(mhz)
// mhz is either CPU80MHZ od CPU160MHZ
static int node_setcpufreq(lua_State* L)
{
  // http://www.esp8266.com/viewtopic.php?f=21&t=1369
  uint32_t new_freq = luaL_checkinteger(L, 1);
  if (new_freq == CPU160MHZ){
    REG_SET_BIT(0x3ff00014, BIT(0));
    ets_update_cpu_frequency(CPU160MHZ);
  } else {
    REG_CLR_BIT(0x3ff00014,  BIT(0));
    ets_update_cpu_frequency(CPU80MHZ);
  }
  new_freq = ets_get_cpu_frequency();
  lua_pushinteger(L, new_freq);
  return 1;
}

// Lua: code = bootreason()
static int node_bootreason (lua_State *L)
{
  lua_pushnumber (L, rtc_get_reset_reason ());
  return 1;
}
#endif

// Lua: restore()
static int node_restore (lua_State *L)
{
  //flash_init_data_default();
  //flash_init_data_blank();
  lua_node_system_restore();
  return 0;
}

#ifdef LUA_OPTIMIZE_DEBUG
/* node.stripdebug([level[, function]]). 
 * level:    1 don't discard debug
 *           2 discard Local and Upvalue debug info
 *           3 discard Local, Upvalue and lineno debug info.
 * function: Function to be stripped as per setfenv except 0 not permitted.
 * If no arguments then the current default setting is returned.
 * If function is omitted, this is the default setting for future compiles
 * The function returns an estimated integer count of the bytes stripped.
 */
static int node_stripdebug (lua_State *L) {
  int level;

  if (L->top == L->base) {
    lua_pushlightuserdata(L, &luaG_stripdebug );
    lua_gettable(L, LUA_REGISTRYINDEX);
    if (lua_isnil(L, -1)) {
      lua_pop(L, 1);
      lua_pushinteger(L, LUA_OPTIMIZE_DEBUG);
    }
    return 1;
  }

  level = luaL_checkint(L, 1);
  if ((level <= 0) || (level > 3)) luaL_argerror(L, 1, "must in range 1-3");

  if (L->top == L->base + 1) {
    /* Store the default level in the registry if no function parameter */
    lua_pushlightuserdata(L, &luaG_stripdebug);
    lua_pushinteger(L, level);
    lua_settable(L, LUA_REGISTRYINDEX);
    lua_settop(L,0);
    return 0;
  }

  if (level == 1) {
    lua_settop(L,0);
    lua_pushinteger(L, 0);
    return 1;
  }

  if (!lua_isfunction(L, 2)) {
    int scope = luaL_checkint(L, 2);
    if (scope > 0) { 
      /* if the function parameter is a +ve integer then climb to find function */
      lua_Debug ar;
      lua_pop(L, 1); /* pop level as getinfo will replace it by the function */
      if (lua_getstack(L, scope, &ar)) {
        lua_getinfo(L, "f", &ar);
      }
    }
  }

  if(!lua_isfunction(L, 2) || lua_iscfunction(L, -1)) luaL_argerror(L, 2, "must be a Lua Function");
  // lua_lock(L);
  Proto *f = clvalue(L->base + 1)->l.p;
  // lua_unlock(L);
  lua_settop(L,0);
  lua_pushinteger(L, luaG_stripdebug(L, f, level, 1));
  return 1;
}
#endif

// Module function map
const LUA_REG_TYPE node_map[] =
{
  { LSTRKEY( "restart" ), LFUNCVAL( node_restart ) },
  { LSTRKEY( "dsleep" ), LFUNCVAL( node_dsleep ) },
  { LSTRKEY( "info" ), LFUNCVAL( node_info ) },
  { LSTRKEY( "chipid" ), LFUNCVAL( node_chipid ) },
  { LSTRKEY( "flashid" ), LFUNCVAL( node_flashid ) },
  { LSTRKEY( "flashsize" ), LFUNCVAL( node_flashsize) },
  { LSTRKEY( "heap" ), LFUNCVAL( node_heap ) },
  { LSTRKEY( "memusage" ), LFUNCVAL( node_memusage ) },
#ifdef DEVKIT_VERSION_0_9
  { LSTRKEY( "key" ), LFUNCVAL( node_key ) },
  { LSTRKEY( "led" ), LFUNCVAL( node_led ) },
#endif
  { LSTRKEY( "input" ), LFUNCVAL( node_input ) },
  { LSTRKEY( "output" ), LFUNCVAL( node_output ) },
// Moved to adc module, use adc.readvdd33()
// { LSTRKEY( "readvdd33" ), LFUNCVAL( node_readvdd33) },
  { LSTRKEY( "compile" ), LFUNCVAL( node_compile) },
  //{ LSTRKEY( "CPU80MHZ" ), LNUMVAL( CPU80MHZ ) },
  //{ LSTRKEY( "CPU160MHZ" ), LNUMVAL( CPU160MHZ ) },
  //{ LSTRKEY( "setcpufreq" ), LFUNCVAL( node_setcpufreq) },
  //{ LSTRKEY( "bootreason" ), LFUNCVAL( node_bootreason) },
  { LSTRKEY( "restore" ), LFUNCVAL( node_restore) },
#ifdef LUA_OPTIMIZE_DEBUG
  { LSTRKEY( "stripdebug" ), LFUNCVAL( node_stripdebug ) },
#endif

// Combined to dsleep(us, option)
// { LSTRKEY( "dsleepsetoption" ), LFUNCVAL( node_deepsleep_setoption) },
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_node(lua_State *L)
{
	luaL_register( L, LUA_NODELIBNAME, node_map );
	return 1;
}
