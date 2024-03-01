// Module for interfacing with file system

#include <string.h>
//#include "modules.h"
#include "lauxlib.h"
#include "platform.h"

#include "c_types.h"
#include "c_string.h"
#include "esp_vfs.h"
#include "esp_log.h"
#include "lualib.h"
#include "esp_spiffs.h"
#include "utils.h"
#include "lrodefs.h"

#define TAG "file"

static FILE* file_fd = NULL;

// Lua: open(filename, mode)
static int file_open( lua_State* L )
{
  size_t len;
  if(file_fd){
    fclose(file_fd);
    file_fd = NULL;
  }

  const char *fname = luaL_checklstring( L, 1, &len );
  const char *name = basename( fname );
  luaL_argcheck(L, strlen(name) <= 32 && strlen(fname) == len, 1, "filename invalid");

  const char *mode = luaL_optstring(L, 2, "r");
  //ESP_LOGI(TAG, "Open file: %s", fname);
  char full_path[64] = {0};
  sprintf(full_path, "%s/%s", LUA_INIT_FILE_DIR, fname);
  file_fd = fopen(full_path, mode);

  if(!file_fd){
    lua_pushnil(L);
  } else {
    lua_pushboolean(L, 1);
  } 
  return 1; 
}

// Lua: close()
static int file_close( lua_State* L )
{
  if(file_fd){
    fclose(file_fd);
    file_fd = NULL;
	//ESP_LOGI(TAG, "file close successfully\n");
  }
  return 0;  
}

// Lua: format()
static int file_format( lua_State* L )
{
  file_close(L);
  char *label = get_partition_label();
  ESP_LOGW(TAG, "Start formating...");
  if( ESP_OK != esp_spiffs_format(label) )
  {
    ESP_LOGE(TAG, "\n*** ERROR ***: unable to format. FS might be compromised.\n" );
    ESP_LOGE(TAG, "It is advised to re-flash the NodeMCU image.\n" );
    luaL_error(L, "Failed to format file system");
  }
  else{
    ESP_LOGI(TAG, "format done.\n" );
  }
  return 0;
}


// Lua: list()
static int file_list( lua_State* L )
{
  DIR  *dir;
  struct dirent *item;

  if ((dir = opendir(""))) {
    lua_newtable( L );
    while ((item = readdir(dir))) {
      lua_pushinteger(L, strlen(item->d_name));
      lua_setfield(L, -2, item->d_name);
    }
    closedir(dir);
    return 1;
  }
  return 0;
}

static int file_seek (lua_State *L) 
{
  static const int mode[] = {SEEK_SET, SEEK_CUR, SEEK_END};
  static const char *const modenames[] = {"set", "cur", "end", NULL};
  if(!file_fd)
    return luaL_error(L, "open a file first");
  int op = luaL_checkoption(L, 1, "cur", modenames);
  long offset = luaL_optlong(L, 2, 0);
  op = fseek(file_fd, offset, mode[op]);
  if (op < 0)
    lua_pushnil(L);  /* error */
  else
    lua_pushinteger(L, ftell(file_fd));
  return 1;
}

// Lua: remove(filename)
static int file_remove( lua_State* L )
{
  size_t len;
  const char *fname = luaL_checklstring( L, 1, &len );    
  const char *name = basename( fname );
  luaL_argcheck(L, strlen(name) <= 32 && strlen(fname) == len, 1, "filename invalid");
  file_close(L);
  remove((char *)fname);
  return 0; 
}

// Lua: flush()
static int file_flush( lua_State* L )
{
  if(!file_fd)
    return luaL_error(L, "open a file first");
  if(fflush(file_fd) == 0)
    lua_pushboolean(L, 1);
  else
    lua_pushnil(L);
  return 1;
}
#if 0
// Lua: check()
static int file_check( lua_State* L )
{
  file_close(L);
  lua_pushinteger(L, fs_check());
  return 1;
}
#endif

// Lua: rename("oldname", "newname")
static int file_rename( lua_State* L )
{
  size_t len;
  if(file_fd){
    fclose(file_fd);
    file_fd = NULL;
  }

  const char *oldname = luaL_checklstring( L, 1, &len );
  const char *name = basename( oldname );
  luaL_argcheck(L, strlen(name) <= 32 && strlen(oldname) == len, 1, "filename invalid");
  
  const char *newname = luaL_checklstring( L, 2, &len );  
  name = basename( newname );
  luaL_argcheck(L, strlen(name) <= 32 && strlen(newname) == len, 2, "filename invalid");

  if(0 <= rename( oldname, newname )){
    lua_pushboolean(L, 1);
  } else {
    lua_pushboolean(L, 0);
  }
  return 1;
}

// Lua: fsinfo()
static int file_fsinfo( lua_State* L )
{
  size_t total, used;
  char *label = get_partition_label();
  if (ESP_OK != esp_spiffs_info(NULL, &total, &used)) {
	ESP_LOGE(TAG, "get spiffs info failed");
    return luaL_error(L, "file system failed");
  }
  //ESP_LOGI(TAG, "total: %d, used:%d", total, used);
  if(total>0x7FFFFFFF || used>0x7FFFFFFF || used > total)
  {
    return luaL_error(L, "file system error");
  }
  lua_pushinteger(L, total-used);
  lua_pushinteger(L, used);
  lua_pushinteger(L, total);
  return 3;
}

// g_read()
static int file_g_read( lua_State* L, int n, int16_t end_char )
{
  if(n <= 0 || n > LUAL_BUFFERSIZE)
    n = LUAL_BUFFERSIZE;
  if(end_char < 0 || end_char >255)
    end_char = EOF;
  
  luaL_Buffer b;
  if(!file_fd)
    return luaL_error(L, "open a file first");

  luaL_buffinit(L, &b);
  char *p = luaL_prepbuffer(&b);
  int i;

  n = fread(p, 1, n, file_fd);
  for (i = 0; i < n; ++i)
    if (p[i] == end_char)
    {
      ++i;
      break;
    }

  if(i==0){
    luaL_pushresult(&b);  /* close buffer */
    return (lua_objlen(L, -1) > 0);  /* check whether read something */
  }

  fseek(file_fd, -(n - i), SEEK_CUR);
  luaL_addsize(&b, i);
  luaL_pushresult(&b);  /* close buffer */
  return 1;  /* read at least an `eol' */ 
}

// Lua: read()
// file.read() will read all byte in file
// file.read(10) will read 10 byte from file, or EOF is reached.
// file.read('q') will read until 'q' or EOF is reached. 
static int file_read( lua_State* L )
{
  unsigned need_len = LUAL_BUFFERSIZE;
  int16_t end_char = EOF;
  size_t el;
  if( lua_type( L, 1 ) == LUA_TNUMBER )
  {
    need_len = ( unsigned )luaL_checkinteger( L, 1 );
    if( need_len > LUAL_BUFFERSIZE ){
      need_len = LUAL_BUFFERSIZE;
    }
  }
  else if(lua_isstring(L, 1))
  {
    const char *end = luaL_checklstring( L, 1, &el );
    if(el!=1){
      return luaL_error( L, "wrong arg range" );
    }
    end_char = (int16_t)end[0];
  }

  return file_g_read(L, need_len, end_char);
}

// Lua: readline()
static int file_readline( lua_State* L )
{
  return file_g_read(L, LUAL_BUFFERSIZE, '\n');
}

// Lua: write("string")
static int file_write( lua_State* L )
{
  if(!file_fd)
    return luaL_error(L, "open a file first");
  size_t l, rl;
  const char *s = luaL_checklstring(L, 1, &l);
  rl = fwrite(s, 1, l, file_fd);
  if(rl==l)
    lua_pushboolean(L, 1);
  else
    lua_pushnil(L);
  return 1;
}

// Lua: writeline("string")
static int file_writeline( lua_State* L )
{
  if(!file_fd)
    return luaL_error(L, "open a file first");
  size_t l, rl;
  const char *s = luaL_checklstring(L, 1, &l);
  rl = fwrite(s, 1, l, file_fd);
  if(rl==l){
    rl = fwrite("\n", 1, 1, file_fd);
    if(rl==1)
      lua_pushboolean(L, 1);
    else
      lua_pushnil(L);
  }
  else{
    lua_pushnil(L);
  }
  return 1;
}

// Module function map
const LUA_REG_TYPE file_map[] = {
  { LSTRKEY( "list" ),      LFUNCVAL( file_list ) },
  { LSTRKEY( "open" ),      LFUNCVAL( file_open ) },
  { LSTRKEY( "close" ),     LFUNCVAL( file_close ) },
  { LSTRKEY( "write" ),     LFUNCVAL( file_write ) },
  { LSTRKEY( "writeline" ), LFUNCVAL( file_writeline ) },
  { LSTRKEY( "read" ),      LFUNCVAL( file_read ) },
  { LSTRKEY( "readline" ),  LFUNCVAL( file_readline ) },
  { LSTRKEY( "format" ),    LFUNCVAL( file_format ) },
#if defined(BUILD_SPIFFS)
  { LSTRKEY( "remove" ),    LFUNCVAL( file_remove ) },
  { LSTRKEY( "seek" ),      LFUNCVAL( file_seek ) },
  { LSTRKEY( "flush" ),     LFUNCVAL( file_flush ) },
//{ LSTRKEY( "check" ),     LFUNCVAL( file_check ) },
  { LSTRKEY( "rename" ),    LFUNCVAL( file_rename ) },
  { LSTRKEY( "fsinfo" ),    LFUNCVAL( file_fsinfo ) },
#endif
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_file(lua_State *L)
{
#if LUA_OPTIMIZE_MEMORY > 0
    return 0;
#else  
  luaL_register( L, LUA_FILELIBNAME, file_map );
  return 1;
#endif
}
