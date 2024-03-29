/*
** $Id: ltm.c,v 2.8.1.1 2007/12/27 13:02:25 roberto Exp $
** Tag methods
** See Copyright Notice in lua.h
*/


#define ltm_c
#define LUA_CORE
#define LUAC_CROSS_FILE

#include "lua.h"
#include <string.h>

#include "lobject.h"
#include "lstate.h"
#include "lstring.h"
#include "ltable.h"
#include "ltm.h"
#include "lrotable.h"



const char *const luaT_typenames[] = {
  "nil", "boolean", "romtable", "lightfunction", "userdata", "number",
  "string", "table", "function", "userdata", "thread",
  "proto", "upval"
};


void luaT_init (lua_State *L) {
  static const char *const luaT_eventname[] = {  /* ORDER TM */
    "__index", "__newindex",
    "__gc", "__mode", "__eq",
    "__add", "__sub", "__mul", "__div", "__mod",
    "__pow", "__unm", "__len", "__lt", "__le",
    "__concat", "__call"
  };
  int i;
  for (i=0; i<TM_N; i++) {
    G(L)->tmname[i] = luaS_new(L, luaT_eventname[i]);
    luaS_fix(G(L)->tmname[i]);  /* never collect these names */
  }
}


/*
** function to be used with macro "fasttm": optimized for absence of
** tag methods
*/
const TValue *luaT_gettm (Table *events, TMS event, TString *ename) {
  const TValue *tm = luaR_isrotable(events) ? luaH_getstr_ro(events, ename) : luaH_getstr(events, ename); 
  lua_assert(event <= TM_EQ);
  if (ttisnil(tm)) {  /* no tag method? */
    if (!luaR_isrotable(events))
      events->flags |= cast_byte(1u<<event);  /* cache this fact */
    return NULL;
  }
  else return tm;
}


const TValue *luaT_gettmbyobj (lua_State *L, const TValue *o, TMS event) {
  Table *mt;
  switch (ttype(o)) {
    case LUA_TTABLE:
      mt = hvalue(o)->metatable;
      break;
    case LUA_TROTABLE:
      mt = (Table*)luaR_getmeta(rvalue(o));
      break;
    case LUA_TUSERDATA:
      mt = uvalue(o)->metatable;
      break;
    default:
      mt = G(L)->mt[ttype(o)];
  }
  if (!mt)
    return luaO_nilobject;
  else if (luaR_isrotable(mt))
    return luaH_getstr_ro(mt, G(L)->tmname[event]);
  else
    return luaH_getstr(mt, G(L)->tmname[event]);
}
