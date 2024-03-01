#ifndef _LUA_DEF_H_
#define _LUA_DEF_H_


/*
@@ LUA_MAXINPUT is the maximum length for an input line in the
@* stand-alone interpreter.
** CHANGE it if you need longer lines.
*/
#define LUA_MAXINPUT	256

/*
@@ LUA_PROMPT is the default prompt used by stand-alone Lua.
@@ LUA_PROMPT2 is the default continuation prompt used by stand-alone Lua.
** CHANGE them if you want different prompts. (You can also change the
** prompts dynamically, assigning to globals _PROMPT/_PROMPT2.)
*/
#define LUA_PROMPT		"> "
#define LUA_PROMPT2		">> "

// EGC operations modes
#define EGC_NOT_ACTIVE        0   // EGC disabled
#define EGC_ON_ALLOC_FAILURE  1   // run EGC on allocation failure
#define EGC_ON_MEM_LIMIT      2   // run EGC when an upper memory limit is hit
#define EGC_ALWAYS            4   // always run EGC before an allocation

/*
** pseudo-indices
*/
#define LUA_REGISTRYINDEX	(-10000)
#define LUA_ENVIRONINDEX	(-10001)
#define LUA_GLOBALSINDEX	(-10002)


#define GCSpause	0

#define GCSTEPSIZE	1024u

typedef LUAI_UMEM lu_mem;
#define MAX_LUMEM	((lu_mem)(~(lu_mem)0)-2)

#define G(L)	(L->l_G)


typedef struct __lua_load{
  lua_State *L;
  int firstline;
  char *line;
  int line_position;
  size_t len;
  int done;
  const char *prmt;
}lua_Load;


#endif
