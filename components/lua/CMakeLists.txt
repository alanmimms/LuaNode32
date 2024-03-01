idf_component_register(SRCS "lapi.c" "lcode.c" "ldo.c" "lfunc.c" "llex.c" "loadlib.c" "lparser.c" "lstring.c" "ltablib.c" "lundump.c" "lauxlib.c" "ldblib.c" "ldump.c" "lgc.c" "lmathlib.c" "lobject.c" "lrotable.c" "lstrlib.c" "ltm.c" "lvm.c" "lbaselib.c" "ldebug.c" "legc.c" "liolib.c" "lmem.c" "lopcodes.c" "lstate.c" "ltable.c" "lua.c" "lzio.c" "linit.c"
                    REQUIRES vfs mylibc
					INCLUDE_DIRS "include")
