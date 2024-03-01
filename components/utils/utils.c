#include <stdio.h>
#include <string.h>
#include "utils.h"

void print_info(void)
{
	printf("	 _                    _   _             _       \n\
	| |                  | \\ | |           | |      \n\
	| |     _   _   __ _ |  \\| |  ___    __| |  ___ \n\
	| |    | | | | / _` || . ` | / _ \\  / _` | / _ \\\n\
	| |____| |_| || (_| || |\\  || (_) || (_| ||  __/\n\
	\\_____/ \\__,_| \\__,_|\\_| \\_/ \\___/  \\__,_| \\___|\n\
													\n\
	For ESP32										\n\
	Version 2.0.0\n\
													\n\
	------------------------------------------------\n\
													\n");
}

/*char *basename(char *path)
{
	int num = strlen(path);
	for (int i = num-1; i >= 0; i++) {
		if (path[i] == '/') {
			return path+i+1;
		}
	}
	return path;
}*/
