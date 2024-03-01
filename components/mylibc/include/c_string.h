/*
 * c_string.h
 *
 * Definitions for memory and string functions.
 */

#ifndef _C_STRING_H_
#define	_C_STRING_H_
#include "c_stddef.h"
//#include "osapi.h"
//#include "esp_libc.h"
#include <string.h>

#ifndef NULL
#define NULL 0
#endif

#define c_memcmp memcmp
#define c_memcpy memcpy
#define c_memset memset

#define os_memcpy ets_memcpy
#define os_memcmp ets_memcmp
#define os_memmove ets_memmove

#define c_strcat strcat
#define c_strchr strchr
#define c_strcmp strcmp
#define c_strcpy strcpy
#define c_strlen strlen
#define c_strncmp strncmp
#define c_strncpy strncpy
// #define c_strstr os_strstr
#define c_strncasecmp strncmp

#define c_strstr strstr
#define c_strncat strncat
#define c_strcspn strcspn
#define c_strpbrk strpbrk
#define c_strcoll strcoll
#define c_strrchr strrchr

//extern unsigned int strlen(char *s);
//extern char *strcat(char *dest, char *src);
//extern char *strchr(const char *s, int c);
//extern char *strcpy(char* des, const char* source);
//extern char *strncpy(char *dest, const char *src, size_t n);
//extern void *memcpy(void *dest, const void *src, size_t n);
extern void *memchr(const void *buf, int ch, size_t count);
extern char *strpbrk(const char *s1, const char *s2);

// const char *c_strstr(const char * __s1, const char * __s2);
// char *c_strncat(char * __restrict /*s1*/, const char * __restrict /*s2*/, size_t n);
// size_t c_strcspn(const char * s1, const char * s2);
// const char *c_strpbrk(const char * /*s1*/, const char * /*s2*/);
// int c_strcoll(const char * /*s1*/, const char * /*s2*/);

#endif /* _C_STRING_H_ */
