// libutils.a
#include <stdlib.h>
#include "lauxlib.h"
#include "c_string.h"
//#include "md5.h"
#include "lualib.h"
#include "esp_log.h"
#include "lrodefs.h"


#define TAG "utils"
 
// base64 转换表, 共64个
static const char base64_alphabet[] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G',
    'H', 'I', 'J', 'K', 'L', 'M', 'N',
    'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z',
    'a', 'b', 'c', 'd', 'e', 'f', 'g',
    'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't',
    'u', 'v', 'w', 'x', 'y', 'z',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '+', '/'};
 
// 解码时使用    base64DecodeChars
static const unsigned char base64_suffix_map[256] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 253, 255,
    255, 253, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 253, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255,  62, 255, 255, 255,  63,
    52,  53,  54,  55,  56,  57,  58,  59,  60,  61, 255, 255,
    255, 254, 255, 255, 255,   0,   1,   2,   3,   4,   5,   6,
    7,   8,   9,  10,  11,  12,  13,  14,  15,  16,  17,  18,
    19,  20,  21,  22,  23,  24,  25, 255, 255, 255, 255, 255,
    255,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,
    37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,
    49,  50,  51, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255 };
 
static char cmove_bits(unsigned char src, unsigned lnum, unsigned rnum) {
    src <<= lnum; 
    src >>= rnum;
    return src;
}
 
static int base64_encode1(char *indata, int inlen, char *outdata, int *outlen) {
    
    int ret = 0; // return value
    if (indata == NULL || inlen == 0) {
        return ret = -1;
    }
    
    int in_len = 0; // 源字符串长度, 如果in_len不是3的倍数, 那么需要补成3的倍数
    int pad_num = 0; // 需要补齐的字符个数, 这样只有2, 1, 0(0的话不需要拼接, )
    if (inlen % 3 != 0) {
        pad_num = 3 - inlen % 3;
    }
    in_len = inlen + pad_num; // 拼接后的长度, 实际编码需要的长度(3的倍数)
    
    int out_len = in_len * 8 / 6; // 编码后的长度
    
    char *p = outdata; // 定义指针指向传出data的首地址
    
    //编码, 长度为调整后的长度, 3字节一组
    for (int i = 0; i < in_len; i+=3) {
        int value = *indata >> 2; // 将indata第一个字符向右移动2bit(丢弃2bit)
        char c = base64_alphabet[value]; // 对应base64转换表的字符
        *p = c; // 将对应字符(编码后字符)赋值给outdata第一字节
        
        //处理最后一组(最后3字节)的数据
        if (i == inlen + pad_num - 3 && pad_num != 0) {
            if(pad_num == 1) {
                *(p + 1) = base64_alphabet[(int)(cmove_bits(*indata, 6, 2) + cmove_bits(*(indata + 1), 0, 4))];
                *(p + 2) = base64_alphabet[(int)cmove_bits(*(indata + 1), 4, 2)];
                *(p + 3) = '=';
            } else if (pad_num == 2) { // 编码后的数据要补两个 '='
                *(p + 1) = base64_alphabet[(int)cmove_bits(*indata, 6, 2)];
                *(p + 2) = '=';
                *(p + 3) = '=';
            }
        } else { // 处理正常的3字节的数据
            *(p + 1) = base64_alphabet[cmove_bits(*indata, 6, 2) + cmove_bits(*(indata + 1), 0, 4)];
            *(p + 2) = base64_alphabet[cmove_bits(*(indata + 1), 4, 2) + cmove_bits(*(indata + 2), 0, 6)];
            *(p + 3) = base64_alphabet[*(indata + 2) & 0x3f];
        }
        
        p += 4;
        indata += 3;
    }
    
    if(outlen != NULL) {
        *outlen = out_len;
    }
    
    return ret;
}
 
 
static int base64_decode1(const char *indata, int inlen, char *outdata, int *outlen) {
    
    int ret = 0;
    if (indata == NULL || inlen <= 0 || outdata == NULL || outlen == NULL) {
        return ret = -1;
    }
    if (inlen % 4 != 0) { // 需要解码的数据不是4字节倍数
        return ret = -2;
    } 
    
    int t = 0, x = 0, y = 0, i = 0;
    unsigned char c = 0;
    int g = 3;
    
    //while (indata[x] != 0) {
    while (x < inlen) {
        // 需要解码的数据对应的ASCII值对应base64_suffix_map的值
        c = base64_suffix_map[(uint32_t)(indata[x++])];
        if (c == 255) return -1;// 对应的值不在转码表中
        if (c == 253) continue;// 对应的值是换行或者回车
        if (c == 254) { c = 0; g--; }// 对应的值是'='
        t = (t<<6) | c; // 将其依次放入一个int型中占3字节
        if (++y == 4) {
            outdata[i++] = (unsigned char)((t>>16)&0xff);
            if (g > 1) outdata[i++] = (unsigned char)((t>>8)&0xff);
            if (g > 2) outdata[i++] = (unsigned char)(t&0xff);
            y = t = 0;
        }
    }
    if (outlen != NULL) {
        *outlen = i;
    }
    return ret;
}


/**
 * translate Hex to String
 * pbDest: the memory to stored encoding string
 * pbSrc: the input hex
 * nLen: the length of hex
 */
void HexToStr(unsigned char *pbDest, unsigned char *pbSrc, int nLen)
{
    char ddl,ddh;
    int i;

    for (i=0; i<nLen; i++) {
        ddh = 48 + pbSrc[i] / 16;
        ddl = 48 + pbSrc[i] % 16;
        if (ddh > 57) ddh = ddh + 7;
        if (ddl > 57) ddl = ddl + 7;
        pbDest[i*2] = ddh;
        pbDest[i*2+1] = ddl;
    }

    pbDest[nLen*2] = '\0';
}

// return the lenght of encoding string
static int base64_encode0( lua_State* L ) {
	int argc = lua_gettop(L);
	const char *str = lua_tostring(L, 1);
	char *buff = malloc(strlen(str) * 2);
	if (buff == NULL) {
		ESP_LOGE(TAG, "malloc failed");
		return -1;
	}
	memset(buff, 0, strlen(str)*2);
	int out_len = 0;
	int ret = base64_encode1(str, strlen(str), buff, &out_len);
	if (ret < 0) {
		ESP_LOGE(TAG, "base64 encode failed");
	}
	lua_pushstring(L, buff);
	return ret;
}

// return the length of decoding string
static int base64_decode0( lua_State* L ) {
	int argc = lua_gettop(L); 
	const char *str = lua_tostring(L, 1);
	char *buff = malloc(strlen(str) * 2);
	if (buff == NULL) {
		ESP_LOGE(TAG, "malloc failed");
		return -1;
	}
	memset(buff, 0, strlen(str)*2);
	int out_len = 0;
	int ret = base64_decode1(str, strlen(str), buff, &out_len);
	if (ret < 0) {
		ESP_LOGE(TAG, "base64 decode failed");
	}
	lua_pushstring(L, buff);
	return 1;
}

/*static int leapyear0( lua_State* L ) {
	int year = lua_tointeger(L, 1);
	int isLeap = isLeapyear(year);
	lua_pushboolean(L, isLeap);
	return 1;
}*/

// md5 encode, return the encoding string
/*static int md5_encode( lua_State* L ) {
	unsigned char *str = (unsigned char *)lua_tostring(L, 1);	// string to be encoded
	char digest[16];
	char ecrypt[32];
	memset(digest, 0, 16);
	memset(ecrypt, 0, 32);
	MD5_CTX md5;
    MD5Init(&md5);
	MD5Update(&md5, str, strlen(str));
    MD5Final(&md5, digest);
	HexToStr(ecrypt, digest, strlen(digest));
	lua_pushstring(L, ecrypt);
	return 1;
}*/

// Module function map
const LUA_REG_TYPE utils_map[] = {
	{ LSTRKEY( "base64_encode" ), LFUNCVAL( base64_encode0 ) },
	{ LSTRKEY( "base64_decode" ), LFUNCVAL( base64_decode0 ) },
	//{ LSTRKEY( "md5_encode" ), LFUNCVAL( md5_encode ) },
	//{ LSTRKEY( "leapyear" ), LFUNCVAL( leapyear0 ) },
	{ LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_utils(lua_State *L)
{
	luaL_register( L, LUA_UTILSLIBNAME, utils_map );
	return 1;
}
