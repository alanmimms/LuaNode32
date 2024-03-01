#ifndef __MY_UART_H__
#define __MY_UART_H__

#include <stdint.h>

typedef enum {
	NONE,
	INPUT_SSID,
	INPUT_PASS,
	INPUT_IP,
	INPUT_PORT,
} inputStatus;

typedef enum {
    EMPTY,
    UNDER_WRITE,
    WRITE_OVER
} RcvMsgBuffState;

typedef struct {
    uint32_t     RcvBuffSize;
    uint8_t     *pRcvMsgBuff;
    uint8_t     *pWritePos;
    uint8_t     *pReadPos;
    uint8_t      TrigLvl; //JLU: may need to pad
    RcvMsgBuffState  BuffState;
} RcvMsgBuff;


// prototype
void uart_init(void);
void show_help_info();

#endif
