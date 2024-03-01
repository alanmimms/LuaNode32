#ifndef _UTILS_H_
#define _UTILS_H_

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

void print_info(void);
//char *basename(char *path);
char * get_partition_label(void);

#endif
