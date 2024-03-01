#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#define UDP_TASK_STACK_SIZE			2048
#define UDP_TASK_PRIO				12
#define UART_TASK_STACK_SIZE		2048
#define UART_TASK_PRIO				11
#define UART_EVT_TASK_STACK_SIZE	2048
#define UART_EVT_TASK_PRIO			10
#define TMP_TASK_STACK_SIZE			2048
#define TMP_TASK_PRIO				9
#define LED_TASK_STACK_SIZE			2048
#define LED_TASK_PRIO				13
#define BUF_SIZE					1024
#define RECV_BUF_SIZE				128
#define WIFI_SSID_MAX_LEN			32
#define WIFI_PASS_MAX_LEN			32
#define SRV_IP_MAX_LEN				16

#define BDA_SIZE					16
#define UUID_SIZE					64
#define WIFI_CHECK_TIMEOUT			15

// user commands
#define CMD_SSID	"ssid"
#define CMD_PASS	"pass"
#define CMD_IP		"ip"
#define CMD_PORT	"port"
#define CMD_HELP	"help"
#define CMD_WIFI	"wifi"
#define CMD_QUIT	"quit"


#define PROMPT						"\r\n"
#define PROMPT_LEN					2

//#define ENABLE_SCAN_OUTPUT			1


extern void set_panic(void);
extern void clear_panic(void);

#endif
