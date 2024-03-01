#ifndef __UDP_TASK_H__
#define __UDP_TASK_H__

void connect_server(const char *ip, int port);
void send_data(const char *data, int len);
void disconnect_server();
void send_all_data(void);

#endif
