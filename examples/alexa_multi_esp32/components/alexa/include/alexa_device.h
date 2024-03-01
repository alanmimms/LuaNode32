// Copyright 2015-2016 Doctors of Intelligence & Technology (Shenzhen) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __ALEXA_DEVICE_H__
#define __ALEXA_DEVICE_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/api.h"

typedef struct alexa_dev {
	char dev_name[20];
	int index;
	int sock_id;
	int local_port;
	void (*turn_on_cb)(void);
	void (*turn_off_cb)(void);
	TaskHandle_t task_handle;
	struct alexa_dev *next;
} alexa_dev_t;

alexa_dev_t *create_new_device(const char *name, int port, void (*on_cb)(void), void (*off_cb)(void));
void alexa_device_add(alexa_dev_t *dev);
void alexa_device_remove(alexa_dev_t *dev);
void alexa_device_list_init(void);
alexa_dev_t *alexa_device_get_dev_by_index(int index);
int alexa_device_get_num(void);
void alexa_device_list_destroy(void);
void alexa_device_list_traverse(void (*cb)(alexa_dev_t *dev));
void response_to_search(struct netconn *udpconn, unsigned char *addr, int port);

#endif
