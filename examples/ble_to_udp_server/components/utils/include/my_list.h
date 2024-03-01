#ifndef __MY_LIST_H__
#define __MY_LIST_H_

typedef struct scan_list {
    char				*bda;
	char				*uuid;
	int					rssi;
    struct scan_list	*pNext;
} scan_list_t;

// prototype
void list_init(void);
scan_list_t *list_new_item(void);
void list_insert_to_head(scan_list_t *item);
void list_destroy(void);
scan_list_t *list_get_head(void);

#endif
