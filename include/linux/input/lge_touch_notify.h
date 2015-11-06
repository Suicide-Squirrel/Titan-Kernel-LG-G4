#ifndef __LINUX_TOUCH_NOTIFY_H
#define __LINUX_TOUCH_NOTIFY_H

#include <linux/notifier.h>


/* the dsv on */
#define LCD_EVENT_TOUCH_LPWG_ON		0x01
#define LCD_EVENT_TOUCH_LPWG_OFF	0x02

#define LCD_EVENT_TOUCH_PWR_OFF      0XFF
/* to let lcd-driver know touch-driver's status */
#define LCD_EVENT_TOUCH_DRIVER_REGISTERED	0x03
/* For notifying proxy status to operate ENA control in lcd driver*/
#define LCD_EVENT_TOUCH_PROXY_STATUS 0X04
#define LCD_EVENT_TOUCH_SLEEP_STATUS 0X05
#define LCD_EVENT_TOUCH_SWIPE_STATUS 0X06
/* For PPlus */
#define LCD_EVENT_HW_RESET			0x07
#define LCD_EVENT_LCD_MODE			0x08

struct touch_event {
	void *data;
};


int touch_register_client(struct notifier_block *nb);
int touch_unregister_client(struct notifier_block *nb);
int touch_notifier_call_chain(unsigned long val, void *v);
#endif /* _LINUX_TOUCH_NOTIFY_H */
