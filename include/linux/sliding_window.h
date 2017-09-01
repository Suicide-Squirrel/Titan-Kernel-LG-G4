#ifndef _SLIDING_WINDOW_H_
#define _SLIDING_WINDOW_H_

#include <linux/spinlock.h>
#include <linux/types.h>

enum slw_val {
	SLW_NONE = 0,
	SLW_READ,
	SLW_WRITE
};

struct sliding_window {
	u32 *__window;		/* initialized to some given size */
	u32 __size;		/* size of the sliding window in words */
	u32 *__major;		/* pointer into window */
	u32 __offset;		/* offset of the 0x3 mask in __major */
	spinlock_t __lock;	/* statistic read/write lock */
	u32 width;		/* sliding window width */
	u32 stat[2];		/* statistic table */
};

int slw_init(struct sliding_window *slw, u32 width);
void slw_uninit(struct sliding_window *slw);
int slw_resize(struct sliding_window *slw, u32 width);

bool slw_advance(struct sliding_window *slw, enum slw_val val_new);
void slw_reset(struct sliding_window *slw);

u32 slw_width_get(struct sliding_window *slw);
u32 slw_val_get(struct sliding_window *slw, enum slw_val val);
#endif
