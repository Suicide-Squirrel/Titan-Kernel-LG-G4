#include <linux/slab.h>
#include <linux/sliding_window.h>

/* val == 0 is reserved for SLW_NONE, which is does not have a stat counter */
#define SLW_STAT(slw, val) (slw)->stat[(val)-1]

void slw_uninit(struct sliding_window *slw)
{
	kfree(slw->__window);
	memset(slw, 0, sizeof(struct sliding_window));
}
EXPORT_SYMBOL(slw_uninit);

static inline int __slw_window_alloc(u32 width, u32 **window, u32 *size)
{
	u32 __size, *__window;

	if (!width) {
		pr_err("sliding_window: invalid width: 0\n");
		return -EINVAL;
	}

	__size = DIV_ROUND_UP(2 * (width), 32); /* size in words */
	__window = (u32*)kzalloc(__size * sizeof(u32), GFP_KERNEL);
	if (!__window) {
		pr_err("sliding_window: unable to allocate storage for " \
			"monitoring %u elements\n", width);
		return -ENOMEM;
	}

	*window = __window;
	*size = __size;
	return 0;
}

static inline void __slw_window_setup(struct sliding_window *slw, u32 width,
	u32 *window, u32 size)
{
	slw->__window = window;
	slw->__size = size;

	slw->width = width;
	memset(slw->stat, 0, sizeof(slw->stat));

	slw->__major = slw->__window;
	slw->__offset = 0x0;
}

int slw_init(struct sliding_window *slw, u32 width)
{
	u32 ret, size, *window;

	ret = __slw_window_alloc(width, &window, &size);
	if (ret)
		return ret;

	__slw_window_setup(slw, width, window, size);
	spin_lock_init(&slw->__lock);

	return 0;
}
EXPORT_SYMBOL(slw_init);

int slw_resize(struct sliding_window *slw, u32 width)
{
	u32 ret, size, *window;
	unsigned long flags;

	ret = __slw_window_alloc(width, &window, &size);
	if (ret)
		return ret;

	spin_lock_irqsave(&slw->__lock, flags);

	kfree(slw->__window);
	__slw_window_setup(slw, width, window, size);
	spin_unlock_irqrestore(&slw->__lock, flags);
	return 0;
}
EXPORT_SYMBOL(slw_resize);

bool slw_advance(struct sliding_window *slw, enum slw_val val_new)
{
	unsigned long flags;
	enum slw_val val_stale;

	BUG_ON(!slw->width);

	spin_lock_irqsave(&slw->__lock, flags);
	val_stale = (*slw->__major & ((0x3) << slw->__offset)) >> slw->__offset;

	if (val_new == val_stale)
		goto out;

	/* update counters */
	*slw->__major &= ~(0x3 << slw->__offset);
	if (val_new)
		*slw->__major |= val_new << slw->__offset;

	if (val_stale && SLW_STAT(slw, val_stale))
		SLW_STAT(slw, val_stale)--;
	if (val_new && (SLW_STAT(slw, val_new) < UINT_MAX))
		SLW_STAT(slw, val_new)++;

out:
	/* update pointers */
	slw->__offset += 2;
	if ((31 < slw->__offset) ||
		((slw->__major == slw->__window + slw->__size - 1) &&
		 ((slw->width * 2 - (slw->__size - 1) * 32) - 1 <
		  slw->__offset))) {

		slw->__offset = 0x0;

		if (slw->__major < slw->__window + slw->__size - 1)
			slw->__major++;
		else
			slw->__major = slw->__window;
	}
	spin_unlock_irqrestore(&slw->__lock, flags);

	return true;
}
EXPORT_SYMBOL(slw_advance);

void slw_reset(struct sliding_window *slw)
{
	unsigned long flags;

	spin_lock_irqsave(&slw->__lock, flags);

	memset(slw->__window, 0, sizeof(u32) * slw->__size);
	memset(slw->stat, 0, sizeof(slw->stat));

	slw->__major = slw->__window;
	slw->__offset = 0x0;

	spin_unlock_irqrestore(&slw->__lock, flags);
}
EXPORT_SYMBOL(slw_reset);

u32 slw_width_get(struct sliding_window *slw)
{
	return slw->width;
}
EXPORT_SYMBOL(slw_width_get);

u32 slw_val_get(struct sliding_window *slw, enum slw_val val)
{
	u32 ret;
	unsigned long flags;

	BUG_ON(!slw->width);

	spin_lock_irqsave(&slw->__lock, flags);
	if (val == SLW_NONE) {
		ret = slw->width -
			(SLW_STAT(slw, SLW_READ) + SLW_STAT(slw, SLW_WRITE));
	} else {
		ret = SLW_STAT(slw, val);
	}
	spin_unlock_irqrestore(&slw->__lock, flags);

	return ret;
}
EXPORT_SYMBOL(slw_val_get);
