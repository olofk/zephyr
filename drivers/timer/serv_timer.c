#include <drivers/system_timer.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <soc.h>

#define CYC_PER_TICK ((u32_t)((u32_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC	\
			      / (u32_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define MAX_TICKS ((0xffffffffu - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY 1000

#define TICKLESS (IS_ENABLED(CONFIG_TICKLESS_KERNEL) &&		\
		  !IS_ENABLED(CONFIG_QEMU_TICKLESS_WORKAROUND))

static struct k_spinlock lock;
static u32_t last_count;

static inline u32_t mtime(void)
{
	return sys_read32(SERV_TIMER_BASE);
}

static void timer_isr(void *arg)
{
	ARG_UNUSED(arg);

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t now = mtime();
	u32_t dticks = (u32_t)((now - last_count) / CYC_PER_TICK);

	last_count += dticks * CYC_PER_TICK;

	if (!TICKLESS) {
		u32_t next = last_count + CYC_PER_TICK;

		if ((s32_t)(next - now) < MIN_DELAY) {
			next += CYC_PER_TICK;
		}
		sys_write32(next, SERV_TIMER_BASE);
	}

	k_spin_unlock(&lock, key);
	z_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? dticks : 1);
}

int z_clock_driver_init(struct device *device)
{
	IRQ_CONNECT(SERV_TIMER_IRQ, 0, timer_isr, NULL, 0);
	sys_write32(mtime() + CYC_PER_TICK, SERV_TIMER_BASE);
	irq_enable(SERV_TIMER_IRQ);
	return 0;
}

void z_clock_set_timeout(s32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#if defined(CONFIG_TICKLESS_KERNEL) && !defined(CONFIG_QEMU_TICKLESS_WORKAROUND)
	/* RISCV has no idle handler yet, so if we try to spin on the
	 * logic below to reset the comparator, we'll always bump it
	 * forward to the "next tick" due to MIN_DELAY handling and
	 * the interrupt will never fire!  Just rely on the fact that
	 * the OS gave us the proper timeout already.
	 */
	if (idle) {
		return;
	}

	ticks = ticks == K_FOREVER ? MAX_TICKS : ticks;
	ticks = MAX(MIN(ticks - 1, (s32_t)MAX_TICKS), 0);

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t now = mtime();
	u32_t cyc = ticks * CYC_PER_TICK;

	/* Round up to next tick boundary.  Note use of 32 bit math,
	 * max_ticks is calibrated to permit this.
	 */
	cyc += (u32_t)(now - last_count) + (CYC_PER_TICK - 1);
	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;

	if ((s32_t)(cyc + last_count - now) < MIN_DELAY) {
		cyc += CYC_PER_TICK;
	}

	sys_write32(cyc + last_count, SERV_TIMER_BASE);
	k_spin_unlock(&lock, key);
#endif
}

u32_t z_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t ret = ((u32_t)mtime() - (u32_t)last_count) / CYC_PER_TICK;

	k_spin_unlock(&lock, key);
	return ret;
}

u32_t z_timer_cycle_get_32(void)
{
	return (u32_t)mtime();
}
