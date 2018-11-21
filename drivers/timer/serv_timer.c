/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <device.h>
#include <system_timer.h>
#include <board.h>

static volatile u32_t *timer = (u32_t *)SERV_TIMER_BASE;

/*
 * The SERV timer is a one shot timer that needs to be rearm upon
 * every interrupt. Timer clock is a 32-bits ART.
 * To arm timer, we need to read the RTC value and update the
 * timer compare register by the RTC value + time interval we want timer
 * to interrupt.
 */
static ALWAYS_INLINE void serv_rearm_timer(void)
{
	/*
	 * Disable timer interrupt while rearming the timer
	 * to avoid generation of interrupts while setting
	 * the mtimecmp->val_low register.
	 */
	irq_disable(SERV_TIMER_IRQ);

	/*
	 * Rearm timer to generate an interrupt after
	 * sys_clock_hw_cycles_per_tick
	 */
	timer = timer + sys_clock_hw_cycles_per_tick;

	/* Enable timer interrupt */
	irq_enable(SERV_TIMER_IRQ);
}

static void serv_timer_irq_handler(void *unused)
{
	ARG_UNUSED(unused);
#ifdef CONFIG_EXECUTION_BENCHMARKING
	extern void read_timer_start_of_tick_handler(void);
	read_timer_start_of_tick_handler();
#endif

	_sys_clock_tick_announce();

	/* Rearm timer */
	serv_rearm_timer();

#ifdef CONFIG_EXECUTION_BENCHMARKING
	extern void read_timer_end_of_tick_handler(void);
	read_timer_end_of_tick_handler();
#endif
}

#ifdef CONFIG_TICKLESS_IDLE
#error "Tickless idle not yet implemented for riscv-machine timer"
#endif

int _sys_clock_driver_init(struct device *device)
{
	ARG_UNUSED(device);

	IRQ_CONNECT(SERV_TIMER_IRQ, 0,
		    serv_timer_irq_handler, NULL, 0);

	/* Initialize timer, just call serv_rearm_timer */
	serv_rearm_timer();

	return 0;
}

/**
 *
 * @brief Read the platform's timer hardware
 *
 * This routine returns the current time in terms of timer hardware clock
 * cycles.
 *
 * @return up counter of elapsed clock cycles
 */
u32_t _timer_cycle_get_32(void)
{
	return *timer;
}
