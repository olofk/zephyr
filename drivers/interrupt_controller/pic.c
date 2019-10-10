/*
 * Copyright (c) 2019 <thomas.pcheng@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief PIC driver
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <init.h>
#include <sw_isr_table.h>
#include <soc.h>
#include <sys_io.h>
#include <logging/log.h>

#define LOG_MODULE_NAME swerv_pic
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define PIC_MAX_NUM			CONFIG_NUM_IRQS
#define PIC_MAX_ID			(PIC_MAX_NUM + RISCV_MAX_GENERIC_IRQ)
#define PIC_MAX_PRIO		16

#define PIC_mpiccfg 		0x3000
#define PIC_meipl(s)		(0x0 + (s)*4)
#define PIC_meip(x)			(0x1000 + (x)*4)
#define PIC_meie(s)			(0x2000 + (s)*4)
#define PIC_meigwctrl(s)	(0x4000 + (s)*4)
#define PIC_meigwclr(s)		(0x5000 + (s)*4)

#define PIC_meivt			"0xBC8"
#define PIC_meipt			"0xBC9"
#define PIC_meicpct			"0xBCA"
#define PIC_meicidpl		"0xBCB"
#define PIC_meicurpl		"0xBCC"
#define PIC_meihap			"0xFC8"

#define piccsr(csr) PIC_##csr

#define pic_readcsr(csr, value) \
	__asm__ volatile("csrr %0, "piccsr(csr) : "=r" (value));
#define pic_writecsr(csr, value) \
	__asm__ volatile("csrw "piccsr(csr)", %0" :: "rK" (value));

static int save_irq;

static u32_t pic_read(u32_t reg)
{
#ifdef DT_RISCV_PIC_0_BASE_ADDRESS
	return *(volatile u32_t *)(DT_RISCV_PIC_0_BASE_ADDRESS + reg);
#endif
}

static void pic_write(u32_t reg, u32_t val)
{
#ifdef DT_RISCV_PIC_0_BASE_ADDRESS
	*(volatile u32_t *)(DT_RISCV_PIC_0_BASE_ADDRESS + reg) = val;
#endif
}

void riscv_pic_irq_enable(u32_t irq)
{
	u32_t key;

	if ((irq >= PIC_MAX_ID) || (irq < RISCV_MAX_GENERIC_IRQ)){
		LOG_ERR("riscv_pic_irq_enable irq %d is illegal\n", irq);
		return;
	}

	LOG_DBG("riscv_pic_irq_enable %d\n", irq);

	key = irq_lock();
	pic_write(PIC_meie(irq - RISCV_MAX_GENERIC_IRQ), 1);
	irq_unlock(key);
}

void riscv_pic_irq_disable(u32_t irq)
{
	u32_t key;

	if ((irq >= PIC_MAX_ID) || (irq < RISCV_MAX_GENERIC_IRQ)){
		LOG_ERR("riscv_pic_irq_disable irq %d is illegal\n", irq);
		return;
	}

	LOG_DBG("riscv_pic_irq_disable %d\n", irq);

	key = irq_lock();
	pic_write(PIC_meie(irq - RISCV_MAX_GENERIC_IRQ), 0);
	irq_unlock(key);
}

int riscv_pic_irq_is_enabled(u32_t irq)
{
	if ((irq >= PIC_MAX_ID) || (irq < RISCV_MAX_GENERIC_IRQ)){
		LOG_ERR("riscv_pic_irq_is_enabled irq %d is illegal\n", irq);
		return -1;
	}

	return pic_read(PIC_meie(irq - RISCV_MAX_GENERIC_IRQ)) & 0x1;
}

void riscv_pic_set_priority(u32_t irq, u32_t priority)
{
	u32_t key;

	if (irq <= RISCV_MAX_GENERIC_IRQ)
		return;

	if ((irq >= PIC_MAX_ID) || (irq < RISCV_MAX_GENERIC_IRQ)){
		LOG_ERR("riscv_pic_set_priority irq %d is illegal\n", irq);
		return;
	}

	if (priority >= PIC_MAX_PRIO){
		LOG_ERR("riscv_pic_set_priority prio %d greater than max\n", priority);
		return;
	}

	LOG_DBG("riscv_pic_set_priority %d prio %d\n", irq, priority);

	key = irq_lock();
	pic_write(PIC_meipl(irq - RISCV_MAX_GENERIC_IRQ), priority);
	irq_unlock(key);
}

int riscv_pic_get_irq(void)
{
	return save_irq;
}

static void pic_irq_handler(void *arg)
{
	u32_t tmp;
	u32_t irq;
	struct _isr_table_entry *ite;

	/* trigger the capture of the interrupt source ID */
	pic_writecsr (meicpct, 0);

	pic_readcsr (meihap, tmp);
	irq = (tmp >> 2) & 0xff;

	save_irq = irq;

	if (irq == 0U || irq >= 64) {
		LOG_DBG("pic_irq_handler irq spurious %d\n", irq);
		z_irq_spurious(NULL);
	}
	irq += RISCV_MAX_GENERIC_IRQ;

	/* Call the corresponding IRQ handler in _sw_isr_table */
	ite = (struct _isr_table_entry *)&_sw_isr_table[irq];
	if (ite->isr)
		ite->isr(ite->arg);
	else
		LOG_DBG("_sw_isr_table is NULL\n");

	pic_write(PIC_meigwclr(irq), 0);
}

void pic_set_vector(void * func)
{
	LOG_DBG("pic_set_vector 0x%x\n", (u32_t)func);

	__asm__ volatile (
	"mv		t0, %0\n\t"
    "csrw	"piccsr(meivt)", t0\n\t"
	: "=r"(func)
	);
}

void pic_dummy()
{
	
}

static void pic_init_core(struct device *dev)
{
	ARG_UNUSED(dev);

	/* No interrupts masked */
	pic_writecsr (meipt, 0);
	pic_writecsr (meicidpl, 0);
	pic_writecsr (meicurpl, 0);

	pic_set_vector(pic_dummy);
}

static int pic_init(struct device *dev)
{
	ARG_UNUSED(dev);
	int i;

	LOG_DBG("pic_init enter\n");

	/* Init priority order to 0, 0=lowest to 15=highest */
	pic_write(PIC_mpiccfg, 0);

	/* Ensure that all interrupts are disabled initially */
	for (i = 1; i < PIC_MAX_ID; i++) {
		pic_write(PIC_meie(i), 0);
	}

	/* Set priority of each interrupt line to 0 initially */
	for (i = 1; i < PIC_MAX_ID; i++) {
		pic_write(PIC_meipl(i), 15);
	}

	/* Set property of each interrupt line to level-triggered/high */
	for (i = 1; i < PIC_MAX_ID; i++) {
		pic_write(PIC_meigwctrl(i), (0<<1)|(0<<0));
	}

	/* clear pending of each interrupt line */
	for (i = 1; i < PIC_MAX_ID; i++) {
		pic_write(PIC_meigwclr(i), 0);
	}

	pic_init_core(dev);

	/* Setup IRQ handler for PLIC driver */
	IRQ_CONNECT(RISCV_MACHINE_EXT_IRQ,
		    0,
		    pic_irq_handler,
		    NULL,
		    0);

	/* Enable IRQ for PLIC driver */
	irq_enable(RISCV_MACHINE_EXT_IRQ);

	return 0;
}

SYS_INIT(pic_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
