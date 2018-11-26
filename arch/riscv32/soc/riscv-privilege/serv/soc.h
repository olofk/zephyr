#ifndef __RISCV32_SERV_SOC_H_
#define __RISCV32_SERV_SOC_H_

#include <soc_common.h>


/* Timer configuration */
#define SERV_TIMER_BASE             0x80000000
#define SERV_TIMER_IRQ              7

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               CONFIG_RISCV_RAM_BASE_ADDR
#define RISCV_RAM_SIZE               CONFIG_RISCV_RAM_SIZE

#endif /* __RISCV32_SERV_SOC_H_ */
