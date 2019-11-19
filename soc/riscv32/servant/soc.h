#ifndef __RISCV32_SERVANT_SOC_H_
#define __RISCV32_SERVANT_SOC_H_

#include <soc_common.h>

#define DT_ALIAS_LED0_GPIOS_CONTROLLER "LED0"
#define DT_ALIAS_LED0_GPIOS_PIN        0
#define DT_ALIAS_LED0_GPIOS_ADDRESS    0x40000000
#define DT_ALIAS_LED0_GPIOS_MASK       0x00000001

#define LED0_GPIO_CONTROLLER "LED0"
#define LED0_GPIO_PIN 0
/* Timer configuration */
#define SERV_TIMER_BASE             0x80000000
#define SERV_TIMER_IRQ              7

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               CONFIG_RISCV_RAM_BASE_ADDR
#define RISCV_RAM_SIZE               CONFIG_RISCV_RAM_SIZE

#endif /* __RISCV32_SERV_SOC_H_ */
