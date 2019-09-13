/*
 * Copyright (c) 2019 Western Digital Corporation or its affiliates
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RISCV32_SWERVOLF_SOC_H_
#define __RISCV32_SWERVOLF_SOC_H_

#include <soc_common.h>
#include <generated_dts_board.h>

#define DT_SIMPLE_SPI_0_BASE_ADDRESS 0x80001040
#define DT_SIMPLE_SPI_0_LABEL "spi0"

#define DT_JEDEC_SPI_NOR_0_LABEL "what_should_be_here"
#define DT_JEDEC_SPI_NOR_0_SIZE (16*1024*1024)
#define DT_JEDEC_SPI_NOR_0_BUS_NAME DT_SIMPLE_SPI_0_LABEL
#define DT_JEDEC_SPI_NOR_0_SPI_MAX_FREQUENCY 2000000
#define DT_JEDEC_SPI_NOR_0_BASE_ADDRESS 0
#define DT_JEDEC_SPI_NOR_0_JEDEC_ID_0 0x01
#define DT_JEDEC_SPI_NOR_0_JEDEC_ID_1 0x20
#define DT_JEDEC_SPI_NOR_0_JEDEC_ID_2 0x18
#define DT_JEDEC_SPI_NOR_0_WRITE_BLOCK_SIZE 256
#define DT_JEDEC_SPI_NOR_0_ERASE_BLOCK_SIZE 4096


#define LED0_GPIO_CONTROLLER         "LED0"
#define LED0_GPIO_PIN                0

#define RISCV_MTIME_BASE             0x80001020
#define RISCV_MTIMECMP_BASE          0x80001028

/* Also define the following for Zephyr 1.14 compatibility */
#define CONFIG_RISCV_RAM_BASE_ADDR DT_SRAM_BASE_ADDRESS
#define CONFIG_RISCV_RAM_SIZE      DT_SRAM_SIZE

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               DT_SRAM_BASE_ADDRESS
#define RISCV_RAM_SIZE               KB(DT_SRAM_SIZE)

#endif /* __RISCV32_SERV_SOC_H_ */
