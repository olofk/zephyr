/*
 * Copyright (c) 2019 Western Digital Corporation or its affiliates
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spi_context.h"

#define SPI_CFG(dev)  ((struct simple_spi_cfg  *) ((dev)->config->config_info))
#define SPI_DATA(dev) ((struct simple_spi_data *) ((dev)->driver_data))

#define SPI_REG(dev, offset) ((mem_addr_t) (SPI_CFG(dev)->base + (offset)))

struct simple_spi_cfg {
	u32_t base;
	u32_t f_sys;
};

struct simple_spi_data {
	struct spi_context ctx;
};

#define SPI_SPCR (0x0 * CONFIG_SPI_SIMPLE_SPI_BUS_WIDTH / 8)
#define SPI_SPSR (0x1 * CONFIG_SPI_SIMPLE_SPI_BUS_WIDTH / 8)
#define SPI_SPDR (0x2 * CONFIG_SPI_SIMPLE_SPI_BUS_WIDTH / 8)
#define SPI_SPER (0x3 * CONFIG_SPI_SIMPLE_SPI_BUS_WIDTH / 8)
#define SPI_SPSS (0x4 * CONFIG_SPI_SIMPLE_SPI_BUS_WIDTH / 8)

#define SPI_SPCR_SPE BIT(6)
