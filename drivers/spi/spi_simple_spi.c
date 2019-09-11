/*
 * Copyright (c) 2019 Western Digital Corporation or its affiliates
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_simple_spi);

#include <sys_io.h>
#include <spi.h>

#include "spi_context.h"
#include "spi_simple_spi.h"

/*
   Bit 5:4 == ESPR
   Bit 1:0 == SPR
 */
u8_t DIVIDERS[] = { 0x00,       //   2
		    0x01,       //   4
		    0x10,       //   8
		    0x02,       //  16
		    0x03,       //  32
		    0x11,       //  64
		    0x12,       // 128
		    0x13,       // 256
		    0x20,       // 512
		    0x21,       // 1024
		    0x22,       // 2048
		    0x23 };     // 4096

int simple_spi_transceive(struct device *dev,
			  const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs,
			  const struct spi_buf_set *rx_bufs)
{
	struct spi_context *ctx = &SPI_DATA(dev)->ctx;

	u8_t rx_byte;

	/* Lock the SPI Context */
	spi_context_lock(ctx, false, NULL);

	/* Set clock divider */
	int i;
	for (i = 0; i < 12; i++)
		if ((config->frequency << (i + 1)) > CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) {
			break;
		}

	sys_write8((DIVIDERS[i] >> 4) & 0x3, SPI_REG(dev, SPI_SPER));
	u8_t spcr = DIVIDERS[i] & 0x3;

	/* Set chip select */
	if (config->cs == NULL) {
		sys_write8(1 << config->slave, SPI_REG(dev, SPI_SPSS));
	}

	/* TODO: Parse operations and other config fields */

	/* Enable SPI controller */
	sys_write8(spcr | SPI_SPCR_SPE, SPI_REG(dev, SPI_SPCR));

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	while (spi_context_tx_buf_on(ctx) || spi_context_tx_buf_on(ctx)) {
		size_t cur_xfer_len = spi_context_longest_current_buf(ctx);
		for (size_t i = 0; i < cur_xfer_len; i++) {

			/* Write byte */
			sys_write8(*ctx->tx_buf, SPI_REG(dev, SPI_SPDR));
			__asm__ volatile ("fence;\n\t");

			spi_context_update_tx(ctx, 1, 1);

			/* Wait for rx FIFO empty flag to clear */
			while (sys_read8(SPI_REG(dev, SPI_SPSR)) & 0x1);

			/* Get received byte */
			rx_byte = sys_read8(SPI_REG(dev, SPI_SPDR));

			/* Store received byte if rx buffer is on */
			if (spi_context_rx_on(ctx)) {
				*ctx->rx_buf = rx_byte;
				spi_context_update_rx(ctx, 1, 1);
			}
		}
	}

	/* Clear chip-select */
	sys_write8(0, SPI_REG(dev, SPI_SPSS));

	spi_context_complete(ctx, 0);
	int rc = spi_context_wait_for_completion(ctx);

	spi_context_release(ctx, rc);

	return rc;
}

int simple_spi_release(struct device *dev, const struct spi_config *config)
{
	spi_context_unlock_unconditionally(&SPI_DATA(dev)->ctx);
	return 0;
}

static struct spi_driver_api simple_spi_api = {
	.transceive = simple_spi_transceive,
	.release = simple_spi_release,
};

int simple_spi_init(struct device *dev)
{
	/* Clear chip selects */
	sys_write8(0, SPI_REG(dev, SPI_SPSS));

	/* Make sure the context is unlocked */
	spi_context_unlock_unconditionally(&SPI_DATA(dev)->ctx);
	return 0;
}

static struct simple_spi_cfg simple_spi_cfg_0 =
{
	.base = DT_SIMPLE_SPI_0_BASE_ADDRESS,
};

static struct simple_spi_data simple_spi_data_0 =
{
	SPI_CONTEXT_INIT_LOCK(simple_spi_data_0, ctx),
	SPI_CONTEXT_INIT_SYNC(simple_spi_data_0, ctx),
};

DEVICE_AND_API_INIT(simple_spi_0,
		    DT_SIMPLE_SPI_0_LABEL,
		    simple_spi_init,
		    &simple_spi_data_0,
		    &simple_spi_cfg_0,
		    POST_KERNEL,
		    CONFIG_SPI_INIT_PRIORITY,
		    &simple_spi_api);
