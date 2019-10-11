/*
 * Copyright (c) 2019
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* based on linux ethoc.c driver. */

#define LOG_MODULE_NAME eth_ethoc
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <soc.h>
#include <device.h>
#include <errno.h>
#include <init.h>
#include <kernel.h>
#include <misc/__assert.h>
#include <net/net_core.h>
#include <net/net_pkt.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys_io.h>
#include <net/ethernet.h>
#include "ethernet/eth_stats.h"

#ifdef CONFIG_SHARED_IRQ
#include <shared_irq.h>
#endif

/* register offsets */
#define	MODER		0x00
#define	INT_SOURCE	0x04
#define	INT_MASK	0x08
#define	IPGT		0x0c
#define	IPGR1		0x10
#define	IPGR2		0x14
#define	PACKETLEN	0x18
#define	COLLCONF	0x1c
#define	TX_BD_NUM	0x20
#define	CTRLMODER	0x24
#define	MIIMODER	0x28
#define	MIICOMMAND	0x2c
#define	MIIADDRESS	0x30
#define	MIITX_DATA	0x34
#define	MIIRX_DATA	0x38
#define	MIISTATUS	0x3c
#define	MAC_ADDR0	0x40
#define	MAC_ADDR1	0x44
#define	ETH_HASH0	0x48
#define	ETH_HASH1	0x4c
#define	ETH_TXCTRL	0x50
#define	ETH_END		0x54

/* PHY Registers       Indices */
#define ETHOC_PHY_BCONTROL   0
#define ETHOC_PHY_BSTATUS    1
#define ETHOC_PHY_ID1        2
#define ETHOC_PHY_ID2        3
#define ETHOC_PHY_ANEG_ADV   4
#define ETHOC_PHY_ANEG_LPA   5
#define ETHOC_PHY_ANEG_EXP   6
#define ETHOC_PHY_MCONTROL   17
#define ETHOC_PHY_MSTATUS    18
#define ETHOC_PHY_CSINDICATE 27
#define ETHOC_PHY_INTSRC     29
#define ETHOC_PHY_INTMASK    30
#define ETHOC_PHY_CS         31

/* mode register */
#define	MODER_RXEN	(1 <<  0) /* receive enable */
#define	MODER_TXEN	(1 <<  1) /* transmit enable */
#define	MODER_NOPRE	(1 <<  2) /* no preamble */
#define	MODER_BRO	(1 <<  3) /* broadcast address */
#define	MODER_IAM	(1 <<  4) /* individual address mode */
#define	MODER_PRO	(1 <<  5) /* promiscuous mode */
#define	MODER_IFG	(1 <<  6) /* interframe gap for incoming frames */
#define	MODER_LOOP	(1 <<  7) /* loopback */
#define	MODER_NBO	(1 <<  8) /* no back-off */
#define	MODER_EDE	(1 <<  9) /* excess defer enable */
#define	MODER_FULLD	(1 << 10) /* full duplex */
#define	MODER_RESET	(1 << 11) /* FIXME: reset (undocumented) */
#define	MODER_DCRC	(1 << 12) /* delayed CRC enable */
#define	MODER_CRC	(1 << 13) /* CRC enable */
#define	MODER_HUGE	(1 << 14) /* huge packets enable */
#define	MODER_PAD	(1 << 15) /* padding enabled */
#define	MODER_RSM	(1 << 16) /* receive small packets */

/* interrupt source and mask registers */
#define	INT_MASK_TXF	(1 << 0) /* transmit frame */
#define	INT_MASK_TXE	(1 << 1) /* transmit error */
#define	INT_MASK_RXF	(1 << 2) /* receive frame */
#define	INT_MASK_RXE	(1 << 3) /* receive error */
#define	INT_MASK_BUSY	(1 << 4)
#define	INT_MASK_TXC	(1 << 5) /* transmit control frame */
#define	INT_MASK_RXC	(1 << 6) /* receive control frame */

#define	INT_MASK_TX	(INT_MASK_TXF | INT_MASK_TXE)
#define	INT_MASK_RX	(INT_MASK_RXF | INT_MASK_RXE)

#define	INT_MASK_ALL ( \
		INT_MASK_TXF | INT_MASK_TXE | \
		INT_MASK_RXF | INT_MASK_RXE | \
		INT_MASK_TXC | INT_MASK_RXC | \
		INT_MASK_BUSY \
	)

/* MII mode register */
#define	MIIMODER_CLKDIV(x)	((x) & 0xfe) /* needs to be an even number */
#define	MIIMODER_NOPRE		(1 << 8) /* no preamble */

/* MII command register */
#define	MIICOMMAND_SCAN		(1 << 0) /* scan status */
#define	MIICOMMAND_READ		(1 << 1) /* read status */
#define	MIICOMMAND_WRITE	(1 << 2) /* write control data */

/* MII status register */
#define	MIISTATUS_LINKFAIL	(1 << 0)
#define	MIISTATUS_BUSY		(1 << 1)
#define	MIISTATUS_INVALID	(1 << 2)

/* MII address register */
#define	MIIADDRESS_FIAD(x)		(((x) & 0x1f) << 0)
#define	MIIADDRESS_RGAD(x)		(((x) & 0x1f) << 8)
#define	MIIADDRESS_ADDR(phy, reg)	(MIIADDRESS_FIAD(phy) | \
								MIIADDRESS_RGAD(reg))

#define RESET_TIMEOUT     K_MSEC(10)
#define PHY_RESET_TIMEOUT K_MSEC(100)
#define REG_WRITE_TIMEOUT K_MSEC(50)

/* TX buffer descriptor */
#define	TX_BD_CS		(1 <<  0) /* carrier sense lost */
#define	TX_BD_DF		(1 <<  1) /* defer indication */
#define	TX_BD_LC		(1 <<  2) /* late collision */
#define	TX_BD_RL		(1 <<  3) /* retransmission limit */
#define	TX_BD_RETRY_MASK	(0x00f0)
#define	TX_BD_RETRY(x)		(((x) & 0x00f0) >>  4)
#define	TX_BD_UR		(1 <<  8) /* transmitter underrun */
#define	TX_BD_CRC		(1 << 11) /* TX CRC enable */
#define	TX_BD_PAD		(1 << 12) /* pad enable for short packets */
#define	TX_BD_WRAP		(1 << 13)
#define	TX_BD_IRQ		(1 << 14) /* interrupt request enable */
#define	TX_BD_READY		(1 << 15) /* TX buffer ready */
#define	TX_BD_LEN(x)		(((x) & 0xffff) << 16)
#define	TX_BD_LEN_MASK		(0xffff << 16)

#define	TX_BD_STATS		(TX_BD_CS | TX_BD_DF | TX_BD_LC | \
				TX_BD_RL | TX_BD_RETRY_MASK | TX_BD_UR)

/* RX buffer descriptor */
#define	RX_BD_LC	(1 <<  0) /* late collision */
#define	RX_BD_CRC	(1 <<  1) /* RX CRC error */
#define	RX_BD_SF	(1 <<  2) /* short frame */
#define	RX_BD_TL	(1 <<  3) /* too long */
#define	RX_BD_DN	(1 <<  4) /* dribble nibble */
#define	RX_BD_IS	(1 <<  5) /* invalid symbol */
#define	RX_BD_OR	(1 <<  6) /* receiver overrun */
#define	RX_BD_MISS	(1 <<  7)
#define	RX_BD_CF	(1 <<  8) /* control frame */
#define	RX_BD_WRAP	(1 << 13)
#define	RX_BD_IRQ	(1 << 14) /* interrupt request enable */
#define	RX_BD_EMPTY	(1 << 15)
#define	RX_BD_LEN(x)	(((x) & 0xffff) << 16)

#define	RX_BD_STATS	(RX_BD_LC | RX_BD_CRC | RX_BD_SF | RX_BD_TL | \
			RX_BD_DN | RX_BD_IS | RX_BD_OR | RX_BD_MISS)

#define	ETHOC_ZLEN		64

/* Controller has only one PHY with address 1 */
#define PHY_ADDR 1

#define	ETHOC_BD_BASE		0x400
#define	ETHOC_BUFSIZ		1536

#define PRIORITY  7
#define ETHOC_THREAD_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(ethoc_thread_stack_area, ETHOC_THREAD_STACK_SIZE);
struct k_thread ethoc_thread_data;

static struct device DEVICE_NAME_GET(eth_ethoc_0);

#define ETHOC_PKT_BUF_LEN 1600
char __aligned(8) pkt_rx_buf[CONFIG_ETHOC_BD_RX_NUM][ETHOC_PKT_BUF_LEN];
char __aligned(8) pkt_tx_buf[CONFIG_ETHOC_BD_TX_NUM][ETHOC_PKT_BUF_LEN];

struct ethoc_bd {
	u32_t stat;
	u32_t addr;
};

struct eth_context {
	struct net_if *iface;
	k_tid_t tid;
	bool polled_mode;
	mem_addr_t iobase;

	u32_t limit;
	u32_t num_bd;
	u32_t num_tx;
	u32_t cur_tx;
	u32_t dty_tx;
	u8_t * pkt_tx_buf;
	void * pkt_tx[CONFIG_ETHOC_BD_TX_NUM];
	u32_t num_rx;
	u32_t cur_rx;
	u8_t * pkt_rx_buf;
	void * pkt_rx[CONFIG_ETHOC_BD_RX_NUM];

	u32_t eth_clkfreq;
	u32_t mask;
	u32_t phy;
	u8_t mac[6];
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

/* SMSC911x helper functions */

static u32_t ethoc_read(struct eth_context *ctx, u32_t reg)
{
	u32_t val;
	val = *(volatile u32_t *)(ctx->iobase + reg);
	return val;
}

static void ethoc_write(struct eth_context *ctx, u32_t reg, u32_t val)
{
	*(volatile u32_t *)(ctx->iobase + reg) = val;
}

int ethoc_phy_regread(struct eth_context *ctx, u32_t reg, u32_t *val)
{
	int i;
	u32_t status;
	u32_t data;

	ethoc_write(ctx, MIIADDRESS, MIIADDRESS_ADDR(ctx->phy, reg));
	ethoc_write(ctx, MIICOMMAND, MIICOMMAND_READ);

	for (i = 0; i < 1000; i++) {
		status = ethoc_read(ctx, MIISTATUS);
		if (!(status & MIISTATUS_BUSY)) {
			data = ethoc_read(ctx, MIIRX_DATA);
			/* reset MII command register */
			ethoc_write(ctx, MIICOMMAND, 0);
			*val = data;
			return 0;
		}
		k_sleep(1);
	}

	return -EBUSY;
}

int ethoc_phy_regwrite(struct eth_context *ctx, u8_t reg, u32_t val)
{
	int i;
	u32_t state;

	ethoc_write(ctx, MIIADDRESS, MIIADDRESS_ADDR(ctx->phy, reg));
	ethoc_write(ctx, MIITX_DATA, val);
	ethoc_write(ctx, MIICOMMAND, MIICOMMAND_WRITE);

	for (i = 0; i < 1000; i++) {
		state = ethoc_read(ctx, MIISTATUS);
		if (!(state & MIISTATUS_BUSY)) {
			/* reset MII command register */
			ethoc_write(ctx, MIICOMMAND, 0);
			return 0;
		}
		k_sleep(1);
	}

	return -EBUSY;
}

static void ethoc_set_mac_address(struct eth_context *ctx)
{
	u8_t *mac = (u8_t *) ctx->mac;

	ethoc_write(ctx, MAC_ADDR0, (mac[2] << 24) | (mac[3] << 16) |
				     (mac[4] <<  8) | (mac[5] <<  0));
	ethoc_write(ctx, MAC_ADDR1, (mac[0] <<  8) | (mac[1] <<  0));
}

static int ethoc_read_mac_address(struct eth_context *ctx)
{
	u8_t *mac = (u8_t *) ctx->mac;
	u32_t reg;

	reg = ethoc_read(ctx, MAC_ADDR0);
	mac[2] = (reg >> 24) & 0xff;
	mac[3] = (reg >> 16) & 0xff;
	mac[4] = (reg >>  8) & 0xff;
	mac[5] = (reg >>  0) & 0xff;

	reg = ethoc_read(ctx, MAC_ADDR1);
	mac[0] = (reg >>  8) & 0xff;
	mac[1] = (reg >>  0) & 0xff;

	return 0;
}

static void ethoc_reset(struct eth_context *ctx)
{
	u32_t tmp;

	/* disable tx and rx */
	tmp = ethoc_read(ctx, MODER);
	tmp &= ~(MODER_RXEN | MODER_TXEN);
	ethoc_write(ctx, MODER, tmp);

	/* enable FCS generation and automatic padding */
	tmp = ethoc_read(ctx, MODER);
	tmp |= MODER_CRC | MODER_PAD;
	ethoc_write(ctx, MODER, tmp);

	/* set full-duplex mode */
	tmp = ethoc_read(ctx, MODER);
	tmp |= MODER_FULLD;
	ethoc_write(ctx, MODER, tmp);
	ethoc_write(ctx, IPGT, 0x15);

	/* clear and disable all interrupts */
	ethoc_write(ctx, INT_SOURCE, INT_MASK_ALL);
	ethoc_write(ctx, INT_MASK, 0);
}

static int ethoc_check_phy(struct eth_context *ctx)
{
	u32_t phyid1, phyid2;

	if (ethoc_phy_regread(ctx, ETHOC_PHY_ID1, &phyid1)) {
		return -1;
	}

	if (ethoc_phy_regread(ctx, ETHOC_PHY_ID2, &phyid2)) {
		return -1;
	}

	LOG_DBG ("read phy id %x %x\n", phyid1, phyid2);

	return ((phyid1 == 0xFFFF && phyid2 == 0xFFFF) ||
	    (phyid1 == 0x0 && phyid2 == 0x0));
}

int ethoc_reset_phy(struct eth_context *ctx)
{
	u32_t val;

	if (ethoc_phy_regread(ctx, ETHOC_PHY_BCONTROL, &val)) {
		return -1;
	}

	val |= 1 << 15;

	if (ethoc_phy_regwrite(ctx, ETHOC_PHY_BCONTROL, val)) {
		return -1;
	}

	return 0;
}

static void ethoc_read_bd(struct eth_context *ctx, int index,
		struct ethoc_bd *bd)
{
	u32_t offset = ETHOC_BD_BASE + (index * sizeof(struct ethoc_bd));
	bd->stat = ethoc_read(ctx, offset + 0);
	bd->addr = ethoc_read(ctx, offset + 4);
}

static void ethoc_write_bd(struct eth_context *ctx, int index,
		const struct ethoc_bd *bd)
{
	u32_t offset = ETHOC_BD_BASE + (index * sizeof(struct ethoc_bd));
	ethoc_write(ctx, offset + 0, bd->stat);
	ethoc_write(ctx, offset + 4, bd->addr);
}

void ethoc_advertise_caps(struct eth_context *ctx)
{
	u32_t aneg_adv = 0U;

	ethoc_phy_regread(ctx, ETHOC_PHY_ANEG_ADV, &aneg_adv);
	aneg_adv |= 0xDE0;
	ethoc_phy_regwrite(ctx, ETHOC_PHY_ANEG_ADV, aneg_adv);
	ethoc_phy_regread(ctx, ETHOC_PHY_ANEG_ADV, &aneg_adv);
}

void ethoc_establish_link(struct eth_context *ctx)
{
	u32_t bcr = 0U;

	ethoc_phy_regread(ctx, ETHOC_PHY_BCONTROL, &bcr);
	bcr |= (1 << 12) | (1 << 9);
	ethoc_phy_regwrite(ctx, ETHOC_PHY_BCONTROL, bcr);
	ethoc_phy_regread(ctx, ETHOC_PHY_BCONTROL, &bcr);
}

static int ethoc_init_ring(struct eth_context *ctx)
{
	struct ethoc_bd bd;
	int i;

	ctx->cur_tx = 0;
	ctx->dty_tx = 0;
	ctx->cur_rx = 0;

	ctx->pkt_rx_buf = (u8_t *)pkt_rx_buf;
	ctx->pkt_tx_buf = (u8_t *)pkt_tx_buf;

	ethoc_write(ctx, TX_BD_NUM, ctx->num_tx);

	/* setup transmission buffers */

	bd.stat = TX_BD_IRQ | TX_BD_CRC;
	bd.addr = (u32_t)ctx->pkt_tx_buf;

	for (i = 0; i < ctx->num_tx; i++) {
		if (i == ctx->num_tx - 1)
			bd.stat |= TX_BD_WRAP;

		ethoc_write_bd(ctx, i, &bd);
		bd.addr += ETHOC_PKT_BUF_LEN;
		ctx->pkt_tx[i] = NULL;
	}

	bd.stat = RX_BD_EMPTY | RX_BD_IRQ;
	bd.addr = (u32_t)ctx->pkt_rx_buf;

	for (i = 0; i < ctx->num_rx; i++) {
		if (i == ctx->num_rx - 1)
			bd.stat |= RX_BD_WRAP;

		ethoc_write_bd(ctx, ctx->num_tx + i, &bd);
		bd.addr += ETHOC_PKT_BUF_LEN;
	}

	return 0;
}

int ethoc_init(struct device *dev)
{
	u32_t phyreset = 0U;
	u32_t clkdiv;
	struct eth_context *ctx = dev->driver_data;

	ctx->num_bd = ctx->num_tx + ctx->num_rx;
	ctx->phy = 1;
	ctx->limit = 12;
	ctx->mask = INT_MASK_ALL;

	ethoc_set_mac_address(ctx);

	ethoc_init_ring(ctx);

	ethoc_reset(ctx);

	clkdiv = MIIMODER_CLKDIV(ctx->eth_clkfreq / 2500000 + 1);
	ethoc_write(ctx, MIIMODER,
			    (ethoc_read(ctx, MIIMODER) & MIIMODER_NOPRE) |
			    clkdiv);

	/* Configure MAC addresses */
	ethoc_set_mac_address(ctx);
	LOG_DBG("write mac %x-%x-%x-%x-%x-%x\n", ctx->mac[0], ctx->mac[1],
				ctx->mac[2], ctx->mac[3], ctx->mac[4], ctx->mac[5]);

	if (ethoc_check_phy(ctx) < 0) {
		LOG_ERR("ethoc_check_phy fails\n");
		return -1;
	}

	if (ethoc_reset_phy(ctx) < 0) {
		LOG_ERR("ethoc_reset_phy fails\n");
		return -1;
	}

	k_sleep(1000);

	/* Checking whether phy reset completed successfully.*/
	if (ethoc_phy_regread(ctx, ETHOC_PHY_BCONTROL, &phyreset)) {
		LOG_ERR("ethoc read phy reset fails\n");
		return -1;
	}

	if (phyreset & (1 << 15)) {
		LOG_ERR("ethoc phy reset fails\n");
		return -1;
	}

	ethoc_advertise_caps(ctx);
	ethoc_establish_link(ctx);

	LOG_DBG("ethoc_init end\n");

	return 0;
}

/* Driver functions */

static enum ethernet_hw_caps eth_ethoc_get_capabilities(struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *get_stats(struct device *dev)
{
	struct eth_context *context = dev->driver_data;

	return &context->stats;
}
#endif

static int ethoc_update_rx_stats(struct eth_context *ctx,
				struct ethoc_bd *bd)
{
	if ((bd->stat & RX_BD_TL) ||
		(bd->stat & RX_BD_SF) ||
		(bd->stat & RX_BD_DN) ||
		(bd->stat & RX_BD_CRC) ||
		(bd->stat & RX_BD_OR) ||
		(bd->stat & RX_BD_MISS) ||
		(bd->stat & RX_BD_LC)) {
		LOG_ERR("RX error occurs 0x%x\n", bd->stat);
#ifdef CONFIG_NET_STATISTICS_ETHERNET
		ctx->stats.errors.rx++;
#endif
		return -1;
	}
	return 0;
}

static void ethoc_update_tx_stats(struct eth_context *ctx, struct ethoc_bd *bd)
{
	if ((bd->stat & TX_BD_LC) ||
		(bd->stat & TX_BD_RL) ||
		(bd->stat & TX_BD_UR) ||
		(bd->stat & TX_BD_CS) ||
		(bd->stat & TX_BD_STATS)) {
		LOG_ERR("TX error occurs 0x%x\n", bd->stat);
#ifdef CONFIG_NET_STATISTICS_ETHERNET
		ctx->stats.errors.tx++;
#endif
	}
}

static int ethoc_tx(struct eth_context *ctx)
{
	int count;
	struct ethoc_bd bd;

	for (count = 0; count < ctx->limit; ++count) {
		u32_t entry;

		if ((ctx->dty_tx == ctx->cur_tx) && (ctx->pkt_tx[ctx->dty_tx] == NULL)) {
			LOG_DBG("ethoc_tx is empty\n");
			break;
		}

		entry = ctx->dty_tx;

		LOG_DBG("ethoc_tx get %d\n", entry);

		ethoc_read_bd(ctx, entry, &bd);

		if (bd.stat & TX_BD_READY) {
			LOG_DBG("ethoc_tx no ready\n");
			break;
		}

		LOG_DBG("proc: eth_tx bd.stat 0x%x\n", bd.stat);

		ethoc_update_tx_stats (ctx, &bd);

		if (ctx->pkt_tx[ctx->dty_tx]) {
			ctx->pkt_tx[ctx->dty_tx] = NULL;
		}
		if (ctx->dty_tx == ctx->num_tx-1)
			ctx->dty_tx = 0;
		else
			ctx->dty_tx++;
	}

	return 0;
}

static int ethoc_rx(struct eth_context *ctx)
{
	int count;
	int res;
	int size;
	unsigned int entry;
	struct ethoc_bd bd;
	struct net_pkt *pkt;

	for (count = 0; count < ctx->limit; ++count) {
		entry = ctx->num_tx + ctx->cur_rx;
		LOG_DBG("ethoc_rx get %d\n", ctx->cur_rx);

		ethoc_read_bd(ctx, entry, &bd);
		if (bd.stat & RX_BD_EMPTY) {
			LOG_DBG("ethoc_rx no ready\n");
			return 0;
		}

		size = bd.stat >> 16;

		LOG_DBG("proc: eth_rx bd.stat 0x%x\n", bd.stat);

		if ((size >= 4) && (ethoc_update_rx_stats(ctx, &bd) == 0)) {
			size -= 4; /* strip the CRC */
			pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, size,
					   AF_UNSPEC, 0, K_NO_WAIT);
			if (!pkt) {
				LOG_ERR("Failed to obtain RX buffer");
				goto next;
			}

			__asm__ volatile ("fence;\n\t");

			if (net_pkt_write(pkt,
					ctx->pkt_rx_buf + ctx->cur_rx*ETHOC_PKT_BUF_LEN, size)) {
				LOG_ERR("Failed to append RX buffer to context buffer");
				net_pkt_unref(pkt);
				goto next;
			}

			res = net_recv_data(ctx->iface, pkt);
			if (res < 0) {
				LOG_ERR("Failed to enqueue frame into RX queue: %d", res);
				net_pkt_unref(pkt);
				goto next;
			}
		}

next:
		/* clear the buffer descriptor so it can be reused */
		bd.stat &= ~RX_BD_STATS;
		bd.stat |=  RX_BD_EMPTY;
		ethoc_write_bd(ctx, entry, &bd);
		if (ctx->cur_rx == ctx->num_rx-1)
			ctx->cur_rx = 0;
		else
			ctx->cur_rx++;
	}

	return 0;
}

static void eth_ethoc_txrx(struct eth_context *ctx)
{
	u32_t pending;
	u32_t mask;

	mask = ctx->mask;
	pending = ethoc_read(ctx, INT_SOURCE);
	pending &= mask;

	if (pending == 0)
		return;

	ethoc_write(ctx, INT_SOURCE, pending);

	/* We always handle the dropped packet interrupt */
	if (pending & INT_MASK_BUSY) {
		eth_stats_update_errors_rx (ctx->iface);
	}

	if (pending & INT_MASK_RX) {
		LOG_DBG ("ethoc_rx pending %x\n", pending);
		ethoc_rx (ctx);
	}

	if (pending & INT_MASK_TX) {
		LOG_DBG ("ethoc_tx pending %x\n", pending);
		ethoc_tx (ctx);
	}
}

static void eth_ethoc_txrx_irq(struct device *dev)
{
	eth_ethoc_txrx(dev->driver_data);
}

void ethoc_thread(void *a, void *b, void *c)
{
	struct eth_context *ctx = (struct eth_context *)a;
	u32_t tmp = 0;

	tmp = ethoc_read(ctx, MODER);
	tmp |= MODER_RXEN | MODER_TXEN;
	ethoc_write(ctx, MODER, tmp);

    while (1) {
        eth_ethoc_txrx(ctx);
		k_sleep(2);
    }
}

static void eth_initialize(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct eth_context *ctx = dev->driver_data;
	u32_t tmp;

	LOG_DBG("eth_initialize");

	ethoc_read_mac_address(ctx);

	/* wait auto negotiation done */
	do {
		if (ethoc_phy_regread(ctx, ETHOC_PHY_CS, &tmp)) {
			LOG_ERR("ethoc read phy reset fails\n");
			return;
		k_sleep(1);
		}
	} while (!(tmp & (1<<12)));

	LOG_DBG ("phy read ETHOC_PHY_CS is %x\n", tmp);

	/* wait link up */
	do {
		if (ethoc_phy_regread(ctx, ETHOC_PHY_BSTATUS, &tmp)) {
			LOG_ERR("ethoc read phy reset fails\n");
			return;
		k_sleep(1);
		}
	} while (!(tmp & (1<<2)));

	LOG_DBG ("phy read ETHOC_PHY_BSTATUS is %x\n", tmp);

	/* create polling task */
	if (ctx->polled_mode) {
		ctx->tid = k_thread_create(&ethoc_thread_data, ethoc_thread_stack_area,
				(size_t)ETHOC_THREAD_STACK_SIZE, ethoc_thread, ctx, NULL, NULL,
				PRIORITY, 0, K_NO_WAIT);
	}
	else {
		IRQ_CONNECT(DT_OPENCORES_ETHOC_0_IRQ_0,
                    DT_OPENCORES_ETHOC_0_IRQ_0_PRIORITY,
                    eth_ethoc_txrx_irq, DEVICE_GET(eth_ethoc_0), 0);
        irq_enable(DT_OPENCORES_ETHOC_0_IRQ_0);
		ethoc_write(ctx, INT_SOURCE, ctx->mask);
        ethoc_write(ctx, INT_MASK, INT_MASK_ALL);

		tmp = ethoc_read(ctx, MODER);
		tmp |= MODER_RXEN | MODER_TXEN;
		ethoc_write(ctx, MODER, tmp);
	}

	net_if_set_link_addr(iface, ctx->mac, sizeof(ctx->mac),
			     NET_LINK_ETHERNET);

	ctx->iface = iface;

	ethernet_init(iface);
}

static int eth_tx(struct device *dev, struct net_pkt *pkt)
{
	struct eth_context *ctx = dev->driver_data;
	struct ethoc_bd bd;
	u32_t entry;
	u32_t total_len = net_pkt_get_len(pkt);

	__ASSERT(pkt, "buf pointer is NULL");

	LOG_DBG("eth_tx enter\n");

	if (total_len > NET_ETH_MAX_FRAME_SIZE) {
		LOG_ERR("eth_tx len too big\n");
		return -1;
	}

	if ((ctx->cur_tx == ctx->dty_tx) && (NULL != ctx->pkt_tx[ctx->cur_tx])) {
		LOG_DBG("eth_tx busy\n");
		return -1;
		}

	entry = ctx->cur_tx;
	LOG_DBG("eth_tx set %d\n", entry);

	ethoc_read_bd(ctx, entry, &bd);
	if (total_len < ETHOC_ZLEN)
		bd.stat |=  TX_BD_PAD;
	else
		bd.stat &= ~TX_BD_PAD;

	if (net_pkt_read(pkt, (void *)bd.addr, total_len)) {
		LOG_ERR("eth_tx net_pkt_read fails\n");
		return -1;
	}

	__asm__ volatile ("fence;\n\t");

	bd.stat &= ~(TX_BD_STATS | TX_BD_LEN_MASK);
	bd.stat |= TX_BD_LEN(total_len);
	ethoc_write_bd(ctx, entry, &bd);

	bd.stat |= TX_BD_READY;
	ethoc_write_bd(ctx, entry, &bd);
	ctx->pkt_tx[ctx->cur_tx] = pkt;

	if (ctx->cur_tx == ctx->num_tx-1)
		ctx->cur_tx = 0;
	else
		ctx->cur_tx++;

	LOG_DBG("eth_tx leave\n");

	return 0;
}

static const struct ethernet_api api_funcs = {
	.iface_api.init = eth_initialize,

	.get_capabilities = eth_ethoc_get_capabilities,
	.send = eth_tx,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = get_stats,
#endif
};

/* Bindings to the platform */

int eth_init(struct device *dev)
{
	int ret = ethoc_init(dev);

	if (ret != 0) {
		LOG_ERR("ethoc failed to initialize");
		return -ENODEV;
	}

	return ret;
}

static struct eth_context eth_0_context = {
	.iobase = DT_OPENCORES_ETHOC_0_BASE_ADDRESS,
	.polled_mode = false,
	.eth_clkfreq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
	.num_tx = CONFIG_ETHOC_BD_TX_NUM,
	.num_rx = CONFIG_ETHOC_BD_RX_NUM,
	.num_bd = CONFIG_ETHOC_BD_TX_NUM + CONFIG_ETHOC_BD_RX_NUM,
	.mac = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
};

NET_DEVICE_INIT(eth_ethoc_0, "ethoc_0", eth_init,
			&eth_0_context, NULL, CONFIG_ETH_INIT_PRIORITY, &api_funcs,
			ETHERNET_L2, NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);
