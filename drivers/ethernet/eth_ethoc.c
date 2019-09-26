/*
 * Copyright (c) 2019
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Based on linux ethoc.c driver. */

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

#if 1
#undef LOG_ERR
#define LOG_ERR printk
#undef LOG_DBG
#define LOG_DBG printk
#define STATIC 
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

#define CONFIG_ETHOC_BD_TX_NUM 8
#define CONFIG_ETHOC_BD_RX_NUM 24

#define PRIORITY  7
#define ETHOC_THREAD_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(ethoc_thread_stack_area, ETHOC_THREAD_STACK_SIZE);
struct k_thread ethoc_thread_data;

struct ethoc_bd {
	u32_t stat;
	u32_t addr;
};

struct eth_context {
	struct net_if *iface;
	k_tid_t tid;
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

STATIC u32_t ethoc_read(struct eth_context *ctx, u32_t reg)
{
	u32_t val;
	val = *(volatile u32_t *)(ctx->iobase + reg);
	return val;
}

STATIC void ethoc_write(struct eth_context *ctx, u32_t reg, u32_t val)
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

STATIC void ethoc_set_mac_address(struct eth_context *ctx)
{
	u8_t *mac = (u8_t *) ctx->mac;
	u32_t reg1;
	u32_t reg2;

	ethoc_write(ctx, MAC_ADDR0, (mac[2] << 24) | (mac[3] << 16) |
				     (mac[4] <<  8) | (mac[5] <<  0));
	printk ("write MAC_ADDR0 %x\n", (mac[2] << 24) | (mac[3] << 16) |
				     (mac[4] <<  8) | (mac[5] <<  0));
	ethoc_write(ctx, MAC_ADDR1, (mac[0] <<  8) | (mac[1] <<  0));
	printk ("write MAC_ADDR1 %x\n", (mac[0] <<  8) | (mac[1] <<  0));

	reg1 = ethoc_read(ctx, MAC_ADDR0);
	printk ("read 1 MAC_ADDR0 %x\n", reg1);
	reg2 = ethoc_read(ctx, MAC_ADDR1);
	printk ("read 1 MAC_ADDR1 %x\n", reg2);
}

STATIC int ethoc_read_mac_address(struct eth_context *ctx)
{
	u8_t *mac = (u8_t *) ctx->mac;
	u32_t reg;

	reg = ethoc_read(ctx, MAC_ADDR0);
	printk ("read 2 MAC_ADDR0 %x\n", reg);
	mac[2] = (reg >> 24) & 0xff;
	mac[3] = (reg >> 16) & 0xff;
	mac[4] = (reg >>  8) & 0xff;
	mac[5] = (reg >>  0) & 0xff;

	reg = ethoc_read(ctx, MAC_ADDR1);
	printk ("read 2 MAC_ADDR1 %x\n", reg);
	mac[0] = (reg >>  8) & 0xff;
	mac[1] = (reg >>  0) & 0xff;

	return 0;
}

STATIC void ethoc_reset(struct eth_context *ctx)
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

	#if 0
	/* enable tx and rx */
	tmp = ethoc_read(ctx, MODER);
	tmp |= MODER_TXEN;

	tmp |= MODER_PRO;
	#endif
	#if 0
	tmp = ethoc_read(ctx, MODER);
	tmp |= MODER_LOOP | MODER_PRO;
	ethoc_write(ctx, MODER, tmp);
	#endif
}

STATIC int ethoc_check_phy(struct eth_context *ctx)
{
	u32_t phyid1, phyid2;

	if (ethoc_phy_regread(ctx, ETHOC_PHY_ID1, &phyid1)) {
		return -1;
	}

	if (ethoc_phy_regread(ctx, ETHOC_PHY_ID2, &phyid2)) {
		return -1;
	}

	printk ("read phy id %x %x\n", phyid1, phyid2);

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
	#if 0
	val |= 1 << 14;
	#endif

	if (ethoc_phy_regwrite(ctx, ETHOC_PHY_BCONTROL, val)) {
		return -1;
	}

	return 0;
}

STATIC void ethoc_read_bd(struct eth_context *ctx, int index,
		struct ethoc_bd *bd)
{
	u32_t offset = ETHOC_BD_BASE + (index * sizeof(struct ethoc_bd));
	bd->stat = ethoc_read(ctx, offset + 0);
	bd->addr = ethoc_read(ctx, offset + 4);
}

STATIC void ethoc_write_bd(struct eth_context *ctx, int index,
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
	printk ("phy read back ETHOC_PHY_ANEG_ADV %x\n", aneg_adv);
}

void ethoc_establish_link(struct eth_context *ctx)
{
	u32_t bcr = 0U;

	ethoc_phy_regread(ctx, ETHOC_PHY_BCONTROL, &bcr);
	bcr |= (1 << 12) | (1 << 9);
	ethoc_phy_regwrite(ctx, ETHOC_PHY_BCONTROL, bcr);
	ethoc_phy_regread(ctx, ETHOC_PHY_BCONTROL, &bcr);
	printk ("phy read back ETHOC_PHY_BCONTROL %x\n", bcr);
}

char __aligned(8) pkt_rx_buf[CONFIG_ETHOC_BD_RX_NUM][1600];
char __aligned(8) pkt_tx_buf[CONFIG_ETHOC_BD_TX_NUM][1600];

STATIC int ethoc_init_ring(struct eth_context *ctx)
{
	struct ethoc_bd bd;
	int i;
	struct net_pkt *pkt;

	ctx->cur_tx = 0;
	ctx->dty_tx = 0;
	ctx->cur_rx = 0;

	ctx->pkt_rx_buf = pkt_rx_buf;
	ctx->pkt_tx_buf = pkt_tx_buf;
	if (!ctx->pkt_rx_buf || !ctx->pkt_tx_buf) {
		printk ("alloc buf fails\n");
		return -1;
	}

	memset(ctx->pkt_rx_buf, 0xee, CONFIG_ETHOC_BD_RX_NUM*1600);
	memset(ctx->pkt_tx_buf, 0xee, CONFIG_ETHOC_BD_TX_NUM*1600);

	ethoc_write(ctx, TX_BD_NUM, ctx->num_tx);

	/* setup transmission buffers */

	bd.stat = TX_BD_IRQ | TX_BD_CRC;
	bd.addr = ctx->pkt_tx_buf;

	for (i = 0; i < ctx->num_tx; i++) {
		if (i == ctx->num_tx - 1)
			bd.stat |= TX_BD_WRAP;

		ethoc_write_bd(ctx, i, &bd);
		bd.addr += 1600;
		ctx->pkt_tx[i] = NULL;
	}

	bd.stat = RX_BD_EMPTY | RX_BD_IRQ;
	bd.addr = ctx->pkt_rx_buf;

	for (i = 0; i < ctx->num_rx; i++) {
		if (i == ctx->num_rx - 1)
			bd.stat |= RX_BD_WRAP;

		ethoc_write_bd(ctx, ctx->num_tx + i, &bd);
		bd.addr += 1600;
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
	ctx->limit = 40;

	LOG_DBG("ethoc_init\n");
	printk ("iobase is %x\n", ctx->iobase);
	printk ("num_bd is %x\n", ctx->num_bd);
	printk ("num_rx is %x\n", ctx->num_rx);
	printk ("num_tx is %x\n", ctx->num_tx);
	printk ("phy is %x\n", ctx->phy);

	ethoc_set_mac_address(ctx);
	LOG_DBG("write mac %x-%x-%x-%x-%x-%x\n", ctx->mac[0], ctx->mac[1],
				ctx->mac[2], ctx->mac[3], ctx->mac[4], ctx->mac[5]);

	ethoc_read_mac_address(ctx);
	LOG_DBG("read mac %x-%x-%x-%x-%x-%x\n", ctx->mac[0], ctx->mac[1],
				ctx->mac[2], ctx->mac[3], ctx->mac[4], ctx->mac[5]);

	{
	u32_t testid[] = {0,1,2,3,4,5,6,17,18,26,27,29,30,31};
	for (int i = 0; i < sizeof(testid)/sizeof(u32_t); i++)
		{
			if (ethoc_phy_regread(ctx, testid[i], &phyreset)) {
				return -1;
			}
			printk ("phy read %d %x\n", testid[i], phyreset);
		}	
	}

	ethoc_init_ring(ctx);

	ethoc_reset(ctx);

	#if 1
	clkdiv = MIIMODER_CLKDIV(ctx->eth_clkfreq / 2500000 + 1);
	ethoc_write(ctx, MIIMODER,
			    (ethoc_read(ctx, MIIMODER) & MIIMODER_NOPRE) |
			    clkdiv);
	#endif

	printk ("read MIIMODER %x\n", ethoc_read(ctx, MIIMODER));

	/* Configure MAC addresses */
	ethoc_set_mac_address(ctx);
	LOG_DBG("write mac %x-%x-%x-%x-%x-%x\n", ctx->mac[0], ctx->mac[1],
				ctx->mac[2], ctx->mac[3], ctx->mac[4], ctx->mac[5]);

	if (ethoc_check_phy(ctx) < 0) {
		LOG_ERR("ethoc_check_phy fails\n");
		return -1;
	}

	#if 1

	if (ethoc_reset_phy(ctx) < 0) {
		LOG_ERR("ethoc_reset_phy fails\n");
		return -1;
	}

	k_sleep(1000);

	/* Checking whether phy reset completed successfully.*/
	if (ethoc_phy_regread(ctx, ETHOC_PHY_BCONTROL, &phyreset)) {
		LOG_ERR("ethoc read phy reset fails\n");
		return 1;
	}

	if (phyreset & (1 << 15)) {
		LOG_ERR("ethoc phy reset fails\n");
		return 1;
	}

	ethoc_advertise_caps(ctx);
	ethoc_establish_link(ctx);

	#endif

	#if 0
	{
	u32_t val;
	while ((0 == ethoc_phy_regread(ctx, ETHOC_PHY_BSTATUS, &val))
			&& ((val & 0x4) != 0x4));
	}
	#endif

	LOG_DBG("ethoc_init end\n");

	return 0;
}

/* Driver functions */

STATIC enum ethernet_hw_caps eth_ethoc_get_capabilities(struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
STATIC struct net_stats_eth *get_stats(struct device *dev)
{
	struct eth_context *context = dev->driver_data;

	return &context->stats;
}
#endif

STATIC unsigned int ethoc_update_rx_stats(struct eth_context *ctx,
				struct ethoc_bd *bd)
{
	return 0;
}

STATIC int ethoc_tx(struct eth_context *ctx)
{
	int count;
	struct ethoc_bd bd;

	for (count = 0; count < ctx->limit; ++count) {
		u32_t entry;

		entry = ctx->dty_tx;

		LOG_DBG("ethoc_tx get %d\n", entry);

		ethoc_read_bd(ctx, entry, &bd);

		if ((bd.stat & TX_BD_READY) || (ctx->dty_tx == ctx->cur_tx)) {
			LOG_DBG("ethoc_tx no ready\n");
			return 0;
		}

		printk("proc: eth_tx bd.stat 0x%x\n", bd.stat);

		if (ctx->pkt_tx[ctx->dty_tx]) {
			net_pkt_unref(ctx->pkt_tx[ctx->dty_tx]);
			LOG_DBG("ethoc_tx unref %d\n", entry);
			ctx->pkt_tx[ctx->dty_tx] = NULL;
		}
		if (++ctx->dty_tx == ctx->num_tx)
			ctx->dty_tx = 0;
	}

	return 0;
}

STATIC int ethoc_rx(struct eth_context *ctx)
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

		if (size == 0){
			/* clear the buffer descriptor so it can be reused */
			bd.stat &= ~RX_BD_STATS;
			bd.stat |=  RX_BD_EMPTY;
			ethoc_write_bd(ctx, entry, &bd);
			if (++ctx->cur_rx == ctx->num_rx)
				ctx->cur_rx = 0;
			LOG_ERR("get zero packag\n");
			continue;
		}

		printk("proc: eth_rx bd.stat 0x%x\n", bd.stat);

		if (ethoc_update_rx_stats(ctx, &bd) == 0) {
			

			size -= 4; /* strip the CRC */
			pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, size,
					   AF_UNSPEC, 0, K_NO_WAIT);
			if (!pkt) {
				LOG_ERR("Failed to obtain RX buffer");
				return -1;
			}

			__asm__ volatile ("fence;\n\t");

			#if 1
			u32_t *p = ctx->pkt_rx_buf + ctx->cur_rx*1600;
			for (int i = 0; i < (size/4+1); i++)
				{
					u32_t x = *(p+i);
					*(p+i) = ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |                      \
					(((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24));
				}
			#endif

			if (net_pkt_write(pkt, 
					ctx->pkt_rx_buf + ctx->cur_rx*1600, size)) {
				LOG_ERR("Failed to append RX buffer to context buffer");
				net_pkt_unref(pkt);
				return -1;
			}

			__asm__ volatile ("fence;\n\t");

			res = net_recv_data(ctx->iface, pkt);
			if (res < 0) {
				LOG_ERR("Failed to enqueue frame into RX queue: %d", res);
				net_pkt_unref(pkt);
				return -1;
			}

			printk("proc: eth_rx receive 1 packets %d\n", size);
			for (int i = 0; i < size; i++)
				printk("%s%02x", (i%16==0)?"\n":" ", ctx->pkt_rx_buf[i]);
			printk("\n");
		}

		/* clear the buffer descriptor so it can be reused */
		bd.stat &= ~RX_BD_STATS;
		bd.stat |=  RX_BD_EMPTY;
		ethoc_write_bd(ctx, entry, &bd);
		if (++ctx->cur_rx == ctx->num_rx)
			ctx->cur_rx = 0;
	}

	return 0;
}

STATIC void eth_ethoc_txrx(struct eth_context *ctx)
{
	u32_t pending;
	u32_t mask;

	/* Figure out what triggered the interrupt...
	 * The tricky bit here is that the interrupt source bits get
	 * set in INT_SOURCE for an event regardless of whether that
	 * event is masked or not.  Thus, in order to figure out what
	 * triggered the interrupt, we need to remove the sources
	 * for all events that are currently masked.  This behaviour
	 * is not particularly well documented but reasonable...
	 */
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

void ethoc_thread(void *a, void *b, void *c)
{
	struct eth_context *ctx = (struct eth_context *)a;
	ctx->mask = INT_MASK_ALL;
	u32_t i = 0;
	u32_t tmp = 0;

	u32_t test = 0x12345678;
	printk ("test is %x %x %x %x\n", ((u8_t *)&test)[0], ((u8_t *)&test)[1], 
									((u8_t *)&test)[2], ((u8_t *)&test)[3]);

	/* wait auto negotiation done */
	do {
		if (ethoc_phy_regread(ctx, ETHOC_PHY_CS, &tmp)) {
			LOG_ERR("ethoc read phy reset fails\n");
			return 1;
		k_sleep(1);
		}
	} while (!(tmp & (1<<12)));

	LOG_DBG ("phy read ETHOC_PHY_CS is %x\n", tmp);

	/* wait link up */
	do {
		if (ethoc_phy_regread(ctx, ETHOC_PHY_BSTATUS, &tmp)) {
			LOG_ERR("ethoc read phy reset fails\n");
			return 1;
		k_sleep(1);
		}
	} while (!(tmp & (1<<2)));

	LOG_DBG ("phy read ETHOC_PHY_BSTATUS is %x\n", tmp);

	tmp = ethoc_read(ctx, MODER);
	tmp |= MODER_RXEN | MODER_TXEN;
	ethoc_write(ctx, MODER, tmp);

    while (1) {
        eth_ethoc_txrx(ctx);
		k_sleep(1);
		#if 1
		{
		if (i++ % 5000 == 0)
		{	 
		u32_t phyreset;
		u32_t testid[] = {0,1,2,3,4,5,6,17,18,26,27,29,30,31};
		printk ("phy read ");
		for (int i = 0; i < sizeof(testid)/sizeof(u32_t); i++)
			{
				if (ethoc_phy_regread(ctx, testid[i], &phyreset)) {
					return -1;
				}
				printk ("[%d] %04x ", testid[i], phyreset);
			}	
		
		printk ("\n");
		}
		}
		#endif
    }
}

STATIC void eth_initialize(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct eth_context *ctx = dev->driver_data;

	LOG_DBG("eth_initialize");

	ethoc_read_mac_address(ctx);

	LOG_DBG("read back mac %x-%x-%x-%x-%x-%x\n", ctx->mac[0], ctx->mac[1],
				ctx->mac[2], ctx->mac[3], ctx->mac[4], ctx->mac[5]);

	/* create polling task */
	ctx->tid = k_thread_create(&ethoc_thread_data, ethoc_thread_stack_area,
			(size_t)ETHOC_THREAD_STACK_SIZE, ethoc_thread, ctx, NULL, NULL,
			PRIORITY, 0, K_NO_WAIT);

	net_if_set_link_addr(iface, ctx->mac, sizeof(ctx->mac),
			     NET_LINK_ETHERNET);

	ctx->iface = iface;

	ethernet_init(iface);
}

STATIC int eth_tx(struct device *dev, struct net_pkt *pkt)
{
	struct eth_context *ctx = dev->driver_data;
	struct ethoc_bd bd;
	u32_t entry;
	u32_t total_len = net_pkt_get_len(pkt);

	__ASSERT(pkt, "buf pointer is NULL");
	__ASSERT(pkt->frags, "Frame data missing");

	LOG_DBG("eth_tx enter\n");
 
	if (total_len > NET_ETH_MAX_FRAME_SIZE) {
		LOG_DBG("eth_tx len too big\n");
		eth_stats_update_errors_tx(ctx->iface);
		return -1;
	}

	if ((ctx->cur_tx == ctx->dty_tx) && (NULL != ctx->pkt_tx[ctx->cur_tx])) {
		LOG_DBG("eth_tx busy\n");
		return -EBUSY;
		}

	entry = ctx->cur_tx;
	LOG_DBG("eth_tx set %d\n", entry);

	ethoc_read_bd(ctx, entry, &bd);
	if (total_len < ETHOC_ZLEN)
		bd.stat |=  TX_BD_PAD;
	else
		bd.stat &= ~TX_BD_PAD;

	if (net_pkt_read(pkt, (void *)bd.addr, total_len)) {
		return -1;
	}
	ctx->pkt_tx[ctx->cur_tx] = pkt;
	net_pkt_ref(pkt);

	#if 1
	u32_t *p = bd.addr;
	for (int i = 0; i < (total_len/4+1); i++)
		{
			u32_t x = *(p+i);
			*(p+i) = ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |                      \
      		(((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24));
		}	 
	#endif

	__asm__ volatile ("fence;\n\t");

	bd.stat &= ~(TX_BD_STATS | TX_BD_LEN_MASK);
	bd.stat |= TX_BD_LEN(total_len);
	ethoc_write_bd(ctx, entry, &bd);

	bd.stat |= TX_BD_READY;
	ethoc_write_bd(ctx, entry, &bd);
	printk("proc: eth_tx send 1 packets %d\n", total_len);
	for (int i = 0; i < total_len; i++)
		printk("%s%02x", (i%16==0)?"\n":" ", ((u8_t *)bd.addr)[i]);
	printk("\n");
	if (++ctx->cur_tx == ctx->num_tx)
		ctx->cur_tx = 0;

	LOG_DBG("eth_tx leave\n");

	return 0;
}

STATIC const struct ethernet_api api_funcs = {
	.iface_api.init = eth_initialize,

	.get_capabilities = eth_ethoc_get_capabilities,
	.send = eth_tx,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = get_stats,
#endif
};

/* Bindings to the platform */

static struct device DEVICE_NAME_GET(eth_ethoc_0);

int eth_init(struct device *dev)
{
	int ret = ethoc_init(dev);

	if (ret != 0) {
		LOG_ERR("ethoc failed to initialize");
		return -ENODEV;
	}

	return ret;
}

STATIC struct eth_context eth_0_context = {
	.iobase = 0x80003000,
	.eth_clkfreq = 25000000,
	.num_tx = CONFIG_ETHOC_BD_TX_NUM,
	.num_rx = CONFIG_ETHOC_BD_RX_NUM,
	.num_bd = CONFIG_ETHOC_BD_TX_NUM + CONFIG_ETHOC_BD_RX_NUM,
	.mac = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
};

NET_DEVICE_INIT(eth_ethoc_0, "ethoc_0", eth_init,
			&eth_0_context, NULL, CONFIG_ETH_INIT_PRIORITY, &api_funcs,
			ETHERNET_L2, NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);
