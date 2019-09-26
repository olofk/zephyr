/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>

u8_t txbuf[2000];
u8_t rxbuf[2000];

void testb(){
  printk("hello finished\n");
}


void main()
{
  
  volatile uint32_t *modereg = (volatile uint32_t *)0x80003000;
  volatile uint32_t *intsrcreg = (volatile uint32_t *)0x80003004;
  volatile uint32_t *intmaskreg = (volatile uint32_t *)0x80003008;
  volatile uint32_t *mac0reg = (volatile uint32_t *)0x80003040;
  volatile uint32_t *mac1reg = (volatile uint32_t *)0x80003044;
  volatile uint32_t *txdescr0_lo = (volatile uint32_t *)0x80003400;
  volatile uint32_t *txdescr0_hi = (volatile uint32_t *)0x80003404;
  volatile uint32_t *rxdescr0_lo = (volatile uint32_t *)0x80003600;
  volatile uint32_t *rxdescr0_hi = (volatile uint32_t *)0x80003604;

  volatile uint32_t *txdescr0_memptr = (volatile uint32_t *)txbuf;
  volatile uint32_t *rxdescr0_memptr = (volatile uint32_t *)rxbuf;
  
  volatile uint32_t *simstopreg = (volatile uint32_t *)0x80001009;
  volatile uint32_t *ledreg = (volatile uint32_t *)0x80001010;

  uint32_t mac_header0 = (uint32_t *)0x00183E02;
  uint32_t mac_header1 = (uint32_t *)0xF83F0102;
  uint32_t mac_header2 = (uint32_t *)0x03040506;
  uint32_t mac_header3 = (uint32_t *)0x0800ABCD;    

  // Setup own macadress
  *mac0reg = (volatile uint32_t *)0x3E02F83F; 
  *mac1reg = (volatile uint32_t *)0x00000018;

  //Readback written macadress
  uint32_t mac0 = *mac0reg;
  uint32_t mac1 = *mac1reg;
  
  volatile uint32_t *miicmd = (volatile uint32_t *)0x8000302c;
  volatile uint32_t *miiaddr = (volatile uint32_t *)0x80003030;
  volatile uint32_t *miitxdat = (volatile uint32_t *)0x80003034;
  volatile uint32_t *miirxdat = (volatile uint32_t *)0x80003038;
  volatile uint32_t *miistatus = (volatile uint32_t *)0x8000303c;
 
  u32_t phyreg;
  
  phyreg = 0;
  *miiaddr = (phyreg << 8)|(1);
  *miicmd = 0x2;
  while (*miistatus & 0x2);
  printk("phy read %x %x\n", phyreg, *miirxdat);
  *miicmd = 0x0;

  phyreg = 1;
  *miiaddr = (phyreg << 8)|(1);
  *miicmd = 0x2;
  while (*miistatus & 0x2);
  printk("phy read %x %x\n", phyreg, *miirxdat);
  *miicmd = 0x0;

  // Enable descriptor buffer interrupt
  //*intmaskreg = (volatile uint32_t *)0x1; // TXDB irq en

  // Fill in minimum 64byte packet in the descriptor buffer
  // Ethernet2 Packet: 6byte Dest MAC, 6byte Src MAC, 2 byte Ethertype, Payload.
  
  volatile uint32_t *memptr = txdescr0_memptr;  
  *memptr = mac_header0;
  memptr++;
  *memptr = mac_header1;
  memptr++;
  *memptr = mac_header2;
  memptr++;
  *memptr = mac_header3;
  memptr++;  
  for (int i=0;i<8;i++) {
    
    *memptr = (volatile uint32_t *)0xAA55AA55;
    memptr++;
    *memptr = (volatile uint32_t *)0x66336633;
    memptr++;
  }

  // Payload data corresponding to this descriptor
  *txdescr0_hi = txdescr0_memptr; 

  // Setup the only tx buffer in wrap mode and set it ready. 
  *txdescr0_lo = 0x0040E000; // 64bytes LEN, Wrap, IRQen, BD ready


  // Payload data corresponding to this descriptor
  *rxdescr0_hi = rxdescr0_memptr; 

  // Setup the only Rx buffer in wrap mode and set it ready. 
  *rxdescr0_lo = 0x0040E000; // 64bytes LEN, Wrap, IRQen, BD ready

  // Read TX buffer status
  uint32_t tmp = *txdescr0_lo;

  // Read RX buffer status
  tmp = *rxdescr0_lo;

  // Set modereg
  tmp = *modereg;  
  *modereg = tmp | 0x4A3; // Full duplex, Loopback, promiscous Txen, Rxen
  //*modereg = tmp | 0x402; // Full duplex, Txen

  tmp = *txdescr0_lo;
  // Poll until buffer sent
  while (1)
    {
      tmp = *txdescr0_lo;
      if (tmp == 0x00406000)
	break;
    }
  
  // Buffer sent, light LED
  *ledreg = (uint32_t *)0x1;
  
  // Read RX buffer status
  tmp = *rxdescr0_lo;
  
  // Disable TXB interrupt
  //*intsrcreg = (volatile uint32_t *)0x1; // TXDB irq

  *simstopreg = (uint32_t *)0x00000000;

  testb();
}

