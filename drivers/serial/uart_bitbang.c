#include <kernel.h>
#include <arch/cpu.h>
#include <uart.h>

#define reg_uart_data (*(volatile uint32_t*)0x40000000)

static unsigned char uart_bitbang_poll_out(struct device *dev,
					   unsigned char c)
{
  
	ARG_UNUSED(dev);

	int cout = (c|0x100) << 1;
	do {
	  reg_uart_data = cout;
	  cout >>= 1;
	  __asm__ __volatile__ ("nop");
	  __asm__ __volatile__ ("nop");
	} while (cout);
	return c;
}

static int uart_bitbang_poll_in(struct device *dev, unsigned char *c)
{
	ARG_UNUSED(dev);

	*c = reg_uart_data;
	return 0;
}

static int uart_bitbang_init(struct device *dev)
{
  /* FIXME: Set baudrate */
  ARG_UNUSED(dev);
  reg_uart_data = 1;
	return 0;
}

static const struct uart_driver_api uart_bitbang_driver_api = {
	.poll_in          = uart_bitbang_poll_in,
	.poll_out         = uart_bitbang_poll_out,
	.err_check        = NULL,
};


DEVICE_AND_API_INIT(uart_bitbang, "uart0", &uart_bitbang_init,
		    NULL, NULL,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_bitbang_driver_api);
