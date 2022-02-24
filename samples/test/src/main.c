/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <string.h>
// #include <include/sys/device_mmio.h>
// #include <sys/sys_io.h>
// #include <zephyr/types.h>

volatile int counter;
volatile uint32_t dummy;
static const char *poll_data = "This is a POLL test.\r\n";


mm_reg_t mmio;

void main(void)
{

  
  const struct device *uart_dev;
  
	uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (!device_is_ready(uart_dev)) {
		return;
	}

	while (1) {
		
  	/* Verify uart_poll_out() if printk is not working */
  	for (int i = 0; i < strlen(poll_data); i++) {
  		uart_poll_out(uart_dev, poll_data[i]);
  	}
		k_sleep(K_SECONDS(1));
   printk("Hello World! %s\n", CONFIG_ARCH);
	}
 
}
