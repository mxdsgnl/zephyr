/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
// #include <sys/printk.h>
#include <drivers/uart.h>
#include <string.h>
#include <devicetree.h>
// // #include <include/sys/device_mmio.h>
// #include <sys/sys_io.h>
// #include <zephyr/types.h>

// volatile int counter;
// volatile uint32_t dummy;
static const char *poll_data = "hello";

char rxbuf[100];
volatile int count;

void main(void)
{

  char ch;
  const struct device *dev;
  static int idx = 0;

	dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
	if (dev == NULL) {
		return;
	}

  
  while(1){
    
    //test uart , if recv a enter then , respond with the buf + hello
    if(uart_poll_in(dev, &ch)==0){
      
      rxbuf[idx%100] = ch;
      
      if (idx < 100)
        idx++;
      else
        idx = 0;
      
      
      if ((ch== 0x0d)||(ch==0x0a))  {
        for (int i=0; i<(idx-1); i++){
          uart_poll_out(dev, rxbuf[i]);
        }
        idx = 0;
      
        for (int i = 0; i < strlen(poll_data); i++) {
          uart_poll_out(dev, poll_data[i]);
        }
      
        for (int i = 0; i<8000; i++){  //roughly 2 ms with 20MHz clk to bp core
          count ++;
        }
      }
      
    }
    
  }

 
}
