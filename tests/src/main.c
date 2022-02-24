/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

volatile int vi;

void main(void)
{
  while(1){
	printk("Hello World! %s\n", CONFIG_BOARD);
   for(int i=0; i<10000; i++){
     vi++;
   }
 }
}
