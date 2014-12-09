/*
 * File	: user_main.c
 * This file is part of Espressif's AT+ command set program.
 * Copyright (C) 2013 - 2016, Espressif Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "at.h"
#include "at_config_store.h"
#include "user_interface.h"

extern uint8_t at_wifiMode;
static atConfig_t *config;

void ICACHE_FLASH_ATTR
init_done(void)
{
	struct ip_info pTempIp;
	at_wifiMode = wifi_get_opmode();
	if(config->wifi_set.ap_setMac == 1)
	{
		wifi_set_macaddr(SOFTAP_IF, config->wifi_set.ap_mac);
	}	
	if(config->wifi_set.ap_dhcp == 1)
	{
		wifi_softap_dhcps_stop();
	}
	if(config->wifi_set.ap_setIp == 1)
	{ //add get dhcp status
		wifi_get_ip_info(0x01, &pTempIp);
    IP4_ADDR(&pTempIp.ip, 
             config->wifi_set.ap_ip[3],
             config->wifi_set.ap_ip[2],
             config->wifi_set.ap_ip[1],
             config->wifi_set.ap_ip[0]);
    os_printf("apIp:%d.%d.%d.%d\r\n", IP2STR(&pTempIp.ip));
    wifi_set_ip_info(0x01, &pTempIp);
	}
	
	if(config->wifi_set.sta_setMac == 1)
	{
		wifi_set_macaddr(STATION_IF, config->wifi_set.sta_mac);
	}	
	if(config->wifi_set.sta_dhcp == 1)
	{
		wifi_station_dhcpc_stop();
	}
	if(config->wifi_set.ap_setIp == 1)
	{ //add get dhcp status
		wifi_get_ip_info(0x00, &pTempIp);
    IP4_ADDR(&pTempIp.ip, 
             config->wifi_set.sta_ip[3],
             config->wifi_set.sta_ip[2],
             config->wifi_set.sta_ip[1],
             config->wifi_set.sta_ip[0]);
    wifi_set_ip_info(0x00, &pTempIp);
	}
			
	os_printf("\r\nready!!!\r\n");
  uart0_sendStr("\r\n+IREADY\r\n");
}

void user_init(void)
{
  uint8_t userbin;
  uint32_t upFlag;
  at_uartType tempUart;

  config = atConfig_init();
  uart_init(config->serial.baud_rate, BIT_RATE_115200);
  
  user_date_test();
  at_init();
  
  system_init_done_cb(&init_done);
}
