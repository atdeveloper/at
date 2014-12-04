/*
 * File	: at_config_store.c
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

#include <string.h>
#include "at_config_store.h"
#include "user_interface.h"

atConfig_t test;

void ICACHE_FLASH_ATTR
user_date_test(void)
{
  bzero(&test, sizeof(atConfig_t));
  
  atConfig_t show;
  char *p;
  int i;

  test.serial.baud_rate = 115200;
  test.wifi_set.sta_dhcp = 1;
  test.wifi_set.ap_dhcp = 2;
  test.start.auto_trans = 1;
  test.start.role_type = 1;
  test.start.client.link_type = 1;
  test.start.client.remote_ip[3] = 168;
	
  user_esp_platform_save_param(&test, sizeof(atConfig_t));

  user_esp_platform_load_param(&show, sizeof(atConfig_t));

  p = (char *)&show;
  for(i; i<sizeof(atConfig_t); i++)
  {
    os_printf("0x%02X ", *p++);
  }
  os_printf("\r\nbaud:%d\r\n", show.serial.baud_rate);
  os_printf("\r\nip part:%d\r\n", show.start.client.remote_ip[3]);
}
