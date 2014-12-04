#ifndef __AT_CONFIG_STORE_H
#define __AT_CONFIG_STORE_H

//#include <stdint.h>
#include "c_types.h"

typedef struct 
{
	int32_t baud_rate;
	uint8_t byte_size;
	uint8_t stop_bits;
	uint8_t parity;
	uint8_t flow_control;
} atSerial_t;

typedef struct 
{
	uint8_t ap_mac[6];
	uint8_t ap_dhcp;
	uint8_t ap_ip[4];
	uint8_t sta_mac[6];
	uint8_t sta_dhcp;
	uint8_t sta_ip[4];
	uint8_t reserve[2];
// ssid;
// passwold;
} atWifi_t;

typedef struct
{
	uint8_t link_type;
	uint8_t remote_type;
	union
	{
		char remote_domain[64];
		uint8_t remote_ip[4];
	} remote_target;
	int32_t remote_port;
	int32_t local_port;
	uint8_t reserve[2];
} atClient_t;

typedef struct 
{
	int32_t local_port;
	uint8_t reserve[4];
} atServer_t;

typedef struct 
{
	uint8_t auto_trans;
	uint8_t role_type;
	union
	{
		atClient_t client;
		atServer_t server;
	} link_role;
	uint8_t reserve[2];
} atStart_t;

typedef struct 
{
	atSerial_t serial;
	atWifi_t wifi_set;
	atStart_t start;
} atConfig_t;

#endif //__AT_CONFIG_STORE_H
