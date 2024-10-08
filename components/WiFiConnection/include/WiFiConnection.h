/*
 * WiFiConnection.h
 *
 *  Created on: 1 thg 4, 2022
 *      Author: A315-56
 */

#ifndef COMPONENTS_WIFICONNECTION_H_
#define COMPONENTS_WIFICONNECTION_H_


#include "esp_netif.h"
#include "esp_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif



typedef enum wifi_state{
	WIFI_CONNECT_FAILED = 0,
	WIFI_CONNECTED = 1,
}wifi_state_t;

typedef struct{
	char *ip;
	char *netmask;
	char *gateway;
} wifi_ipinfo_t;

typedef struct {
	char *name;
	int8_t rssi;
} wifi_info_t;


void WiFi_STA_Connect(char *ssid, char *pass, wifi_auth_mode_t auth);
void WiFi_STA_Disconnect(void);

wifi_state_t WiFi_GetState(void);
esp_netif_t *WiFi_STA_get_netif(void);

esp_err_t WiFi_STA_Set_IPV4(wifi_ipinfo_t *info);
esp_err_t WiFi_STA_Set_IPDHCP(void);
esp_err_t WiFi_STA_Get_IPInfo(wifi_ipinfo_t *info);
void WiFi_STA_Release_IPInfo(wifi_ipinfo_t *info);

uint8_t WiFi_STA_Scan(void);
void WiFi_STA_Scan_Get_Info(uint8_t num, wifi_info_t *info);
void WiFi_STA_Release_Info(wifi_info_t *info);
char *WiFi_STA_Scan_Get_SSID(uint8_t num);
int8_t WiFi_STA_Scan_Get_RSSI(uint8_t num);

wifi_auth_mode_t WiFi_StrToAuth(char *str);


#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_WIFICONNECTION_H_ */
