

#include "WiFiConnection.h"

#include <stdio.h>
#include "sdkconfig.h"
#include "string.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "lwip/err.h"
#include "lwip/sys.h"


#define WIFI_MAXIMUM_RETRY  		  5
#define WIFI_CONNECTED_BIT 			  BIT0
#define WIFI_FAIL_BIT      			  BIT1
#define WIFI_SCAN_BIT                 BIT2

static const char *TAG = "WIFI";

static EventGroupHandle_t WiFi_Event_Group = NULL;
static wifi_state_t WiFi_State = WIFI_CONNECT_FAILED;
static esp_netif_t *netif = NULL;
static uint8_t connect_retry_num = 0;

static EventGroupHandle_t WiFi_Scan_Event_Group = NULL;
static uint16_t max_scan_num = 20;
static wifi_ap_record_t scan_ap_info[20];
static uint16_t scan_ap_num = 0;


static const char *wifi_auth_str[] = {
	"WIFI_AUTH_OPEN",         /**< authenticate mode : open */
	"WIFI_AUTH_WEP",              /**< authenticate mode : WEP */
	"WIFI_AUTH_WPA_PSK",          /**< authenticate mode : WPA_PSK */
	"WIFI_AUTH_WPA2_PSK",         /**< authenticate mode : WPA2_PSK */
	"WIFI_AUTH_WPA_WPA2_PSK",     /**< authenticate mode : WPA_WPA2_PSK */
	"WIFI_AUTH_WPA2_ENTERPRISE",  /**< authenticate mode : WPA2_ENTERPRISE */
	"WIFI_AUTH_WPA3_PSK",         /**< authenticate mode : WPA3_PSK */
	"WIFI_AUTH_WPA2_WPA3_PSK",    /**< authenticate mode : WPA2_WPA3_PSK */
	"WIFI_AUTH_WAPI_PSK",         /**< authenticate mode : WAPI_PSK */
	"WIFI_AUTH_OWE",              /**< authenticate mode : OWE */
	"WIFI_AUTH_MAX"
};

wifi_auth_mode_t WiFi_StrToAuth(char *str){
	for(int i=0; i<11; i++){
		if(strcmp(str, wifi_auth_str[i]) == 0){
			return (wifi_auth_mode_t)i;
		}
	}
	return WIFI_AUTH_OPEN;
}

static void WiFi_Event_Handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

static void WiFi_Event_Handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGW(TAG, "WIFI_EVENT_STA_START");
    }

    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (connect_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            connect_retry_num++;
            ESP_LOGE(TAG, "Retry to connect to the access point.");
        }
        else {
            xEventGroupSetBits(WiFi_Event_Group, WIFI_FAIL_BIT);
            esp_wifi_disconnect();
            ESP_LOGE(TAG,"WiFi disconnected.");
        }
        ESP_LOGW(TAG,"WIFI_EVENT_STA_DISCONNECTED");
        WiFi_State = WIFI_CONNECT_FAILED;
    }

    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_STOP) {
        esp_wifi_disconnect();
        ESP_LOGW(TAG, "WIFI_EVENT_STA_STOP");
    }

    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
    	ESP_ERROR_CHECK(esp_wifi_scan_stop());

    	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&scan_ap_num));
    	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&max_scan_num, scan_ap_info));

        ESP_LOGW(TAG, "WIFI_EVENT_SCAN_DONE");
        xEventGroupSetBits(WiFi_Scan_Event_Group, WIFI_SCAN_BIT);
    }

    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Station IP:" IPSTR, IP2STR(&event -> ip_info.ip));
        connect_retry_num = 0;

        xEventGroupSetBits(WiFi_Event_Group, WIFI_CONNECTED_BIT);
    }
}

void WiFi_STA_Connect(char *ssid, char *pass, wifi_auth_mode_t auth){
	WiFi_Event_Group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    netif = esp_netif_create_default_wifi_sta();
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START,  &WiFi_Event_Handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_STOP,  &WiFi_Event_Handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,  &WiFi_Event_Handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,  &WiFi_Event_Handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &WiFi_Event_Handler, NULL));

	esp_wifi_set_ps(WIFI_PS_NONE);

    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.sta.ssid, (ssid)? ssid:CONFIG_APP_WIFI_SSID, strlen((ssid)? ssid:CONFIG_APP_WIFI_SSID));
	wifi_config.sta.threshold.authmode = auth;
	wifi_config.sta.channel = 1;

	memcpy(wifi_config.sta.password, (pass)? pass:CONFIG_APP_WIFI_PASSWORD, strlen((pass)? pass:CONFIG_APP_WIFI_PASSWORD));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi station initialize finished.");
    
    EventBits_t bits = xEventGroupWaitBits(WiFi_Event_Group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to ap SSID: %s password: %s", (ssid)? ssid:CONFIG_APP_WIFI_SSID, (pass)? pass:CONFIG_APP_WIFI_PASSWORD);
        WiFi_State = WIFI_CONNECTED;
    }
    else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s, password: %s", (ssid)? ssid:CONFIG_APP_WIFI_SSID, (pass)? pass:CONFIG_APP_WIFI_PASSWORD);
        WiFi_State = WIFI_CONNECT_FAILED;
    }
    else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void WiFi_STA_Disconnect(void){
	if(WiFi_State == WIFI_CONNECTED){
		ESP_ERROR_CHECK(esp_wifi_stop());

		ESP_ERROR_CHECK(esp_wifi_deinit());
		ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(netif));
		esp_netif_destroy(netif);

		ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_START, &WiFi_Event_Handler));
		ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_STOP, &WiFi_Event_Handler));
		ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &WiFi_Event_Handler));
		ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &WiFi_Event_Handler));
		ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &WiFi_Event_Handler));
		xEventGroupClearBits(WiFi_Event_Group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

		ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(netif));

		netif = NULL;
		free(WiFi_Event_Group);
		WiFi_State = WIFI_CONNECT_FAILED;
		connect_retry_num = 0;
	}
}

wifi_state_t WiFi_GetState(void){
	return WiFi_State;
}

esp_netif_t *WiFi_STA_get_netif(void){
    return netif;
}

esp_err_t WiFi_STA_Set_IPV4(wifi_ipinfo_t *info){
	esp_netif_ip_info_t ip_info = {0};
	if(netif){
		memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));
		esp_netif_dhcpc_stop(netif);
        ip_info.ip.addr = esp_ip4addr_aton((const char *)info->ip);
        ip_info.netmask.addr = esp_ip4addr_aton((const char *)info->netmask);
        ip_info.gw.addr = esp_ip4addr_aton((const char *)info->gateway);
        esp_err_t err = esp_netif_set_ip_info(netif, &ip_info);
	    if (err != ESP_OK) {
	        ESP_LOGE(TAG, "Failed to set station IP");
	        return err;
	    }
	    return ESP_OK;
	}
	return ESP_FAIL;
}

esp_err_t WiFi_STA_Set_IPDHCP(void){
	if(netif){
		esp_err_t err = esp_netif_dhcpc_start(netif);
	    if (err != ESP_OK) {
	        ESP_LOGE(TAG, "Failed to start dhcp.");
	        return err;
	    }
	    return ESP_OK;
	}
	return ESP_FAIL;
}

esp_err_t WiFi_STA_Get_IPInfo(wifi_ipinfo_t *info){
	if(WiFi_State == WIFI_CONNECTED){
		esp_netif_ip_info_t IP_info_t = {0};

		if(info->ip != NULL) free(info->ip);
		if(info->netmask != NULL) free(info->netmask);
		if(info->gateway != NULL) free(info->gateway);

		info->ip      = (char *)malloc(16*sizeof(char));
		info->netmask = (char *)malloc(16*sizeof(char));
		info->gateway = (char *)malloc(16*sizeof(char));

		esp_netif_get_ip_info(netif, &IP_info_t);
		esp_ip4addr_ntoa(&IP_info_t.ip, info->ip, 16);
		esp_ip4addr_ntoa(&IP_info_t.netmask, info->netmask, 16);
		esp_ip4addr_ntoa(&IP_info_t.gw, info->gateway, 16);

		return ESP_OK;
	}

	return ESP_FAIL;
}

void WiFi_STA_Release_IPInfo(wifi_ipinfo_t *info){
	if(info->ip != NULL) free(info->ip);
	if(info->netmask != NULL) free(info->netmask);
	if(info->gateway != NULL) free(info->gateway);
}

uint8_t WiFi_STA_Scan(void){
	scan_ap_num = 0;

	WiFi_Scan_Event_Group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,  &WiFi_Event_Handler, NULL));

	if(netif == NULL){
	    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	    ESP_ERROR_CHECK(esp_wifi_start());
	}

	ESP_ERROR_CHECK(esp_wifi_clear_ap_list());
	ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));

	EventBits_t bits = xEventGroupWaitBits(WiFi_Scan_Event_Group, WIFI_SCAN_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
	if(bits == WIFI_SCAN_BIT) {
		ESP_LOGE(TAG, "WiFi Scan access point finished.");
		ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &WiFi_Event_Handler));
		xEventGroupClearBits(WiFi_Scan_Event_Group, WIFI_SCAN_BIT);
	}
	if(netif == NULL){
		esp_err_t err = esp_wifi_stop();
		if (err == ESP_ERR_WIFI_NOT_INIT) ESP_LOGE(TAG, "%s", "ESP_ERR_WIFI_NOT_INIT");
		esp_wifi_deinit();
	}
	for(uint8_t num=0; num<scan_ap_num; num++){
		if(scan_ap_info[num].rssi == 0) scan_ap_num -= 1;
	}

	free(WiFi_Scan_Event_Group);

    return (uint8_t)scan_ap_num;
}

void WiFi_STA_Scan_Get_Info(uint8_t num, wifi_info_t *info){
	if(info->name != NULL) free(info->name);
	uint8_t len = sizeof(scan_ap_info[num].ssid);

	info->name = (char *)malloc(len * sizeof(uint8_t));
	memcpy(info->name, scan_ap_info[num].ssid, sizeof(scan_ap_info[num].ssid));

	info->rssi = scan_ap_info[num].rssi;
}

void WiFi_STA_Release_Info(wifi_info_t *info){
	if(info->name != NULL) free(info->name);
	info->rssi = 0;
}

char *WiFi_STA_Scan_Get_SSID(uint8_t Number){
	char *buffer;
	uint8_t len = sizeof(scan_ap_info[Number].ssid);
	buffer = (char *)malloc(len * sizeof(uint8_t));
	memcpy(buffer, scan_ap_info[Number].ssid, sizeof(scan_ap_info[Number].ssid));
	return buffer;
}

int8_t WiFi_STA_Scan_Get_RSSI(uint8_t Number){
	return scan_ap_info[Number].rssi;
}

























