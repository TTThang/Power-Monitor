#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ina3221.h>
#include <string.h>

#include "freertos/projdefs.h"
#include "mqtt.h"

const char *TAG = "POWER-MONITOR";

#define I2C_PORT I2C_NUM_0
#define WARNING_CHANNEL 1
#define WARNING_CURRENT (40.0)

//#define STRUCT_SETTING 0
#if defined EXAMPLE_MEASURING_MODE_TRIGGER
#define MODE false  // true : continuous  measurements // false : trigger measurements
#else
#define MODE true
#endif

#define VOLTAGE_MAX 4.2
#define VOLTAGE_MIN 3.0

typedef struct {
    float bus_voltage;
    float current;
    float power;
    double total_mAh;
    double total_mWh;
    float battery_remain;
} INA_Data_t;


typedef struct {
    INA_Data_t channel_1;
    INA_Data_t channel_2;
    INA_Data_t channel_3;
    char timestamp[64];
} INA_Data_Snapshot_t;

INA_Data_Snapshot_t ina_data;

char *power_data;
int data_length;

extern esp_mqtt_client_handle_t mqtt_client;

float battery_capacitor_remain_cal(float batt_vol) {
    if (batt_vol >= VOLTAGE_MAX) {
        return 100.0;
    } else if (batt_vol <= VOLTAGE_MIN) {
        return 0.0;
    } else {
        return (batt_vol - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100.0;
    }
}

void task_push_data_to_mqtt(void* arg){
	while(1){
		esp_mqtt_client_publish(mqtt_client, "smart-farm/hub/sensor/power-monitor", power_data, data_length, 0, 1);
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

void update_timestamp(INA_Data_Snapshot_t* snapshot) {
    time_t now;
    time(&now);

    struct tm *timeinfo = gmtime(&now);

    timeinfo->tm_hour += 7;

    if (timeinfo->tm_hour >= 24) {
        timeinfo->tm_hour -= 24;
        timeinfo->tm_mday += 1;
        mktime(timeinfo);
    }

    strftime(snapshot->timestamp, sizeof(snapshot->timestamp), "%Y-%m-%dT%H:%M:%SZ", timeinfo);
}


void update_ina_data(ina3221_t* dev,INA_Data_Snapshot_t* snapshot) {
	float bus_voltage;
    float shunt_voltage;
    float shunt_current;
    
	ina3221_get_bus_voltage(dev, 0, &bus_voltage);
    ina3221_get_shunt_value(dev, 0, &shunt_voltage, &shunt_current);
    
    ina_data.channel_1.bus_voltage = bus_voltage + (shunt_current/1000)*(-0.1);
    ina_data.channel_1.current = shunt_current;
    ina_data.channel_1.total_mAh += (shunt_current/36000);
    ina_data.channel_1.power = bus_voltage*shunt_current;
    ina_data.channel_1.total_mWh += (ina_data.channel_1.power/36000);
    
    ina_data.channel_1.battery_remain = battery_capacitor_remain_cal(bus_voltage);

	ina3221_get_bus_voltage(dev, 1, &bus_voltage);
    ina3221_get_shunt_value(dev, 1, &shunt_voltage, &shunt_current);
    
    ina_data.channel_2.bus_voltage = bus_voltage + (shunt_current/1000)*(-0.1);
    ina_data.channel_2.current = shunt_current;
    ina_data.channel_2.total_mAh += (shunt_current/36000);
    ina_data.channel_2.power = bus_voltage*shunt_current;
    ina_data.channel_2.total_mWh += (ina_data.channel_2.power/36000);

    ina3221_get_bus_voltage(dev, 2, &bus_voltage);
    ina3221_get_shunt_value(dev, 2, &shunt_voltage, &shunt_current);
    
    ina_data.channel_3.bus_voltage = bus_voltage + (shunt_current/1000)*(-0.1);
    ina_data.channel_3.current = -shunt_current;
    ina_data.channel_3.total_mAh += (shunt_current/36000);
    ina_data.channel_3.power = bus_voltage*shunt_current;
    ina_data.channel_3.total_mWh += (ina_data.channel_3.power/36000);

    update_timestamp(snapshot);
}

void task_update_data(void *arg) {
	while(1){
		if(power_data != NULL){
			free(power_data);
		}
	    data_length = asprintf(&power_data,
		    "{\n"
		    "\t\"channels\": [\n"
		    "\t\t{\"channel\": 1, \"bus_voltage\": %.2f V, \"current\": %.2f mA, \"power\": %.2f mW, \"capacitor\": %.5f mAh, \"consumption\": %.5f mWh, \"battery\": %.2f %%},\n"
		    "\t\t{\"channel\": 2, \"bus_voltage\": %.2f V, \"current\": %.2f mA, \"power\": %.2f mW, \"capacitor\": %.5f mAh, \"consumption\": %.5f mWh},\n"
		    "\t\t{\"channel\": 3, \"bus_voltage\": %.2f V, \"current\": %.2f mA, \"power\": %.2f mW, \"capacitor\": %.5f mAh, \"consumption\": %.5f mWh}\n"
		    "\t],\n"
		    "\t\"timestamp\": \"%s\"\n"
		    "}",
		    ina_data.channel_1.bus_voltage, ina_data.channel_1.current, ina_data.channel_1.power,
		    ina_data.channel_1.total_mAh, ina_data.channel_1.total_mWh, ina_data.channel_1.battery_remain,
		    ina_data.channel_2.bus_voltage, ina_data.channel_2.current, ina_data.channel_2.power,
		    ina_data.channel_2.total_mAh, ina_data.channel_2.total_mWh,
		    ina_data.channel_3.bus_voltage, ina_data.channel_3.current, ina_data.channel_3.power,
		    ina_data.channel_3.total_mAh, ina_data.channel_3.total_mWh,
		    ina_data.timestamp);

	
	    ESP_LOGI("INA_DATA_JSON", "Updated JSON:\n%s", power_data);
	    vTaskDelay(pdMS_TO_TICKS(200));
	}
}

void task(void *pvParameters)
{
    ina3221_t dev = {
            .shunt = {
                100,
                100,
                100
            },
            .config.config_register = INA3221_DEFAULT_CONFIG,
            .mask.mask_register = INA3221_DEFAULT_MASK
    };
    memset(&dev.i2c_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(ina3221_init_desc(&dev, 0x40, I2C_PORT, 48, 47));

#ifndef STRUCT_SETTING
    ESP_ERROR_CHECK(ina3221_set_options(&dev, MODE, true, true)); 
    ESP_ERROR_CHECK(ina3221_enable_channel(&dev, true, true, true));
    ESP_ERROR_CHECK(ina3221_set_average(&dev, INA3221_AVG_1024));
    ESP_ERROR_CHECK(ina3221_set_bus_conversion_time(&dev, INA3221_CT_8244));
    ESP_ERROR_CHECK(ina3221_set_shunt_conversion_time(&dev, INA3221_CT_8244));
#else
    dev.config.mode = MODE;
    dev.config.esht = true;
    dev.config.ebus = true;
    dev.config.ch1 = true;
    dev.config.ch2 = true;
    dev.config.ch3 = true;
    dev.config.avg = INA3221_AVG_1024;
    dev.config.vbus = INA3221_CT_8244;
    dev.config.vsht = INA3221_CT_8244;
    ESP_ERROR_CHECK(ina3221_sync(&dev));
#endif

    ESP_ERROR_CHECK(ina3221_set_warning_alert(&dev, WARNING_CHANNEL - 1, WARNING_CURRENT)); // Set overcurrent security flag

    /*uint32_t measure_number = 0;
    bool warning = false;
    float bus_voltage;
    float shunt_voltage;
    float shunt_current;*/

    while (1)
    {
       //  measure_number++;

/*#if CONFIG_EXAMPLE_MEASURING_MODE_TRIGGER
        ESP_ERROR_CHECK(ina3221_trigger(&dev)); // Start a measure
        printf("trig done, wait: ");
        do
        {
            printf("X");

            ESP_ERROR_CHECK(ina3221_get_status(&dev)); // get mask

            if (dev.mask.wf & (1 << (3 - WARNING_CHANNEL)))
                warning = true;

            vTaskDelay(pdMS_TO_TICKS(20));

        } while (!(dev.mask.cvrf)); // check if measure done
#else
        ESP_ERROR_CHECK(ina3221_get_status(&dev)); // get mask

        if (dev.mask.wf & (1 << (3 - WARNING_CHANNEL)))
            warning = true;
#endif*/
        /*for (uint8_t i = 0; i < INA3221_BUS_NUMBER; i++)
        {
            // Get voltage in volts
            ESP_ERROR_CHECK(ina3221_get_bus_voltage(&dev, i, &bus_voltage));
            // Get voltage in millivolts and current in milliamperes
            ESP_ERROR_CHECK(ina3221_get_shunt_value(&dev, i, &shunt_voltage, &shunt_current));

            printf("\nC%u:Measure number %" PRIu32 "\n", i + 1, measure_number);
            if (warning && (i + 1) == WARNING_CHANNEL)
                printf("C%u:Warning Current > %.2f mA !!\n", i + 1, WARNING_CURRENT);
            printf("C%u:Bus voltage: %.02f V\n", i + 1, bus_voltage);
            printf("C%u:Shunt voltage: %.02f mV\n", i + 1, shunt_voltage);
            printf("C%u:Shunt current: %.02f mA\n\n", i + 1, shunt_current);
        }*/
        
        update_ina_data(&dev, &ina_data);
       // warning = false ;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
	memset(&ina_data, 0, sizeof(INA_Data_Snapshot_t));
	init_mqtt();
    xTaskCreate(task, "ina3221_test", configMINIMAL_STACK_SIZE * 8, NULL, 20, NULL);
    xTaskCreate(task_push_data_to_mqtt, "task_push_data_to_mqtt", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(task_update_data, "task_update_data", configMINIMAL_STACK_SIZE * 8, NULL, 10, NULL);
}
