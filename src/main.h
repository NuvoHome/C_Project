#ifndef __MAIN_H__
#define __MAIN_H__
 #include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include <driver/adc.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_adc_cal.h"
#include "bme680/bme680.h"
#include "TCS/TCS34725.h"
#include "AMG/AMG8833.h"
#include "LSM9DS1/LSM9DS1.h"
// #include "espmqtt/include/mqtt.h"
#include "esp-mqtt-master/include/mqtt_client.h"
#define WEB_SERVER                  "45.79.8.213"
#define WEB_PORT                    1883                                // should be an integer and not a string
#define SSID "Home"
#define PASSWORD "Casabl12"
#define FIRMWARE_UPG_URL "10.0.1.29:8080/firmware.bin"

//  void connected_cb(void *self, void *params);
// void disconnected_cb(void *self, void *params);
// void reconnect_cb(void *self, void *params);
// void subscribe_cb(void *self, void *params);
// void update_cb(void *self, void *params);
// void publish_cb(void *self, void *params);
// void data_cb(void *self, void *params);
// void wifi_conn_init(void);

// extern mqtt_client * gb_mqttClient;
// extern mqtt_settings settings;

#endif