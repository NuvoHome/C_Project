#include "unity.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "bme680/bme680.h"
#include "i2c.h"
#include "i2s.h"
#include <math.h>
// #include "mqtt_client.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <lwmqtt/unix.h>
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#ifdef UNIT_TEST
void app_main()
{
    UNITY_BEGIN();
    
    UNITY_END();

}
#endif