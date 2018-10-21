#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "bme680/bme680.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
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
#include "TCS34725.h"
#define MQTT_TOPIC "/topic/data"
#define TASK_STACK_DEPTH 2048
#define SPI_BUS       HSPI_HOST
#define SPI_SCK_GPIO 22
#define SPI_MOSI_GPIO 2
#define SPI_MISO_GPIO 23
#define SPI_CS_GPIO 15
#define SDA_PIN 21
#define SCL_PIN 22
//set i2c addresses to connect to 
// #define MAX44009_ADDRESS1 0x4A
// #define MAX44009_ADDRESS2 0x4B
#define I2C_BUS       0

#define I2C_FREQ I2C_FREQ_100K

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1
#define SAMPLE_RATE     (36000)
#define I2S_NUM         (0)
static const char *TAG = "MQTT_SAMPLE";

static bme680_sensor_t* sensor = 0;

void user_task(void *pvParameters)
{
    bme680_values_float_t values;

    TickType_t last_wakeup = xTaskGetTickCount();

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration = bme680_get_measurement_duration(sensor);

    while (1)
    {
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement (sensor))
        {
            // passive waiting until measurement results are available
            vTaskDelay (duration);

            // alternatively: busy waiting until measurement results are available
            // while (bme680_is_measuring (sensor)) ;

            // get the results and do something with them
            if (bme680_get_results_float (sensor, &values))
                printf("%.3f BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                       (double)sdk_system_get_time()*1e-3,
                       values.temperature, values.humidity,
                       values.pressure, values.gas_resistance);
        }
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
    }
}

void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 4000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
   return ESP_OK;
}
static void initialise_wifi(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "Home",
            .password = "Casabl12",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}
// static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
// {
//     esp_mqtt_client_handle_t client = event->client;
//     int msg_id;
//     // your_context_t *context = event->context;
//     switch (event->event_id) {
//         case MQTT_EVENT_CONNECTED:
//             ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
//             msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC, 0);
//             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

//         case MQTT_EVENT_DISCONNECTED:
//             ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
//             break;

//         case MQTT_EVENT_SUBSCRIBED:
//             ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
//             msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, "data", 0, 0, 0);
//             ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
//             break;
//         case MQTT_EVENT_UNSUBSCRIBED:
//             ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
//             break;
//         case MQTT_EVENT_PUBLISHED:
//             ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
//             break;
//         case MQTT_EVENT_DATA:
//             ESP_LOGI(TAG, "MQTT_EVENT_DATA");
//             printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
//             printf("DATA=%.*s\r\n", event->data_len, event->data);
//             break;
//         case MQTT_EVENT_ERROR:
//             ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
//             break;
//     }
//     return ESP_OK;
// }
// static void mqtt_app_start(void)
// {
//     const esp_mqtt_client_config_t mqtt_cfg = {
//         .uri = "mqtt://iot.eclipse.org",
//         .event_handle = mqtt_event_handler,
//         // .user_context = (void *)your_context
//     };

//     esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
//     esp_mqtt_client_start(client);
// }
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCS34725_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
void app_main()
{
    
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    nvs_flash_init();
    initialise_wifi();
    //  mqtt_app_start();
        // ESP_LOGI(TAG, "Flash encryption %d", esp_flash_encryption_enabled());
// esp_flash_encryption_enabled();
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);

    // #ifdef SPI_USED

    // spi_bus_init (SPI_BUS, SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);

    // // init the sensor connected to SPI_BUS with SPI_CS_GPIO as chip select.
    // sensor = bme680_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    
    // #else  // I2C

    // Init all I2C bus interfaces at which BME680 sensors are connected
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ);

    // init the sensor with slave address BME680_I2C_ADDRESS_2 connected to I2C_BUS.
    sensor = bme680_init_sensor (I2C_BUS, BME680_I2C_ADDRESS_2, 0);

    // #endif  // SPI_USED
    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample)
    //depend on bits_per_sample
    //using 6 buffers, we need 60-samples per buffer
    //if 2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes
    //if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .use_apll = false,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,
        .ws_io_num = 25,
        .data_out_num = 22,
        .data_in_num = -1                                                       //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    int dest, try;
    while(1){
    try = i2s_read(I2S_NUM,&dest, 32, 0, 1);
    printf(try);
    }

 if (sensor)
    {
        /** -- SENSOR CONFIGURATION PART (optional) --- */

        // Changes the oversampling rates to 4x oversampling for temperature
        // and 2x oversampling for humidity. Pressure measurement is skipped.
        bme680_set_oversampling_rates(sensor, osr_4x, osr_none, osr_2x);

        // Change the IIR filter size for temperature and pressure to 7.
        bme680_set_filter_size(sensor, iir_size_7);

        // Change the heater profile 0 to 200 degree Celcius for 100 ms.
        bme680_set_heater_profile (sensor, 0, 200, 100);
        bme680_use_heater_profile (sensor, 0);

        // Set ambient temperature to 10 degree Celsius
        bme680_set_ambient_temperature (sensor, 10);
            
        /** -- TASK CREATION PART --- */

        // must be done last to avoid concurrency situations with the sensor 
        // configuration part

        // Create a task that uses the sensor
        xTaskCreate(user_task, "user_task", TASK_STACK_DEPTH, NULL, 2, NULL);
    }
    else{
        printf("Could not initialize BME680 sensor\n");
    }
    

    while (1){
      int r, ret;
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_cmd_begin(TCS34725_ADDRESS,cmd,1000 / portTICK_RATE_MS);
    // i2c_master_write_byte(cmd, TCS34725_ENABLE << 1 | TCS34725_ENABLE_PON, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, TCS34725_ENABLE << 1 | TCS34725_ENABLE_PON | TCS34725_ENABLE_AIEN , ACK_CHECK_EN);
    i2c_master_write_byte(cmd, TCS34725_COMMAND_BIT, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
//    cmd = i2c_cmd_link_create();
//   i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (TCS34725_RDATAH << 1) | TCS34725_COMMAND_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &r , ACK_VAL);   
  ret = i2c_master_cmd_begin(TCS34725_ADDRESS, cmd, 1000 / portTICK_RATE_MS);
    i2c_master_stop(cmd);

  i2c_cmd_link_delete(cmd);
//    return ret;
printf("%d",ret);
  }
}

// /* OTA example
//    This example code is in the Public Domain (or CC0 licensed, at your option.)
//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.
// */
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include <stdio.h>
// #include "esp_system.h"
// #include "esp_wifi.h"
// #include "esp_event_loop.h"
// #include "esp_log.h"
// #include "esp_ota_ops.h"
// #include "esp_http_client.h"
// #include "esp_flash_partitions.h"
// #include "esp_partition.h"

// #include "nvs.h"
// #include "nvs_flash.h"
// #include "main.h"
// #define WIFI_SSID SSID
// #define WIFI_PASS PASSWORD
// #define SERVER_URL FIRMWARE_UPG_URL
// #define BUFFSIZE 1024
// #define HASH_LEN 32 /* SHA-256 digest length */

// static const char *TAG = "native_ota_example";
// /*an ota data write buffer ready to write to the flash*/
// static char ota_write_data[BUFFSIZE + 1] = { 0 };
// //  uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
// //  uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");


// /* FreeRTOS event group to signal when we are connected & ready to make a request */
// static EventGroupHandle_t wifi_event_group;

// /* The event group allows multiple bits for each event,
//    but we only care about one event - are we connected
//    to the AP with an IP? */
// const int CONNECTED_BIT = BIT0;

// static esp_err_t event_handler(void *ctx, system_event_t *event)
// {
//     switch (event->event_id) {
//     case SYSTEM_EVENT_STA_START:
//         esp_wifi_connect();
//         break;
//     case SYSTEM_EVENT_STA_GOT_IP:
//         xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
//         break;
//     case SYSTEM_EVENT_STA_DISCONNECTED:
//         /* This is a workaround as ESP32 WiFi libs don't currently
//            auto-reassociate. */
//         esp_wifi_connect();
//         xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
//         break;
//     default:
//         break;
//     }
//     return ESP_OK;
// }

// static void initialise_wifi(void)
// {
//     tcpip_adapter_init();
//     wifi_event_group = xEventGroupCreate();
//     ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
//     ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = SSID,
//             .password = PASSWORD,
//         },
//     };
//     ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
//     ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
//     ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
//     ESP_ERROR_CHECK( esp_wifi_start() );
// }

// static void http_cleanup(esp_http_client_handle_t client)
// {
//     esp_http_client_close(client);
//     esp_http_client_cleanup(client);
// }

// static void __attribute__((noreturn)) task_fatal_error()
// {
//     ESP_LOGE(TAG, "Exiting task due to fatal error...");
//     (void)vTaskDelete(NULL);

//     while (1) {
//         ;
//     }
// }

// void print_sha256 (const uint8_t *image_hash, const char *label)
// {
//     char hash_print[HASH_LEN * 2 + 1];
//     hash_print[HASH_LEN * 2] = 0;
//     for (int i = 0; i < HASH_LEN; ++i) {
//         sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
//     }
//     ESP_LOGI(TAG, "%s: %s", label, hash_print);
// }

// static void ota_example_task(void *pvParameter)
// {
    
//     esp_err_t err;
//     /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
//     esp_ota_handle_t update_handle = 0 ;
//     const esp_partition_t *update_partition = NULL;

//     ESP_LOGI(TAG, "Starting OTA example...");

//     const esp_partition_t *configured = esp_ota_get_boot_partition();
//     const esp_partition_t *running = esp_ota_get_running_partition();

//     if (configured != running) {
//         ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
//                  configured->address, running->address);
//         ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
//     }
//     ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
//              running->type, running->subtype, running->address);

//     /* Wait for the callback to set the CONNECTED_BIT in the
//        event group.
//     */
//     xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
//                         false, true, portMAX_DELAY);
//     ESP_LOGI(TAG, "Connect to Wifi ! Start to Connect to Server....");
    
//     esp_http_client_config_t config = {
//         .url = SERVER_URL,
//         // .cert_pem = (char *)server_cert_pem_start,
//     };
//     esp_http_client_handle_t client = esp_http_client_init(&config);
//     if (client == NULL) {
//         ESP_LOGE(TAG, "Failed to initialise HTTP connection");
//         task_fatal_error();
//     }
//     err = esp_http_client_open(client, 0);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to open HTTP connection: 1");
//         esp_http_client_cleanup(client);
//         task_fatal_error();
//     }
//     esp_http_client_fetch_headers(client);

//     update_partition = esp_ota_get_next_update_partition(NULL);
//     ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
//              update_partition->subtype, update_partition->address);
//     assert(update_partition != NULL);

//     err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "esp_ota_begin failed 3");
//         http_cleanup(client);
//         task_fatal_error();
//     }
//     ESP_LOGI(TAG, "esp_ota_begin succeeded");

//     int binary_file_length = 0;
//     /*deal with all receive packet*/
//     while (1) {
//         int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
//         if (data_read < 0) {
//             ESP_LOGE(TAG, "Error: SSL data read error");
//             http_cleanup(client);
//             task_fatal_error();
//         } else if (data_read > 0) {
//             err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
//             if (err != ESP_OK) {
//                 http_cleanup(client);
//                 task_fatal_error();
//             }
//             binary_file_length += data_read;
//             ESP_LOGD(TAG, "Written image length %d", binary_file_length);
//         } else if (data_read == 0) {
//             ESP_LOGI(TAG, "Connection closed,all data received");
//             break;
//         }
//     }
//     ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

//     if (esp_ota_end(update_handle) != ESP_OK) {
//         ESP_LOGE(TAG, "esp_ota_end failed!");
//         http_cleanup(client);
//         task_fatal_error();
//     }

//     if (esp_ota_get_running_partition() == update_partition) {
//         ESP_LOGI(TAG, "The current running firmware is same as the firmware just downloaded");
//         int i = 0;
//         ESP_LOGI(TAG, "When a new firmware is available on the server, press the reset button to download it");
//         while(1) {
//             ESP_LOGI(TAG, "Waiting for a new firmware ... %d", ++i);
//             vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//     }

//     err = esp_ota_set_boot_partition(update_partition);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "esp_ota_set_boot_partition failed 2!");
//         http_cleanup(client);
//         task_fatal_error();
//     }
//     ESP_LOGI(TAG, "Prepare to restart system!");
//     esp_restart();
//     return ;
// }


// void app_main()
// {
    
//     // Initialize NVS.
//     nvs_flash_init();
//         // OTA app partition table has a smaller NVS partition size than the non-OTA
//         // partition table. This size mismatch may cause NVS initialization to fail.
//         // If this happens, we erase NVS partition and initialize NVS again.
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         nvs_flash_init();


//     initialise_wifi();
//     xTaskCreate(&ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
// }