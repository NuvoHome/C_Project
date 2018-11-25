#include "main.h"
#define MQTT_TOPIC "augustine.clement101@gmail.com/test"

#define TASK_STACK_DEPTH 2048
#define SDA_PIN 15
#define SCL_PIN 4
#define I2C_FREQ I2C_FREQ_100K
#define SAMPLE_RATE (36000)
#define I2S_NUM (0)
static const char *TAG = "MQTT_SAMPLE";
static bme680_sensor_t *sensor = 0;
const static int CONNECTED_BIT = BIT0;
EventGroupHandle_t esp32_event_group = NULL;
const static int WIFI_CONNECTED_BIT = BIT0;
const static int MQTT_PUBLISHED_BIT = BIT1;
const static int MQTT_INITIATE_PUBLISH_BIT = BIT2;
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}
//PIR--WORKING
static uint32_t val;
static int pirmotion()
{
        val = adc1_get_raw(ADC1_GPIO35_CHANNEL);
       return val;
}
//EMI--WORKING
static int read_raw;

static int emisensor()
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    adc2_config_channel_atten(ADC2_GPIO14_CHANNEL, ADC_ATTEN_0db);
    esp_err_t r = adc2_get_raw(ADC2_GPIO14_CHANNEL, ADC_WIDTH_12Bit, &read_raw);
    if (r == ESP_OK)
    {
        // printf("%d\n", read_raw);
        return read_raw;
    }
    else if (r == ESP_ERR_TIMEOUT)
    {
        printf("ADC2 used by Wi-Fi.\n");
        return 0;
    }
    else {
        return 1;
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}
//AMG--WORKING
static float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
static char pixelchar[AMG88xx_PIXEL_ARRAY_SIZE];
static int amg()
{
    readPixels(pixels, 64);
    for (int x = 0; x <= 64; x++)
    {
    gcvt(pixels[x], 6, pixelchar[x]); 

        // printf("%3f,", pixels[x]);
    }
    return pixelchar;
}
//TCS--WORKING
static int tcs_array[10];
int tcs()
{
    int c, r, g, b;
    getRawData(&r, &g, &b, &c);
    tcs_array[0] = r;
    tcs_array[1] = g;
    tcs_array[2] = b;
    tcs_array[3] = c;
    tcs_array[4] = calculateColorTemperature(r, g, b);
    tcs_array[5] = calculateLux(r, g, b);
    // printf("%d, %d", calculateColorTemperature(r, g, b), calculateLux(r, g, b));
    return tcs;
}
//WIFI--WORKING
static void initialise_wifi(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t sta_config = {
        .sta = {
            .ssid = SSID,
            .password = PASSWORD,
            .bssid_set = false}};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}
//I2C init--WORKING
void i2c_master_init(int bus)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ;
    esp_err_t res = i2c_param_config(bus, &conf);
    printf("Driver param setup : %d\n", res);
    res = i2c_driver_install(bus, I2C_MODE_MASTER, 0, 0, 0);
    printf("Driver installed   : %d\n", res);
}
//I2S init--TESTING
void i2s_init()
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX, // Only TX
        .sample_rate = 16000,
        .bits_per_sample = 32,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, //2-channels
        // .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1 //Interrupt level 1
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,
        .ws_io_num = 25,
        .data_out_num = -1, //Not used
        .data_in_num = 27
        };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
}

//BME680 Sensor--WORKING
int bme_sensor_value[4];
void bme680_test(void *pvParameters)
{
    bme680_values_float_t values;

    TickType_t last_wakeup = xTaskGetTickCount();

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration = bme680_get_measurement_duration(sensor);


        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(sensor))
        {
            // passive waiting until measurement results are available
            vTaskDelay(duration);

            // alternatively: busy waiting until measurement results are available
            // while (bme680_is_measuring (sensor)) ;

            // get the results and do something with them
            if (bme680_get_results_float(sensor, &values))
                printf("BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                       values.temperature, values.humidity,
                       values.pressure, values.gas_resistance);
                       bme_sensor_value[0] = values.temperature;
                       bme_sensor_value[1] = values.humidity;
                       bme_sensor_value[2] = values.pressure;
                       bme_sensor_value[3] = values.gas_resistance;
        }
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
}
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM9DS1_ADDRESS_MAG << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);

        //   for (int x = 0; x <= 64; x++){
      msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, pixelchar[1], 0, 0, 0);
//  }
//         for (int x = 0; x <6; x++){
//  msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, (char)tcs_array[x], 0, 0, 0);
//  }
            // msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, "Hello"/*(char)val*/, 0, 0, 0);
            // msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, (char)read_raw, 0, 0, 0);
    //         for(int x = 0; x <4; x++){
    //         msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, (char)bme_sensor_value[x], 0, 0, 0);
    // }
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
    }
    return ESP_OK;
}
static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .host = "10.0.1.29",
        // .uri = "mqtt://rxarkckf:gzmXEr4rAClH@m15.cloudmqtt.com:10793",
        // .host = "m15.cloudmqtt.com",
        .port = 1883,
        // .username = "rxarkckf",
        // .password = "gzmXEr4rAClH",
  
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

void app_main()
{
    nvs_flash_init();
    initialise_wifi();
 
    i2c_master_init(I2C_BUS);
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    // ESP_LOGI(TAG, "Flash encryption %d", esp_flash_encryption_enabled());
    // esp_flash_encryption_enabled();
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);
    //init ADC1(PIR MOTION SENSOR) width and attenuation
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_DB_11);
 

    //MQTT testing
        while(1){
    amg();
//     emisensor();
//     pirmotion();
//     tcs();
// if (sensor)
//     {
//         /** -- SENSOR CONFIGURATION PART (optional) --- */

//         // Changes the oversampling rates to 4x oversampling for temperature
//         // and 2x oversampling for humidity. Pressure measurement is skipped.
//         // bme680_set_oversampling_rates(sensor, osr_4x, osr_none, osr_2x);

//         // Change the IIR filter size for temperature and pressure to 7.
//         bme680_set_filter_size(sensor, iir_size_7);

//         // Change the heater profile 0 to 200 degree Celcius for 100 ms.
//         bme680_set_heater_profile(sensor, 0, 200, 100);
//         bme680_use_heater_profile(sensor, 0);

//         // Set ambient temperature to 10 degree Celsius
//         bme680_set_ambient_temperature(sensor, 10);

//         /** -- TASK CREATION PART --- */

//         // must be done last to avoid concurrency situations with the sensor
//         // configuration part

//         // Create a task that uses the sensor
//         xTaskCreate(bme680_test, "bme680_test", TASK_STACK_DEPTH, NULL, 2, NULL);
//     }
//     else
//     {
//         printf("Could not initialize BME680 sensor\n");
//     }
    mqtt_app_start();
    vTaskDelay(100);
    }

    //TCS--WORKING
// while(1){
//     tcs();
// }
    //LSM9DS1(accel/gyro/magnetometer)--TESTING
    // while(1){
    // readAccel();
    // // readMag();
    // // printf("X:%f Y: %f Z: %f",accelData.x, accelData.y, accelData.z);
    // // int r = 0;
    // // int f = 0;
    // // i2c_master_read_slave(I2C_BUS, &r, 1);
    // // lsm_accel_read_reg(I2C_BUS,  LSM9DS1_REGISTER_OUT_X_L_XL, &f, 1);
    // printf("%f,", accelData.x);
    // // printf("%d",r);
    // }
    //PIR--WORKING
 // // uint32_t val;
 // // pirmotion(val);
    //Microphone Sensor--WORKING
    // i2s_init();
    // int samples_value;
    // float samples_data;
    //     while(1){
    //     samples_value = i2s_pop_sample(I2S_NUM,&samples_data,2);
    //     if(samples_value != -1){
    //     printf("%f,",samples_data);
    //     }
    //     }
//     const esp_mqtt_client_config_t mqtt_cfg = {
//         // .host = "10.0.1.29",
//         // .uri = "mqtt://@m15.cloudmqtt.com:10793",
//         .uri = "mqtt://rxarkckf:gzmXEr4rAClH@m15.cloudmqtt.com:10793",

//         // .host = "m15.cloudmqtt.com",
//         // .port = 1883,
//         // .username = "rxarkckf",
//         // .password = "gzmXEr4rAClH",
  
//         .event_handle = mqtt_event_handler,
//         // .user_context = (void *)your_context
//     };

//     esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
//     while(esp_mqtt_client_start(client)){
//         if (esp_mqtt_client_subscribe(client, MQTT_TOPIC, 0) == true){
//                   for (int x = 0; x <= 64; x++){
// esp_mqtt_client_publish(client, MQTT_TOPIC, (char)pixels[x], 0, 0, 0);
//  }
//         for (int x = 0; x <6; x++){
// esp_mqtt_client_publish(client, MQTT_TOPIC, (char)tcs_array[x], 0, 0, 0);
//  }
//             esp_mqtt_client_publish(client, MQTT_TOPIC, (char)val, 0, 0, 0);
//             esp_mqtt_client_publish(client, MQTT_TOPIC, (char)read_raw, 0, 0, 0);
//             for(int x = 0; x <5; x++){
//             esp_mqtt_client_publish(client, MQTT_TOPIC, (char)bme_sensor_value[0], 0, 0, 0);
//             esp_mqtt_client_publish(client, MQTT_TOPIC, (char)bme_sensor_value[1], 0, 0, 0);
//             esp_mqtt_client_publish(client, MQTT_TOPIC, (char)bme_sensor_value[2], 0, 0, 0);
//             esp_mqtt_client_publish(client, MQTT_TOPIC, (char)bme_sensor_value[3], 0, 0, 0);
//             }
//         }
//     }
    
//EVERYTHING TOGETHER HAHAHAHA
        // xTaskCreate(mqtt_event_handler,"mqtt",TASK_STACK_DEPTH, NULL, 2, NULL);
    
    
}