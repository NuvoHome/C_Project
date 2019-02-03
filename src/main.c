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


#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
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
//PIR--WORKING
static uint32_t val;
// static char pirmotion()
// {      

//        return val1;
// }
//EMI--WORKING
static char read_raw_char[10];


static int emisensor( int read_raw)
{
       
    ESP_ERROR_CHECK(esp_wifi_stop());
    adc2_config_channel_atten(ADC2_GPIO14_CHANNEL, ADC_ATTEN_0db);
    esp_err_t r = adc2_get_raw(ADC2_GPIO14_CHANNEL, ADC_WIDTH_12Bit, &read_raw);
    // if (r == ESP_OK)
    // {
    //     // gcvt(read_raw, 6, read_raw_char);
    //     // printf("%d\n", read_raw);
    //     return read_raw;
    // }
    return read_raw;
    initialise_wifi();
}
//AMG--WORKING
static float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
static char pixelchar[AMG88xx_PIXEL_ARRAY_SIZE];

// static int amg()
// {

//     return pixelchar;
// }
// //TCS--WORKING
// static int tcs_array[10];
// int tcs()
// {
 
//     // printf("%d, %d", calculateColorTemperature(r, g, b), calculateLux(r, g, b));
//     return tcs;
// }

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
        .mode = I2S_MODE_MASTER | I2S_MODE_RX, // I2S_MODE_TX Only TX
        .sample_rate = 17000,
        .bits_per_sample = 32,
        // .channel_format = I2S_CHANNEL_FMT_ALL_LEFT, //2-channels
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        // .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
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
void bme680_test()
{
    bme680_values_float_t values;


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
                       bme_sensor_value[0] = values.temperature;
                       bme_sensor_value[1] = values.humidity;
                       bme_sensor_value[2] = values.pressure;
                       bme_sensor_value[3] = values.gas_resistance;
        }
        // passive waiting until 1 second is over
}

void app_main()
{
    nvs_flash_init();
    initialise_wifi();
    //I2C init
    // i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ);

    i2c_master_init(I2C_BUS);
    sensor = bme680_init_sensor (I2C_BUS, BME680_I2C_ADDRESS_2, 0);
    // ESP_LOGI(TAG, "Flash encryption %d", esp_flash_encryption_enabled());
    // esp_flash_encryption_enabled();
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);

    //init ADC1(PIR MOTION SENSOR) width and attenuation
adc1_config_width(ADC_WIDTH_BIT_12);
adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

// val = adc1_get_raw(ADC1_CHANNEL_7);
// printf("val: %d ",val);
 



    //       while (1) {
    //     uint32_t adc_reading = 0;
    //       adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

    //     //Multisampling
    //     for (int i = 0; i < NO_OF_SAMPLES; i++) {

    //             adc_reading += adc1_get_raw(ADC1_GPIO35_CHANNEL);
            
    //     }
    //     adc_reading /= NO_OF_SAMPLES;
    //     //Convert adc_reading to voltage in mV
    //     uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    //     printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }



    //LSM9DS1(accel/gyro/magnetometer)--TESTING
    // while(1){

    // // printf("X:%f Y: %f Z: %f",accelData.x, accelData.y, accelData.z);
    // // int r = 0;
    // // int f = 0;
    // // i2c_master_read_slave(I2C_BUS, &r, 1);
    // // lsm_accel_read_reg(I2C_BUS,  LSM9DS1_REGISTER_OUT_X_L_XL, &f, 1);
    // // printf("%f,", accelData.x);
    // // printf("%d",r);

    //     }

    i2s_init(); //initializes mics 
    
    if (sensor) //bme680
    {
        /** -- SENSOR CONFIGURATION PART (optional) --- */

        // Changes the oversampling rates to 4x oversampling for temperature
        // and 2x oversampling for humidity. Pressure measurement is skipped.
        // bme680_set_oversampling_rates(sensor, osr_4x, osr_none, osr_2x);

        // Change the IIR filter size for temperature and pressure to 7.
        bme680_set_filter_size(sensor, iir_size_7);

        // Change the heater profile 0 to 200 degree Celcius for 100 ms.
        bme680_set_heater_profile(sensor, 0, 200, 100);
        bme680_use_heater_profile(sensor, 0);

        // Set ambient temperature to 10 degree Celsius
        bme680_set_ambient_temperature(sensor, 10);

        /** -- TASK CREATION PART --- */

        // must be done last to avoid concurrency situations with the sensor
        // configuration part

        // Create a task that uses the sensor
        printf("Initialized Sensor");
    }
    else
    {
        printf("Could not initialize BME680 sensor\n");
    }

    //    const esp_mqtt_client_config_t mqtt_cfg = {
    //     // .host = "10.0.1.29",
    //     .uri = "mqtt://rxarkckf:gzmXEr4rAClH@m15.cloudmqtt.com:10793",
    //     // .host = "m15.cloudmqtt.com",
    //     // .port = 1883,
    //     // .username = "rxarkckf",
    //     // .password = "gzmXEr4rAClH",
  
    //     // .event_handle = mqtt_event_handler,
    //     // .user_context = (void *)your_context
    // };

          const esp_mqtt_client_config_t mqtt_cfg = {
        // .host = "10.0.1.29",
        .uri = "mqtt://10.0.1.29:1883",
        // .uri = "mqtt://rxarkckf:gzmXEr4rAClH@m15.cloudmqtt.com:10793",

        // .host = "m15.cloudmqtt.com",
        // .port = 1883,
        // .username = "rxarkckf",
        // .password = "gzmXEr4rAClH",
  
        // .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
        .transport = MQTT_TRANSPORT_OVER_WS,
    };
char temperature[60] = "{temperature:";
char gasc[60] = " gas:";
char pressurec[60] = " pressure:";
char humidityc[60] = " humidity:";
char red[40] = " r:";
char green[40] = " g:";
char blue[40] = " b:";
char clear[40] = " c:";
char luxc[40] = " l:";
char colortemp[40] = " ct:";
char mic1[200] = " mic:";
char mic2[200] = " ";
char emic[20] = " emi:";
char motion[20] = " motion:";
// char test[20] =  "";


// while(1){

// }
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    while(esp_mqtt_client_start(client)){
//MIC
int32_t sample;
size_t bytes_read;
int32_t mica[101];
for(int x = 0; x < 64; x++){
i2s_read(I2S_NUM, &sample, 4, &bytes_read, 2); 
mica[x] = sample;
// printf("%d",mica[x]);
}//%d,%d,%d,%d,%d,%d,%d,%d, ,mica[32],mica[33],mica[34],mica[35],mica[36],mica[37],mica[38],mica[39]
sprintf(mic1, "mic:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",mica[0],mica[1],mica[2],mica[3],mica[4],mica[5],mica[6],mica[7],mica[8],mica[9],mica[10],mica[11],mica[12],mica[13],mica[14],mica[15],mica[16],mica[17],mica[18],mica[19],mica[20],mica[21],mica[22],mica[23],mica[24],mica[25],mica[26],mica[27],mica[28],mica[29],mica[30],mica[31]);
sprintf(mic2, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}'",mica[32],mica[33],mica[34],mica[35],mica[36],mica[37],mica[38],mica[39],mica[40],mica[41],mica[42],mica[43],mica[44],mica[45],mica[46],mica[47],mica[48],mica[49],mica[50],mica[51],mica[52],mica[53],mica[54],mica[55],mica[56],mica[57],mica[58],mica[59],mica[60],mica[61],mica[62],mica[63]);
char * mic3 = (char *) malloc(1 + strlen(mic1) + strlen(mic2));
// strcpy(mic3,mic1);
// strcat(mic3,mic2);
// printf("%s",mic3);
esp_mqtt_client_subscribe(client, "mic", 2);
esp_mqtt_client_publish(client, "mic",mic3, 0, 0, 0);

//TEN-HERTZ
int tempa[10],gasa[10],pressurea[10],humiditya[10];
for(int x = 0; x < 10; x++){
bme680_test();
tempa[x] = bme_sensor_value[0];
gasa[x] = bme_sensor_value[3];
pressurea[x] = bme_sensor_value[1];
humiditya[x] = bme_sensor_value[2];
}
int c, r, g, b,ct,l;
int ca[10],ra[10],ga[10],ba[10],cta[10],la[10];

for(int x = 0; x <10; x++){
getRawData(&r, &g, &b, &c);
ct = calculateColorTemperature(r, g, b);
l = calculateLux(r, g, b);
ra[x] = r;
ga[x] = g;
ba[x] = b;
ca[x] = c;
la[x] = l;
cta[x] = ct;
}
sprintf(red, " r:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",ra[0],ra[1],ra[2],ra[3],ra[4],ra[5],ra[6],ra[7],ra[8],ra[9]);
sprintf(green, " g:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",ga[0],ga[1],ga[2],ga[3],ga[4],ga[5],ga[6],ga[7],ga[8],ga[9]);
sprintf(blue, " b:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",ba[0],ba[1],ba[2],ba[3],ba[4],ba[5],ba[6],ba[7],ba[8],ba[9]);
sprintf(clear, " c:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",ca[0],ca[1],ca[2],ca[3],ca[4],ca[5],ca[6],ca[7],ca[8],ca[9]);
sprintf(luxc, " l:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",la[0],la[1],la[2],la[3],la[4],la[5],la[6],la[7],la[8],la[9]);
sprintf(colortemp, " ct:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",cta[0],cta[1],cta[2],cta[3],cta[4],cta[5],cta[6],cta[7],cta[8],cta[9]);
sprintf(temperature,"{temperature:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",tempa[0],tempa[1],tempa[2],tempa[3],tempa[4],tempa[5],tempa[6],tempa[7],tempa[8],tempa[9]);
sprintf(gasc," gas:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',", gasa[0],gasa[1],gasa[2],gasa[3],gasa[4],gasa[5],gasa[6],gasa[7],gasa[8],gasa[9]);
sprintf(pressurec," pressure:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",pressurea[0],pressurea[1],pressurea[2],pressurea[3],pressurea[4],pressurea[5],pressurea[6],pressurea[7],pressurea[8],pressurea[9]);
sprintf(humidityc, " humidity:'{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}',",humiditya[0],humiditya[1],humiditya[2],humiditya[3],humiditya[4],humiditya[5],humiditya[6],humiditya[7],humiditya[8],humiditya[9]);
char * tenhertz = (char *) malloc( 1 + strlen(temperature) + strlen(gasc) + strlen(pressurec) + strlen(humidityc) + strlen(red) + strlen(green));
strcpy(tenhertz,temperature);
strcat(tenhertz,gasc);
strcat(tenhertz,pressurec);
strcat(tenhertz,humidityc);
strcat(tenhertz,red);
strcat(tenhertz,green);
esp_mqtt_client_subscribe(client, "ten-hertz", 2);
esp_mqtt_client_publish(client, "ten-hertz", tenhertz, 0, 0, 0);
//ACCEL
readAccel();
readMag();
char accel[40] = "";
char mag[40] = "";
char gyro[40] = "";
sprintf(accel, "accel:'{x: %d, y: %d, z: %d}' ,", accelData.x, accelData.y, accelData.z);
sprintf(mag, " mag:'{x: %d, y: %d, z: %d}' ,", magData.x, magData.y, magData.z);
sprintf(gyro, " gyro:'{x: %d, y: %d, z: %d}'",gyroData.x, gyroData.y, gyroData.z);
char * accelmg = (char *) malloc(1 + strlen(accel) + strlen(mag) + strlen(gyro));
strcat(accelmg,accel);
strcat(accelmg,mag);
strcat(accelmg,gyro);
esp_mqtt_client_subscribe(client, "accelerometer", 2);
esp_mqtt_client_publish(client, "accelerometer", accelmg, 0, 0, 0);


// printf("r:%d,g:%d,b:%d",r,g,b);
// float *samples;
// float s;
// samples = &s;       //pointer initialization


    //   esp_mqtt_client_publish(client, "mic", micc, 0, 0, 0);



// // gcvt(bme_sensor_value[0], 6, temp);
// // gcvt(bme_sensor_value[1], 6, pressure);
// // gcvt(bme_sensor_value[2], 6, humidity);
// // gcvt(bme_sensor_value[3], 6, gas);
// int read_raw =0;
// // emisensor(read_raw);


// sprintf(micc, " mic:'%d',",samples_data); 
// sprintf(emic, " emi:'%d',", read_raw);
// sprintf(motion, " motion:'%d',", val);



//     //Thermal
//     readPixels(pixels, 64);
    
//     char thermalc1[100] = "thermal: ";
//     char thermalc2[100] = " ";
//     // sprintf(thermalc1, "t:{'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f'",pixels[0],pixels[1],pixels[2],pixels[3],pixels[4],pixels[5],pixels[6],pixels[7],pixels[8],pixels[9],pixels[10],pixels[11],pixels[12],pixels[13],pixels[14],pixels[15],pixels[16],pixels[17],pixels[18],pixels[19],pixels[20],pixels[21],pixels[22],pixels[23],pixels[24],pixels[25],pixels[26],pixels[27],pixels[28],pixels[29],pixels[30],pixels[31]);
//     // sprintf(thermalc2, ",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f'}",pixels[32],pixels[33],pixels[34],pixels[35],pixels[36],pixels[37],pixels[38],pixels[39], pixels[40],pixels[41],pixels[42],pixels[43],pixels[44],pixels[45],pixels[46],pixels[47],pixels[48],pixels[49],pixels[50],pixels[51],pixels[52],pixels[53],pixels[54],pixels[55],pixels[56],pixels[57],pixels[58],pixels[59],pixels[60],pixels[61],pixels[62],pixels[63]);
// // sprintf(thermalc,"t:'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f'",pixels[0],pixels[1],pixels[2],pixels[3],pixels[4],pixels[5],pixels[6],pixels[7],pixels[8],pixels[9],pixels[10],pixels[11],pixels[12],pixels[13],pixels[14],pixels[15],pixels[16],pixels[17],pixels[18],pixels[19],pixels[20],pixels[21],pixels[22],pixels[23],pixels[24],pixels[25],pixels[26],pixels[27],pixels[28],pixels[29],pixels[30],pixels[31],pixels[32],pixels[33],pixels[34],pixels[35],pixels[36],pixels[37],pixels[38],pixels[39], pixels[40],pixels[41],pixels[42],pixels[43],pixels[44],pixels[45],pixels[46],pixels[47],pixels[48],pixels[49],pixels[50],pixels[51],pixels[52],pixels[53],pixels[54],pixels[55],pixels[56],pixels[57],pixels[58],pixels[59],pixels[60],pixels[61],pixels[62],pixels[63]);
// char * thermalc3 = (char *) malloc(1 + strlen(thermalc1) + strlen(thermalc2));

// char * data = (char *) malloc(1 + strlen(temperature) + strlen(gasc) + strlen(pressurec) + strlen(humidityc) + strlen(micc)+strlen(motion) + strlen(red) +strlen(green) + strlen(blue) + strlen(clear) + strlen(luxc) + strlen(colortemp) + strlen(emic) + strlen(thermalc3));
// // + strlen(red) + strlen(green)+ strlen(blue)+ strlen(clear)+ strlen(luxc)+ strlen(colortemp)+  strlen(emic)+  strlen(thermalc)
// strcpy(thermalc3,thermalc1);
// strcat(thermalc3,thermalc2);
// strcpy(data, temperature);
// strcat(data, gasc);
// strcat(data,pressurec);
// strcat(data,humidityc);
// strcat(data,red);
// strcat(data,green);
// strcat(data,blue);
// strcat(data,clear);
// strcat(data,luxc);
// strcat(data,colortemp);
// strcat(data,micc);
// strcat(data,emic);
// strcat(data,motion);
// // strcat(data,thermalc3);
// // printf("%s",thermalc);

// //     strcat(thermalc,tmp2);
// //     strcat(data, thermalc);
//                 vTaskDelay(100);
//                 // printf("%s", data);
//                 esp_mqtt_client_subscribe(client, "data", 2);
//             esp_mqtt_client_publish(client, "data", data, 0, 0, 0);

            // esp_mqtt_client_publish(client, "thermal", thermalc1,0,0,0);
            // esp_mqtt_client_publish(client, "thermal", thermalc2,0,0,0);

    
//EVERYTHING TOGETHER HAHAHAHA :)
//         while(1){
//                 // int c, r, g, b,ct,l;
//                 // getRawData(&r, &g, &b, &c);
//                 // ct = calculateColorTemperature(r, g, b);
//                 // l = calculateLux(r, g, b);
//                 // printf("r:%d, g:%d, b:%d,ct:%d,l:%d\n",r,g,b,ct,l);
//             // readPixels(pixels, 64);
//             // printf("{");
//             // for(int f = 0; f <8; f++){
//             // for(int x = 0; x < 8; x++){
//             //     printf("%f \t",pixels[x]);
//             // }
//             // printf("\n");
//             // }
//             // printf("}");





//     // emisensor();
//     // mqtt_app_start();
// // mqtt_event_handler(MQTT_EVENT_SUBSCRIBED);
//     bme680_test();
//     vTaskDelay(100);

    }
}