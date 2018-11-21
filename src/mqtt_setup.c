#include "main.h"
#include "espmqtt/include/mqtt.h"

const char *MQTT_TAG = "MQTT_SAMPLE";

mqtt_client *gb_mqttClient = NULL;

void connected_cb(void *self, void *params)
{
    mqtt_client *client = (mqtt_client *)self;
    const char topic_subscribe[] = "bme680_values";
    mqtt_subscribe(client, topic_subscribe, 1);
    /*
    const char topic_publish[] = "sensors/"SENSOR_ID"/values";
    const char topic_subscribe[] = "sensors/"SENSOR_ID"/configuration";
    const char body[] = "{\"Voltage\":3.3}";
    ESP_LOGI("connected_cb", "topic_publish: %s   body: %s", topic_publish, body);
    mqtt_publish(client, topic_publish, body, sizeof(body)-1, 1, 0);                            // sizeof()-1 to compensate for the trailing '\0' in the string
    */
}

void disconnected_cb(void *self, void *params)
{

}

void reconnect_cb(void *self, void *params)
{

}

void subscribe_cb(void *self, void *params)
{
    ESP_LOGI(MQTT_TAG, "[APP] Subscribe ok, test publish msg");
    const char topic_publish[] = "bme680_values";
    // char body[25];
    char body = "test";
    mqtt_client *client = (mqtt_client *)self;
    mqtt_publish(client, topic_publish, body, strlen(body), 1, 0);                            // sizeof()-1 to compensate for the trailing '\0' in the string
}

void update_cb(void *self, void *params)
{
    ESP_LOGI(MQTT_TAG, "[APP] Updating");
    const char topic_publish[] = "bme680_values";
    char body = "test";
    mqtt_client *client = (mqtt_client *)self;
    mqtt_publish(client, topic_publish, body, strlen(body), 1, 0);                            // sizeof()-1 to compensate for the trailing '\0' in the string
}

void publish_cb(void *self, void *params)
{
    EventGroupHandle_t esp32_event_group = NULL;
    xEventGroupSetBits(esp32_event_group, 0x00000002);
}

void data_cb(void *self, void *params)
{
    mqtt_client *client = (mqtt_client *)self;
    mqtt_event_data_t *event_data = (mqtt_event_data_t *)params;
    char *topic = NULL, *data = NULL;

    if(event_data->data_offset == 0) {

        topic = malloc(event_data->topic_length + 1);
        memcpy(topic, event_data->topic, event_data->topic_length);
        topic[event_data->topic_length] = 0;
        ESP_LOGI(MQTT_TAG, "[APP] Publish topic: %s", topic);
    }

    data = malloc(event_data->data_length + 1);
    memcpy(data, event_data->data, event_data->data_length);
    data[event_data->data_length] = 0;
    ESP_LOGI(MQTT_TAG, "[APP] Publish data[%d/%d bytes]",
             event_data->data_length + event_data->data_offset,
             event_data->data_total_length);
    ESP_LOGI(MQTT_TAG, "Publish Data: %s", data);
    free(topic);
    free(data);
}
mqtt_settings settings = {
    .host = "m15.cloudmqtt.com",
/*
#if defined(CONFIG_MQTT_SECURITY_ON)
    .port = 8883, // encrypted
#else
    .port = 1883, // unencrypted
#endif
*/
    //.port = WEB_PORT,
    .port = 10793,
    // .client_id = "mqtt_"SENSOR_ID,
    .username = "rxarkckf",
    .password = "smNb81Ppfe7T",
    .clean_session = 0,
    .keepalive = 120,
    .lwt_topic = "",
    .lwt_msg = "",
    .lwt_qos = 0,
    .lwt_retain = 0,
    .connected_cb = connected_cb,
    .disconnected_cb = disconnected_cb,
    //.reconnect_cb = reconnect_cb,
    .subscribe_cb = subscribe_cb,
    .publish_cb = publish_cb,
    .data_cb = data_cb
};