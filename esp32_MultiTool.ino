/************ includes  *************/
#include <WiFi.h> 
#include <DHT.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include "user_config.h"

/*** BlueTooth***/
#include "SimpleBLE.h" 
#include "bt.h"
#include "bt_trace.h"
#include "bt_types.h"
#include "btm_api.h"
#include "bta_api.h"
#include "bta_gatt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"

/* OverTheAir */
int OTAport = 8266;


/* Pins */
#define LUXPIN 34

Adafruit_BME280 bme;
SimpleBLE ble;

/* sensor Def */
float luxValue;
int LUX;
float calcLux;
float diffLux = 10;

float diffTemp = 0.2;
float tempValue;

float diffHum = 1;
float humValue;

float diffPres = 100;
float presValue;


WiFiClient espClient;
PubSubClient client(espClient);
char message_buff[100];
const int BUFFER_SIZE = 300;
/* bluetooth parameter*/

static esp_ble_scan_params_t ble_scan_params = {
  .scan_type              = BLE_SCAN_TYPE_ACTIVE,
  .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
  .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
  .scan_interval          = 0x50,
  .scan_window            = 0x30,
};

String macAddr;
int rssi;
#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

struct gattc_profile_inst {
  esp_gattc_cb_t gattc_cb;
  uint16_t gattc_if;
  uint16_t app_id;
  uint16_t conn_id;
  esp_bd_addr_t remote_bda;
};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_gatt_srvc_id_t alert_service_id = {
  .id = {
    .uuid = {
      .len = ESP_UUID_LEN_16,
      .uuid = {.uuid16 = 0x1811,},
    },
    .inst_id = 0,
  },
  .is_primary = true,
};

static esp_gatt_id_t notify_descr_id = {
  .uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = GATT_UUID_CHAR_CLIENT_CONFIG,},
  },
  .inst_id = 0,
};

/* Start Setup */

void setup(){

  Serial.begin(115200);

  pinMode(LUXPIN, INPUT);

  Serial.begin(115200);
  delay(10);

  ArduinoOTA.setPort(OTAport);

  ArduinoOTA.setHostname(SENSORNAME);

  

  Serial.println("Starting Node named " + String(SENSORNAME));

  setup_wifi();
  btStart();
  
  esp_bluedroid_init();
  esp_bluedroid_enable();
 
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 Sensor, check wiring!"); 
    while (1);
  }

  client.setServer(mqtt_server, mqtt_port);
 

  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total){
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error ==  OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IPess: ");
  Serial.println(WiFi.localIP());
  reconnect();

  ble.begin("ESP32 Sensor1");
  ble_client_appRegister();

}

/* Wifi */
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/* BLE */



static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
  [PROFILE_A_APP_ID] = {
    .gattc_cb = gattc_profile_a_event_handler,
    .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
  },
  [PROFILE_B_APP_ID] = {
    .gattc_cb = gattc_profile_b_event_handler,
    .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
  },
};

static void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
  uint16_t conn_id = 0;
  esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

  switch (event) {
    case ESP_GATTC_REG_EVT:
      Serial.printf("REG_EVT\n");
      esp_ble_gap_set_scan_params(&ble_scan_params);
      break;
    case ESP_GATTC_OPEN_EVT:
      conn_id = p_data->open.conn_id;

      memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
      Serial.printf("ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d\n", conn_id, gattc_if, p_data->open.status, p_data->open.mtu);

      Serial.printf("REMOTE BDA  %02x:%02x:%02x:%02x:%02x:%02x\n",
                    gl_profile_tab[PROFILE_A_APP_ID].remote_bda[0], gl_profile_tab[PROFILE_A_APP_ID].remote_bda[1],
                    gl_profile_tab[PROFILE_A_APP_ID].remote_bda[2], gl_profile_tab[PROFILE_A_APP_ID].remote_bda[3],
                    gl_profile_tab[PROFILE_A_APP_ID].remote_bda[4], gl_profile_tab[PROFILE_A_APP_ID].remote_bda[5]
                   );

      esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
      break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        esp_gatt_srvc_id_t *srvc_id = &p_data->search_res.srvc_id;
        conn_id = p_data->search_res.conn_id;
        Serial.printf("SEARCH RES: conn_id = %x\n", conn_id);
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16) {
          Serial.printf("UUID16: %x\n", srvc_id->id.uuid.uuid.uuid16);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_32) {
          Serial.printf("UUID32: %x\n", srvc_id->id.uuid.uuid.uuid32);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) {
          Serial.printf("UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", srvc_id->id.uuid.uuid.uuid128[0],
                        srvc_id->id.uuid.uuid.uuid128[1], srvc_id->id.uuid.uuid.uuid128[2], srvc_id->id.uuid.uuid.uuid128[3],
                        srvc_id->id.uuid.uuid.uuid128[4], srvc_id->id.uuid.uuid.uuid128[5], srvc_id->id.uuid.uuid.uuid128[6],
                        srvc_id->id.uuid.uuid.uuid128[7], srvc_id->id.uuid.uuid.uuid128[8], srvc_id->id.uuid.uuid.uuid128[9],
                        srvc_id->id.uuid.uuid.uuid128[10], srvc_id->id.uuid.uuid.uuid128[11], srvc_id->id.uuid.uuid.uuid128[12],
                        srvc_id->id.uuid.uuid.uuid128[13], srvc_id->id.uuid.uuid.uuid128[14], srvc_id->id.uuid.uuid.uuid128[15]);
        } else {
          Serial.printf("ERROR: UNKNOWN LEN %d\n", srvc_id->id.uuid.len);
        }
        break;
      }
    case ESP_GATTC_SEARCH_CMPL_EVT:
      conn_id = p_data->search_cmpl.conn_id;
      Serial.printf("SEARCH_CMPL: conn_id = %x, status %d\n", conn_id, p_data->search_cmpl.status);
      esp_ble_gattc_get_characteristic(gattc_if, conn_id, &alert_service_id, NULL);
      break;
    case ESP_GATTC_GET_CHAR_EVT:
      if (p_data->get_char.status != ESP_GATT_OK) {
        break;
      }
      Serial.printf("GET CHAR: conn_id = %x, status %d\n", p_data->get_char.conn_id, p_data->get_char.status);
      Serial.printf("GET CHAR: srvc_id = %04x, char_id = %04x\n", p_data->get_char.srvc_id.id.uuid.uuid.uuid16, p_data->get_char.char_id.uuid.uuid.uuid16);

      if (p_data->get_char.char_id.uuid.uuid.uuid16 == 0x2a46) {
        Serial.printf("register notify\n");
        esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, &alert_service_id, &p_data->get_char.char_id);
      }

      esp_ble_gattc_get_characteristic(gattc_if, conn_id, &alert_service_id, &p_data->get_char.char_id);
      break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        uint16_t notify_en = 1;
        Serial.printf("REG FOR NOTIFY: status %d\n", p_data->reg_for_notify.status);
        Serial.printf("REG FOR_NOTIFY: srvc_id = %04x, char_id = %04x\n", p_data->reg_for_notify.srvc_id.id.uuid.uuid.uuid16, p_data->reg_for_notify.char_id.uuid.uuid.uuid16);

        esp_ble_gattc_write_char_descr(
          gattc_if,
          conn_id,
          &alert_service_id,
          &p_data->reg_for_notify.char_id,
          &notify_descr_id,
          sizeof(notify_en),
          (uint8_t *)&notify_en,
          ESP_GATT_WRITE_TYPE_RSP,
          ESP_GATT_AUTH_REQ_NONE);
        break;
      }
    case ESP_GATTC_NOTIFY_EVT:
      Serial.printf("NOTIFY: len %d, value %08x\n", p_data->notify.value_len, *(uint32_t *)p_data->notify.value);
      break;
    case ESP_GATTC_WRITE_DESCR_EVT:
      Serial.printf("WRITE: status %d\n", p_data->write.status);
      break;
    default:
      break;
  }
}

static void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
  uint16_t conn_id = 0;
  esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

  switch (event) {
    case ESP_GATTC_REG_EVT:
      Serial.printf("REG_EVT\n");
      break;
    case ESP_GATTC_OPEN_EVT:
      conn_id = p_data->open.conn_id;

      memcpy(gl_profile_tab[PROFILE_B_APP_ID].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
      Serial.printf("ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d\n", conn_id, gattc_if, p_data->open.status, p_data->open.mtu);

      Serial.printf("REMOTE BDA  %02x:%02x:%02x:%02x:%02x:%02x\n",
                    gl_profile_tab[PROFILE_B_APP_ID].remote_bda[0], gl_profile_tab[PROFILE_B_APP_ID].remote_bda[1],
                    gl_profile_tab[PROFILE_B_APP_ID].remote_bda[2], gl_profile_tab[PROFILE_B_APP_ID].remote_bda[3],
                    gl_profile_tab[PROFILE_B_APP_ID].remote_bda[4], gl_profile_tab[PROFILE_B_APP_ID].remote_bda[5]
                   );

      esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
      break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        esp_gatt_srvc_id_t *srvc_id = &p_data->search_res.srvc_id;
        conn_id = p_data->search_res.conn_id;
        Serial.printf("SEARCH RES: conn_id = %x\n", conn_id);
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16) {
          Serial.printf("UUID16: %x\n", srvc_id->id.uuid.uuid.uuid16);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_32) {
          Serial.printf("UUID32: %x\n", srvc_id->id.uuid.uuid.uuid32);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) {
          Serial.printf("UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", srvc_id->id.uuid.uuid.uuid128[0],
                        srvc_id->id.uuid.uuid.uuid128[1], srvc_id->id.uuid.uuid.uuid128[2], srvc_id->id.uuid.uuid.uuid128[3],
                        srvc_id->id.uuid.uuid.uuid128[4], srvc_id->id.uuid.uuid.uuid128[5], srvc_id->id.uuid.uuid.uuid128[6],
                        srvc_id->id.uuid.uuid.uuid128[7], srvc_id->id.uuid.uuid.uuid128[8], srvc_id->id.uuid.uuid.uuid128[9],
                        srvc_id->id.uuid.uuid.uuid128[10], srvc_id->id.uuid.uuid.uuid128[11], srvc_id->id.uuid.uuid.uuid128[12],
                        srvc_id->id.uuid.uuid.uuid128[13], srvc_id->id.uuid.uuid.uuid128[14], srvc_id->id.uuid.uuid.uuid128[15]);
        } else {
          Serial.printf("ERROR: UNKNOWN LEN %d\n", srvc_id->id.uuid.len);
        }
        break;
      }
    case ESP_GATTC_SEARCH_CMPL_EVT:
      conn_id = p_data->search_cmpl.conn_id;
      Serial.printf("SEARCH_CMPL: conn_id = %x, status %d\n", conn_id, p_data->search_cmpl.status);
      esp_ble_gattc_get_characteristic(gattc_if, conn_id, &alert_service_id, NULL);
      break;
    case ESP_GATTC_GET_CHAR_EVT:
      if (p_data->get_char.status != ESP_GATT_OK) {
        break;
      }
      Serial.printf("GET CHAR: conn_id = %x, status %d\n", p_data->get_char.conn_id, p_data->get_char.status);
      Serial.printf("GET CHAR: srvc_id = %04x, char_id = %04x\n", p_data->get_char.srvc_id.id.uuid.uuid.uuid16, p_data->get_char.char_id.uuid.uuid.uuid16);

      if (p_data->get_char.char_id.uuid.uuid.uuid16 == 0x2a46) {
        Serial.printf("register notify\n");
        esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_B_APP_ID].remote_bda, &alert_service_id, &p_data->get_char.char_id);
      }

      esp_ble_gattc_get_characteristic(gattc_if, conn_id, &alert_service_id, &p_data->get_char.char_id);
      break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        uint16_t notify_en = 1;
        Serial.printf("REG FOR NOTIFY: status %d\n", p_data->reg_for_notify.status);
        Serial.printf("REG FOR_NOTIFY: srvc_id = %04x, char_id = %04x\n", p_data->reg_for_notify.srvc_id.id.uuid.uuid.uuid16, p_data->reg_for_notify.char_id.uuid.uuid.uuid16);

        esp_ble_gattc_write_char_descr(
          gattc_if,
          conn_id,
          &alert_service_id,
          &p_data->reg_for_notify.char_id,
          &notify_descr_id,
          sizeof(notify_en),
          (uint8_t *)&notify_en,
          ESP_GATT_WRITE_TYPE_RSP,
          ESP_GATT_AUTH_REQ_NONE);
        break;
      }
    case ESP_GATTC_NOTIFY_EVT:
      Serial.printf("NOTIFY: len %d, value %08x\n", p_data->notify.value_len, *(uint32_t *)p_data->notify.value);
      break;
    case ESP_GATTC_WRITE_DESCR_EVT:
      Serial.printf("WRITE: status %d\n", p_data->write.status);
      break;
    default:
      break;
  }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    uint8_t *adv_data = NULL;
    uint8_t adv_data_len = 0;
    
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 60;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {

            // most IDF GATT Client example code stripped out but this
            
            byte *deviceAddr = scan_result->scan_rst.bda;
            Serial.print("Msg from device ");
            macAddr = "";
            
            
            
            for (int i = 0; i < 6; i++) {
                       
              String macAddr_addition = String(deviceAddr[i], HEX);
              
              if (macAddr_addition.length() < 2){ 
              macAddr_addition = "0" + macAddr_addition;
              }
              
              macAddr = macAddr + macAddr_addition;
              
            }
            
            Serial.print("Mac: "+ macAddr);
            
            rssi = scan_result->scan_rst.rssi;            
            Serial.printf(", RSSI=%i, ", rssi);
            
            Serial.printf("\n");
            
            sendBleState();
     

            break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT: {
            // add code to start another scan
            Serial.println("scan completed, restarting ...");
            esp_ble_gap_start_scanning(60);
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
  Serial.printf("EVT %d, gattc if %d\n", event, gattc_if);

  /* If event is register event, store the gattc_if for each profile */
  if (event == ESP_GATTC_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
    } else {
      Serial.printf("Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
      return;
    }
  }

  /* If the gattc_if equal to profile A, call profile A cb handler,
     so here call each profile's callback */
  do {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
      if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
          gattc_if == gl_profile_tab[idx].gattc_if) {
        if (gl_profile_tab[idx].gattc_cb) {
          gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
        }
      }
    }
  } while (0);
}

void ble_client_appRegister(void)
{
  esp_err_t status;

  Serial.printf("register callback\n");

  //register the scan callback function to the gap moudule
  if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
    Serial.printf("ERROR: gap register error, error code = %x\n", status);
    return;
  }

  //register the callback function to the gattc module
  if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK) {
    Serial.printf("ERROR: gattc register error, error code = %x\n", status);
    return;
  }

  esp_ble_gattc_app_register(PROFILE_A_APP_ID);
  esp_ble_gattc_app_register(PROFILE_B_APP_ID);
  
}

/* Send State */

void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["lux"] = (String)LUX;
  root["temperature"] = (String)tempValue;
  root["humidity"] = (String)humValue;
  root["pressure"] = (String)presValue;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  client.publish(data_state_topic, buffer, true);
  
}

void sendBleState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["macaddr"] = (String)macAddr;
  root["rssi"] = (String)rssi;
  

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  client.publish(presence_state_topic, buffer, true);
  
}

/* Reconnect */
void reconnect() {
  // Loop until reconnected
  while(!client.connected()){
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(SENSORNAME, mqtt_user, mqtt_password)) {
      Serial.println("Connected");
      client.subscribe(data_set_topic);
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.print(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
     }
  }
}

/* Start Check Sensor */
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}
/* Start main loop */
void loop() {
  ArduinoOTA.handle();

  if(!client.connected()){
    //reconnect();
    software_Reset();
  }

  float newLuxValue = analogRead(LUXPIN);
  float newTempValue = bme.readTemperature();
  float newHumValue = bme.readHumidity();
  float newPresValue = bme.readPressure();
  
  client.loop();

 if (checkBoundSensor(newTempValue, tempValue, diffTemp)){ 
  tempValue = newTempValue;
  sendState();
 }

 if (checkBoundSensor(newHumValue, humValue, diffHum)){
  humValue = newHumValue;
  sendState();
 }

 if (checkBoundSensor(newPresValue, presValue, diffPres)){
  presValue = newPresValue;
  sendState();
 }

 if (checkBoundSensor(newLuxValue, LUX, diffLux)){
  LUX = newLuxValue;
  sendState();
 }
 
 
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
Serial.print("resetting");
esp_restart(); 
}

