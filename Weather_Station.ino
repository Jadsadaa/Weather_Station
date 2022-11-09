#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>
#include "NB_BC95_G.h"

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>

/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
const char* ssid = "greenio"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "green7650"; //replace "xxxxxx" with your WIFI's password

HardwareSerial modbus(2);
HardwareSerial myserial(1);


#define SERIAL1_RXPIN 14
#define SERIAL1_TXPIN 27

#define battPIN  34
#define donePIN  25

char buffer_receive[1000];
unsigned int count_buffer = 0;
String data_str = "";
int read_timeout = 0;

NB_BC95_G AISnb;

BluetoothSerial SerialBT;

// thingcontrol.io setup
String deviceToken = "";
String serverIP = "147.50.151.130";
String serverPort = "19956";
String json = "";


unsigned long previousMillisSleep;
unsigned long previousMillis;
unsigned long Readtime;

int Count_SEMI1000_Sensor = 0, Count_soil_Sensor = 0;
ModbusMaster node;


#define trigWDTPin    32
#define ledHeartPIN   0




struct Meter
{
  //for SEMI1000 Sensor
  String Wind_Speed ;
  String Wind_force ;
  String Wind_Direct_F ;
  String Wind_Direct_D ;
  String Hum_Air ;
  String Temp_Air ;
  String Noise_SEMI_1000 ;
  String Pm25 ;
  String Pm10 ;
  String atm ;
  String lux ;
  String light ;
  String rainfall ;

  // for Soil Sensor
  String Moisture;
  String Temp_Soil;
  String EC;
  String Ph;
  String Ni;
  String Pho;
  String Pot;

  // for CO2 Sensor
  String Noise_CO2 ;
  String CO2;

  String bat;
  String rssi;


};

Meter meter;
uint16_t modbusdata[14];

uint16_t Count_Soil = 0;
uint16_t Count_SEMI1000_10 = 0;
uint16_t Count_SEMI1000_50 = 0;

uint16_t TempSoil_now[11];
uint16_t Moisture_now[11];
uint16_t Ph_now[11];
uint16_t Wind_Speed_now[51];
uint16_t Wind_force_now[11];
uint16_t Wind_Direct_F_now[11];
uint16_t Wind_Direct_D_now[11];
uint16_t Hum_Air_now[11];
uint16_t Temp_Air_now[11];
uint16_t atm_now[11];
uint16_t rainfall_now[11];

unsigned long ms;


//WiFi&OTA 参数
String HOSTNAME = "greenio";
#define PASSWORD "green7650" //the password for OTA upgrade, can set it in any char you want

void setup()
{
  HeartBeat();
  Serial.begin(115200);
  modbus.begin(4800, SERIAL_8N1, 16, 17);


  HeartBeat();
  delay(2000);

  HeartBeat();
  HOSTNAME.concat(getMacAddress());
  SerialBT.begin(HOSTNAME); //Bluetooth device name
  SerialBT.println(HOSTNAME);
  AISnb.debug = true;
  AISnb.delayAfterCommand = 1000;
  AISnb.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  Serial.println("Waiting for AIS NB test status!");
  while (!AISnb.testCommand().status)
  {
    Serial.print('.');
    SerialBT.print(',');
  }
  Serial.println("AIS NB OK!");
  SerialBT.println("AIS NB OK!");

  AISnb.setupDevice(serverPort);
  nb_resp_t res_DeviceIP = AISnb.getDeviceIP();
  nb_resp_t res_testPing = AISnb.testPing(serverIP);
  nb_resp_t res_nccid = AISnb.getNCCID();
  deviceToken = res_nccid.data;
  while (deviceToken == 0) {
    nb_resp_t res_nccid = AISnb.getNCCID();
    deviceToken = res_nccid.data;
  }
  SerialBT.print("NCCID:");
  SerialBT.println(deviceToken);
  SerialBT.println("VER:0");


  Serial.print("NCCID:");
  Serial.println(deviceToken);
  HeartBeat();


  Serial.println();
  Serial.println(F("***********************************"));
  Serial.println("Initialize...");
  HeartBeat();
  setupWIFI();
  HeartBeat();
  setupOTA();
  HeartBeat();
}







String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

void setupWIFI()
{
  Serial.println("Connecting...");
  Serial.println(String(ssid));
  //连接WiFi，删除旧的配置，关闭WIFI，准备重新配置
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  //WiFi.onEvent(WiFiEvent);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);    //断开WiFi后自动重新连接,ESP32不可用
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");
}


void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);
  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());
  //No authentication by default
  ArduinoOTA.setPassword(password);
  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");
    SerialBT.println("Start Updating....");
    HeartBeat();
    SerialBT.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });
  ArduinoOTA.onEnd([]()
  {
    SerialBT.println("Update Complete!");
    Serial.println("Update Complete!");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    //    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);
    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
    ms = millis();
    if (ms % 10000 == 0)
    {
      HeartBeat();
    }
  });
  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;
      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;
      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;
      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;
      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }
    Serial.println(info);
    ESP.restart();
  });
  ArduinoOTA.begin();
}

//********************************************************************//
//*********************** HeartBeat Function **************************//
//********************************************************************//
void HeartBeat() {
  //   Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);
  // Led monitor for Heartbeat
  digitalWrite(ledHeartPIN, LOW);
  delay(300);
  digitalWrite(ledHeartPIN, HIGH);
  // Return to high-Z
  pinMode(trigWDTPin, INPUT);
  Serial.println("Heartbeat");
}


void t2CallsendViaNBIOT ()
{

  nb_signal_t res_signal = AISnb.getSignal();
  meter.rssi = res_signal.rssi;

  while (meter.rssi == 0) {
    nb_signal_t res_signal = AISnb.getSignal();
    meter.rssi = res_signal.rssi;
  }
    Serial.print("RSSI:");
  Serial.println(meter.rssi);
  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"RSSI\":");
  //  json.concat(0);
  json.concat(meter.rssi);
  json.concat(",\"bat\":");
  json.concat(meter.bat);
  json.concat(",\"Ws\":");
  json.concat(meter.Wind_Speed);
  json.concat(",\"Wf\":");
  json.concat(meter.Wind_force);
  json.concat(",\"WdF\":");
  json.concat(meter.Wind_Direct_F);
  json.concat(",\"Wdd\":");
  json.concat(meter.Wind_Direct_D);
  json.concat(",\"Ha\":");
  json.concat(meter.Hum_Air);
  json.concat(",\"Ta\":");
  json.concat(meter.Temp_Air);
  json.concat(",\"atm\":");
  json.concat(meter.atm);
  json.concat(",\"lux\":");
  json.concat(meter.lux);
  json.concat(",\"light\":");
  json.concat(meter.light);
  json.concat(",\"R\":");
  json.concat(meter.rainfall);
  json.concat(",\"M\":");
  json.concat(meter.Moisture);
  json.concat(",\"Ts\":");
  json.concat(meter.Temp_Soil);
  json.concat(",\"Ec\":");
  json.concat(meter.EC);
  json.concat(",\"Ph\":");
  json.concat(meter.Ph);
  json.concat(",\"Ni\":");
  json.concat(meter.Ni);
  json.concat(",\"Pho\":");
  json.concat(meter.Pho);
  json.concat(",\"CO2\":");
  json.concat(meter.CO2);
  json.concat("}");
  Serial.println(json);
}


void readsensor()
{
  readModbus(ID_SEMI_1000, Address[0], 14);
  delay(50);
  readModbus(ID_SoilSensor, Address2[0], 7);
  delay(50);
  readModbus(ID_CO2_Sensor, Address3[0], 3);
}

void readModbus(char addr , uint16_t  REG, uint8_t  Read)
{
  static uint32_t i;
  uint16_t j, k, result;

  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(addr, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, Read);
  //Serial.print("result:"); Serial.println(result); Serial.print(" node.ku8MBSuccess:"); Serial.println(node.ku8MBSuccess);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < Read; j++)
    {
      modbusdata[j] = node.getResponseBuffer(j);
      // Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(modbusdata[j]);
    }

     switch (addr) {
      case ID_SEMI_1000:
        if (Count_SEMI1000_10 < 9) {
          Wind_force_now[Count_SEMI1000_10] = modbusdata[1];
          Wind_Direct_F_now[Count_SEMI1000_10] = modbusdata[2];
          Wind_Direct_D_now[Count_SEMI1000_10] = modbusdata[3];
          Hum_Air_now[Count_SEMI1000_10] = modbusdata[4];
          Temp_Air_now[Count_SEMI1000_10] = modbusdata[5];
          atm_now[Count_SEMI1000_10] = modbusdata[9];
          rainfall_now[Count_SEMI1000_10] = modbusdata[13];

          Count_SEMI1000_10 ++;

        } else if (Count_SEMI1000_10 == 9) {
          for (int i = 0; i < 10 ; ++i) {
            Wind_force_now[10]    += Wind_force_now[i];
            Wind_Direct_F_now[10] += Wind_Direct_F_now[i];
            Wind_Direct_D_now[10] += Wind_Direct_D_now[i];
            Hum_Air_now[10]       += Hum_Air_now[i];
            Temp_Air_now[10]      += Temp_Air_now[i];
            atm_now[10]           += atm_now[i];
            rainfall_now[10]      += rainfall_now[i];
          }

          meter.Wind_force = Wind_force_now[10] / 10;
          meter.Wind_Direct_F =  Wind_Direct_F_now[10] / 10;
          meter.Wind_Direct_D =   Wind_Direct_D_now[10] / 10;
          meter.Hum_Air =  Hum_Air_now[10] / 10;
          meter.Temp_Air =  Temp_Air_now[10] / 10;
          meter.atm =  atm_now[10] / 10;
          meter.rainfall = rainfall_now[10] / 10;

          Wind_force_now[10]    = 0;
          Wind_Direct_F_now[10] = 0;
          Wind_Direct_D_now[10] = 0;
          Hum_Air_now[10]       = 0;
          Temp_Air_now[10]      = 0;
          atm_now[10]           = 0;
          rainfall_now[10]      = 0;
          Count_SEMI1000_10 = 0 ;
        }

        if (Count_SEMI1000_50 < 49) {
          Wind_Speed_now[Count_SEMI1000_50] = modbusdata[0];
          Count_SEMI1000_50 ++;
        } else if (Count_SEMI1000_50 == 49) {
          for (int i = 0; i < 50 ; ++i) {
            Wind_Speed_now[50]   += Wind_force_now[i];
          }
          meter.Wind_Speed = Wind_Speed_now[50] / 50;
          Wind_Speed_now[50]    = 0;
          Count_SEMI1000_50 = 0 ;
        }

        meter.lux =  modbusdata[11] + modbusdata[11];
        meter.light =  modbusdata[12];
        break;


      case ID_SoilSensor:

        if (Count_Soil < 9) {
          Moisture_now[Count_Soil] = modbusdata[0];
          TempSoil_now[Count_Soil] = modbusdata[1];
          Ph_now[Count_Soil] = modbusdata[3];
          Count_Soil ++;
        } else if (Count_Soil == 9) {
          for (int i = 0; i < 10 ; ++i) {
            Moisture_now[10] += Moisture_now[i];
            TempSoil_now[10]    += TempSoil_now[i];
            Ph_now[10] += Ph_now[i];
          }

          meter.Moisture = Moisture_now[10] / 10;
          meter.Temp_Soil = TempSoil_now[10] / 10;
          meter.Ph = Ph_now[10] / 10;

          TempSoil_now[10] = 0;
          Moisture_now[10] = 0;
          Ph_now[10] = 0;
          Count_Soil = 0 ;
        }

        meter.EC =  modbusdata[2];
        meter.Ni =  modbusdata[4];
        meter.Pho =  modbusdata[5];
        meter.Pot =  modbusdata[6];

        break;
      case ID_CO2_Sensor:
        meter.Noise_CO2 = modbusdata[0];
        meter.CO2 =  modbusdata[1];
        break;
      default:
        break;
      }  


   /* switch (addr) {
      case ID_SEMI_1000:
        meter.Wind_Speed = modbusdata[0];
        meter.Wind_force =  modbusdata[1];
        meter.Wind_Direct_F = modbusdata[2];
        meter.Wind_Direct_D =   modbusdata[3];
        meter.Hum_Air =  modbusdata[4];
        meter.Temp_Air =  modbusdata[5];
        meter.atm =  modbusdata[9];
        meter.lux =  modbusdata[11] + modbusdata[11];
        meter.light =  modbusdata[12];
        meter.rainfall = modbusdata[13];
        break;

      case ID_SoilSensor:
        meter.Moisture = modbusdata[0];
        meter.Temp_Soil = modbusdata[1];
        meter.EC =  modbusdata[2];
        meter.Ph = modbusdata[3];
        meter.Ni =  modbusdata[4];
        meter.Pho =  modbusdata[5];
        meter.Pot =  modbusdata[6];

        break;
      case ID_CO2_Sensor:
        meter.Noise_CO2 = modbusdata[0];
        meter.CO2 =  modbusdata[1];
        break;
      default:
        break;
    }*/


  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug

  }
}


void loop()
{
  ms = millis();
  if (ms % 30000 == 0)
  {
    Serial.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );
    setupWIFI();
    HeartBeat();
    setupOTA();
    previousMillisSleep = millis();
  }

  if (ms - Readtime > 1000) {
    readsensor();
    Readtime = millis();
  }
  if (ms - previousMillis > 120000) {
    meter.bat = Read_Batt();
    t2CallsendViaNBIOT();
    nb_resp_t res_send = AISnb.sendUDPMessage(1, serverIP, serverPort, json.length(), json, MODE_STRING_HEX);
    if (!res_send.status)
    {
      AISnb.createUDPSocket(serverPort);
    }
    String getResponse = AISnb.getSocketResponse();
    if (getResponse.length() > 0) {
      Serial.print("UDP response: ");
      Serial.println(getResponse);
    }
    
    previousMillis = millis();
    doneProcess();
  }
  ArduinoOTA.handle();
}


float Read_Batt()
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  float R1 = 15000.0;
  float R2 = 3450.0;
  int bitRes = 4096;
  int16_t adc = 0;
  //Serial.println("Read_Batt()");
  for (int a = 0; a < 20; a++)
  {
    adc  += analogRead(battPIN);
    //Serial.print(adc);
    delay(1);
  }

  vRAW = adc / 20;
  Vout = (vRAW * 3.3) / bitRes;
  Vin = Vout / (R2 / (R1 + R2));
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  //Serial.println("end.Read_Batt()");
  return Vin;
}

float HexTofloat(uint32_t x)
{
  return (*(float*)&x);
}

String decToHex(int decValue) {
  String hexString = String(decValue, HEX);
  return hexString;
}

unsigned int hexToDec(String hexString) {
  unsigned int decValue = 0;
  int nextInt;
  for (int i = 0; i < hexString.length(); i++) {
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    decValue = (decValue * 16) + nextInt;
  }
  return decValue;
}

int getResult( unsigned int x_high, unsigned int x_low)
{
  String hex2 = "";
  hex2.concat(decToHex(x_low));
  hex2.concat(decToHex(x_high));
  Serial.print("hex:");  Serial.println(hex2);
  Serial.print("dec:");  Serial.println(hexToDec(hex2));                                                               //rightmost 8 bits
  return hexToDec(hex2);
}

/****************************************************
   [通用函数]ESP32 WiFi Kit 32事件处理
*/
void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:               /**< ESP32 WiFi ready */
      break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
      break;
    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
      break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
      break;
    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
      break;
    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
    case SYSTEM_EVENT_STA_LOST_IP:              /**< ESP32 station lost IP and the IP is reset to 0 */
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
      break;
    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
      break;
    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
    case SYSTEM_EVENT_MAX:
      break;
  }
}


void doneProcess()
{
  //Serial.println("!!!!!! Done ready to Sleep ~10 Min (TPL5110) !!!!!!");
  pinMode(donePIN, OUTPUT);
  digitalWrite(donePIN, HIGH);
  delay(100);
}
