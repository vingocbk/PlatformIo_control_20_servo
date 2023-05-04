#include <Arduino.h>
#include "config.h"
#include "AppDebug.h"
#include "app_state.h"
#include <Wire.h>
#include "soc/soc.h"  //Brownout detector was triggered
#include "soc/rtc_cntl_reg.h"
#include "EEPROM.h"
#include "ArduinoJson.h"
#include "BluetoothSerial.h"
#include "PCA9685.h"

BluetoothSerial SerialBT;
PCA9685 pwmController_1(ADDRESS_PCA9685_1);           // Library using Wire1 @400kHz
PCA9685 pwmController_2(ADDRESS_PCA9685_2);           // Library using Wire1 @400kHz
PCA9685_ServoEval pwmServo;

String dataSendPreset = "[{\"1\":\"ngoc\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"tuyet\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"tuan\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hcchu\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhy\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"gxyy\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhx\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"t8t7ry\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhxucupc\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}]#";
String dataSendTour = "{\"1\":[{\"1\":\"ngoc\",\"2\":50},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"2\":[{\"1\":\"tuyet\",\"2\":40},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"3\":[{\"1\":\"tuan\",\"2\":50},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"4\":[{\"1\":\"tuan\",\"2\":5},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"5\":[0,2]}%";

struct DataConfig
{
    /* data */
    uint8_t totalPreset = 0;
    String namePreset[20]; 
    uint8_t speedPreset[20]; 
    uint16_t anglePreset[20][20]; 
    uint8_t totalPresetOfTour1 = 0;
    String namePresetOfTour1[20]; 
    uint8_t totalPresetOfTour2 = 0;
    String namePresetOfTour2[20]; 
    uint8_t totalPresetOfTour3 = 0;
    String namePresetOfTour3[20]; 
    uint8_t totalPresetOfTour4 = 0;
    String namePresetOfTour4[20]; 
    uint8_t modeOpenTour5 = 0;
    uint8_t modeCloseTour5 = 0;
}dataConfig;

void pca9685Init();
void bluetoothInit();
void saveDataPreset(JsonArray& dataPreset);
void callbackBluetooth(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void readPulseInMode3(void *pvParameters);
void sendDataToApp(String data);
void loadDataFromEeprom();


void pca9685Init(){
    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    ECHOLN ();
    ECHOLN ("I2C scanner. Scanning ...");

    int count = 0;
    for (uint8_t i = 8; i < 120; i++)
    {
        Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
        if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
        {
        Serial.print ("Found address: ");
        Serial.print (i, DEC);
        Serial.print (" (0x");
        Serial.print (i, HEX);     // PCF8574 7 bit address
        ECHOLN (")");
        // ina219_info.address_ina[count] = i;
        count++;
        }
    }
    Serial.print ("Found ");      
    Serial.print (count, DEC);        // numbers of devices
    ECHOLN (" device(s).");
	pwmController_1.resetDevices();       // Resets all PCA9685 devices on i2c line
    pwmController_1.init();               // Initializes module using default totem-pole driver mode, and default disabled phase balancer
    pwmController_1.setPWMFreqServo();    // 50Hz provides standard 20ms servo phase length

	// pwmController_2.resetDevices();       // Resets all PCA9685 devices on i2c line
    pwmController_2.init();               // Initializes module using default totem-pole driver mode, and default disabled phase balancer
    pwmController_2.setPWMFreqServo();    // 50Hz provides standard 20ms servo phase length
}

void bluetoothInit()
{
    SerialBT.flush();
    SerialBT.end(); 
    if(!SerialBT.begin("Sc 001")){
        ECHOLN("An error occurred initializing Bluetooth");
    }else{
        ECHOLN("Bluetooth initialized");
    }
  	SerialBT.register_callback(callbackBluetooth);
}

void saveDataPreset(JsonArray& dataPreset){
    dataConfig.totalPreset = (uint8_t)dataPreset.size();
    EEPROM.write(EEPROM_NUMBER_TOTAL_PRESET,(uint8_t)dataPreset.size());
    for(int i = 0; i < dataConfig.totalPreset; i++){
        dataConfig.namePreset[i] = "";
    }
    for(int i = 0; i < dataConfig.totalPreset; i++){
        dataConfig.namePreset[i] = dataPreset[i]["1"].as<String>();
        ECHO(dataConfig.namePreset[i]);
        ECHO(" - ");
        //Clear name EERPROM first
        for(int j = 0; j < DISTANT_FROM_2_NAME_OF_PRESET_TOUR; j++){
            EEPROM.write(EEPROM_NAME_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j, 0);
        }
        //then save name
        for (int j = 0; j < dataConfig.namePreset[i].length(); ++j){
            EEPROM.write(EEPROM_NAME_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j, dataConfig.namePreset[i][j]);             
        }

        //speed
        dataConfig.speedPreset[i] = dataPreset[i]["2"];
        EEPROM.write(EEPROM_SPEED_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i, dataConfig.speedPreset[i]);
        ECHO( dataConfig.speedPreset[i]);
        ECHO("-");

        //angle
        JsonArray& jsonArrayAngle = dataPreset[i]["3"].as<JsonArray&>();
        for(int j = 0; j < jsonArrayAngle.size(); j++){
            // uint8_t angle = jsonArrayAngle[j].as<int>();
            dataConfig.anglePreset[i][j] = jsonArrayAngle[j].as<int>();
            EEPROM.write(EEPROM_ANGLE_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j, dataConfig.anglePreset[i][j]);
            ECHO(dataConfig.anglePreset[i][j]);
            ECHO("-");
        }
        ECHOLN();
    }
    EEPROM.commit();
}

void callbackBluetooth(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	switch (event)
  	{
  	case ESP_SPP_SRV_OPEN_EVT:
    	ECHOLN("Client Connected");
    	break;
    case ESP_SPP_CLOSE_EVT:
        ECHOLN("Client Disconnected");
    	break;
  	case ESP_SPP_DATA_IND_EVT:  
    	if (param->data_ind.len < MAX_RESPONSE_LENGTH)
        {
            String data;
            for(int i = 0; i < param->data_ind.len; i++)
            {
                data += (char)param->data_ind.data[i];
            }
            ECHO("data bluetooth: ");
            ECHOLN(data);
            StaticJsonBuffer<MAX_RESPONSE_LENGTH> jsonBuffer;
            JsonObject& rootData = jsonBuffer.parseObject(data);
            if(rootData.success())
            {
                String type = rootData["type"];
                if(type == "sync_preset"){
                    // sendDataToApp(dataSendPreset);
                    APP_FLAG_SET(SEND_DATA_PRESET);
                }
                else if(type == "sync_tour"){
                    // sendDataToApp(dataSendTour);
                    APP_FLAG_SET(SEND_DATA_TOUR);
                }
                else if(type == "save_preset"){
                    JsonArray& dataPreset = rootData["data"].as<JsonArray&>();
                    saveDataPreset(dataPreset);
                    
                }
                else if(type == "save_tour"){

                }
                else if(type == "control"){
                    String mode = rootData["mode"];
                    String data = rootData["data"];
                    if(mode == "preset"){
                        
                    }
                    else if(mode == "tour"){
                        if(data == "mode 1"){

                        }
                        else if(data == "mode 2"){

                        }
                        else if(data == "mode 3"){

                        }
                        else if(data == "mode 4"){

                        }
                        else if(data == "mode 5"){

                        }
                    }
                }
            }
        }
        break;
    default:
        break;
    }
}


// void readPulseInMode3(void *pvParameters){
//     bool check_start_connect_mode_run = false;
//     unsigned long check_time_begin;
// 	for( ;; )
// 	{
// 		vTaskDelay(100/portTICK_RATE_MS);
// 	}
// }

void sendDataToApp(String data){
    ECHOLN("sendDataToApp");
    for(int i = 0; i < data.length(); i++){
        SerialBT.write(data[i]);
    }
}

void loadDataFromEeprom(){
    dataConfig.totalPreset = EEPROM.read(EEPROM_NUMBER_TOTAL_PRESET);
    if(dataConfig.totalPreset == 255){
        dataConfig.totalPreset = 0;
    }
    for(int i = 0; i < dataConfig.totalPreset; i++){
        dataConfig.namePreset[i] = "";
        for(int j = 0; j < DISTANT_FROM_2_NAME_OF_PRESET_TOUR; j++){
            if(EEPROM.read(EEPROM_NAME_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j) == 0){
                break;
            }
            dataConfig.namePreset[i] += char(EEPROM.read(EEPROM_NAME_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j));
        }
        ECHO(dataConfig.namePreset[i]);
        ECHO(" - ");
        for(int j = 0; j < 20; j++){
            dataConfig.anglePreset[i][j] = EEPROM.read(EEPROM_ANGLE_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j);
            ECHO(dataConfig.anglePreset[i][j]);
            ECHO("-");
        }
        ECHOLN();
    }

    dataConfig.totalPresetOfTour1 = EEPROM.read(EEPROM_NUMBER_TOTAL_TOUR_MODE_1);
    if(dataConfig.totalPresetOfTour1 == 255){
        dataConfig.totalPresetOfTour1 = 0;
    }
    dataConfig.totalPresetOfTour2 = EEPROM.read(EEPROM_NUMBER_TOTAL_TOUR_MODE_2);
    if(dataConfig.totalPresetOfTour2 == 255){
        dataConfig.totalPresetOfTour2 = 0;
    }
    dataConfig.totalPresetOfTour3 = EEPROM.read(EEPROM_NUMBER_TOTAL_TOUR_MODE_3);
    if(dataConfig.totalPresetOfTour3 == 255){
        dataConfig.totalPresetOfTour3 = 0;
    }
    dataConfig.totalPresetOfTour4 = EEPROM.read(EEPROM_NUMBER_TOTAL_TOUR_MODE_4);
    if(dataConfig.totalPresetOfTour4 == 255){
        dataConfig.totalPresetOfTour4 = 0;
    }
    dataConfig.modeOpenTour5 = EEPROM.read(EEPROM_OPEN_TOUR_MODE_5);
    dataConfig.modeCloseTour5 = EEPROM.read(EEPROM_CLOSE_TOUR_MODE_5);

    // for(int i)
    
}


void setup() {
	// put your setup code here, to run once:
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //Brownout detector was triggered
    Serial.begin (SERIAL_BAUDRATE);  
    Wire.begin (SDA_PIN, SCL_PIN);   // sda= GPIO_21 /scl= GPIO_22
    EEPROM.begin(MAX_SIZE_EEPROM_BUFFER);
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    vTaskDelay(10/portTICK_RATE_MS);
    digitalWrite(PIN_LED_R, LED_ON);
    digitalWrite(PIN_LED_G, LED_OFF);
    digitalWrite(PIN_LED_B, LED_OFF);
	pca9685Init();
	bluetoothInit();
    loadDataFromEeprom();

    while (1)
    {
        for(int i = 102; i < 512; i++){
            pwmController_1.setChannelPWM(PIN_SERVO_1, i);
            pwmController_1.setChannelPWM(PIN_SERVO_2, i);
            pwmController_1.setChannelPWM(PIN_SERVO_3, i);
            pwmController_1.setChannelPWM(PIN_SERVO_4, i);
            pwmController_1.setChannelPWM(PIN_SERVO_5, i);
            pwmController_1.setChannelPWM(PIN_SERVO_6, i);
            pwmController_1.setChannelPWM(PIN_SERVO_7, i);
            pwmController_1.setChannelPWM(PIN_SERVO_8, i);
            pwmController_1.setChannelPWM(PIN_SERVO_9, i);
            pwmController_1.setChannelPWM(PIN_SERVO_10, i);
            pwmController_1.setChannelPWM(PIN_SERVO_11, i);
            pwmController_1.setChannelPWM(PIN_SERVO_12, i);
            pwmController_2.setChannelPWM(PIN_SERVO_13, i);
            pwmController_2.setChannelPWM(PIN_SERVO_14, i);
            pwmController_2.setChannelPWM(PIN_SERVO_15, i);
            pwmController_2.setChannelPWM(PIN_SERVO_16, i);
            pwmController_2.setChannelPWM(PIN_SERVO_17, i);
            pwmController_2.setChannelPWM(PIN_SERVO_18, i);
            pwmController_2.setChannelPWM(PIN_SERVO_19, i);
            pwmController_2.setChannelPWM(PIN_SERVO_20, i);
            vTaskDelay(30/portTICK_RATE_MS);
        }

        for(int i = 512; i > 102; i--){
            pwmController_1.setChannelPWM(PIN_SERVO_1, i);
            pwmController_1.setChannelPWM(PIN_SERVO_2, i);
            pwmController_1.setChannelPWM(PIN_SERVO_3, i);
            pwmController_1.setChannelPWM(PIN_SERVO_4, i);
            pwmController_1.setChannelPWM(PIN_SERVO_5, i);
            pwmController_1.setChannelPWM(PIN_SERVO_6, i);
            pwmController_1.setChannelPWM(PIN_SERVO_7, i);
            pwmController_1.setChannelPWM(PIN_SERVO_8, i);
            pwmController_1.setChannelPWM(PIN_SERVO_9, i);
            pwmController_1.setChannelPWM(PIN_SERVO_10, i);
            pwmController_1.setChannelPWM(PIN_SERVO_11, i);
            pwmController_1.setChannelPWM(PIN_SERVO_12, i);
            pwmController_2.setChannelPWM(PIN_SERVO_13, i);
            pwmController_2.setChannelPWM(PIN_SERVO_14, i);
            pwmController_2.setChannelPWM(PIN_SERVO_15, i);
            pwmController_2.setChannelPWM(PIN_SERVO_16, i);
            pwmController_2.setChannelPWM(PIN_SERVO_17, i);
            pwmController_2.setChannelPWM(PIN_SERVO_18, i);
            pwmController_2.setChannelPWM(PIN_SERVO_19, i);
            pwmController_2.setChannelPWM(PIN_SERVO_20, i);
            vTaskDelay(30/portTICK_RATE_MS);
        }
        
    }
    

	// xTaskCreatePinnedToCore(
	// 	readPulseInMode3,    /* Function to implement the task */
	// 	"readPulseInMode3",  /* Name of the task */
	// 	4096,             /* Stack size in words */
	// 	NULL,             /* Task input parameter */
	// 	1,                /* Priority of the task */
	// 	NULL,             /* Task handle. */
	// 	1);               /* Core where the task should run */
}

void loop() {
  	// put your main code here, to run repeatedly:
    if(digitalRead(0) == LOW){
        vTaskDelay(500/portTICK_RATE_MS);
        sendDataToApp(dataSendPreset);
    }
    if(APP_FLAG(SEND_DATA_PRESET)){
        APP_FLAG_CLEAR(SEND_DATA_PRESET);
        sendDataToApp(dataSendPreset);
    }
    if(APP_FLAG(SEND_DATA_TOUR)){
        APP_FLAG_CLEAR(SEND_DATA_TOUR);
        sendDataToApp(dataSendTour);
    }
}