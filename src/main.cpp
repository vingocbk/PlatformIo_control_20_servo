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
#include "struct.h"

BluetoothSerial SerialBT;
PCA9685 pwmController_1(ADDRESS_PCA9685_1);           // Library using Wire1 @400kHz
PCA9685 pwmController_2(ADDRESS_PCA9685_2);           // Library using Wire1 @400kHz
PCA9685_ServoEval pwmServo;
SemaphoreHandle_t xMutexI2C;

// String dataSendPreset = "[{\"1\":\"ngoc\",\"2\":9,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"tuyet\",\"2\":9,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"tuan\",\"2\":8,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hcchu\",\"2\":9,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhy\",\"2\":9,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"gxyy\",\"2\":9,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhx\",\"2\":9,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"t8t7ry\",\"2\":9,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhxucupc\",\"2\":9,\"3\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}]#";
// String dataSendTour = "{\"1\":[{\"1\":\"ngoc\",\"2\":50},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"2\":[{\"1\":\"tuyet\",\"2\":40},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"3\":[{\"1\":\"tuan\",\"2\":50},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"4\":[{\"1\":\"tuan\",\"2\":5},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}]}%";

struct DataConfig
{
    /* data */
    uint8_t totalPreset = 0;
    String namePreset[MAX_SERVO]; 
    uint8_t speedPreset[MAX_SERVO]; 
    uint16_t anglePreset[MAX_SERVO][MAX_SERVO]; 
    uint8_t totalPresetOfTour1 = 0;
    String namePresetOfTour1[MAX_SERVO]; 
    uint8_t timePresetOfTour1[MAX_SERVO];
    uint8_t totalPresetOfTour2 = 0;
    String namePresetOfTour2[MAX_SERVO]; 
    uint8_t timePresetOfTour2[MAX_SERVO];
    uint8_t totalPresetOfTour3 = 0;
    String namePresetOfTour3[MAX_SERVO]; 
    uint8_t timePresetOfTour3[MAX_SERVO];
    uint8_t totalPresetOfTour4 = 0;
    String namePresetOfTour4[MAX_SERVO]; 
    uint8_t timePresetOfTour4[MAX_SERVO];
    // uint8_t modeOpenTour5 = 0;
    // uint8_t modeCloseTour5 = 0;

    String currentDataPreset;
    String currentDataTour;

    uint8_t currentModeRun; //check current moderun
    uint8_t previousModeRun; //check previous moderun
    String nameOfModeRunPreset; //current name of Preset if run mode preset
    uint8_t index_position_current_preset;  //preset of current running mode on all preset and tour mode

    uint8_t statusCurrentServo[MAX_SERVO] = {SERVO_STOP, };
    int value_old_pwm_servo[MAX_SERVO] = {0};
    int value_new_pwm_servo[MAX_SERVO] = {0};
}dataConfig;

void pca9685Init();
void bluetoothInit();
void saveDataPreset(JsonArray& dataPreset);
void saveDataTour(JsonObject& dataTour);
void callbackBluetooth(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
// void readPulseInMode3(void *pvParameters);
void sendDataToApp(String data);
// void sendDataPresetToApp(String data);
// void sendDataTourToApp(String data);
void loadDataFromEeprom();
void setupStartPositionServo();
bool doneRunPreset();
void controlServo1(void *pvParameters);
// void controlServo2(void *pvParameters);
// void controlServo3(void *pvParameters);
// void controlServo4(void *pvParameters);
// void controlServo5(void *pvParameters);
// void controlServo6(void *pvParameters);
// void controlServo7(void *pvParameters);
// void controlServo8(void *pvParameters);
// void controlServo9(void *pvParameters);
// void controlServo10(void *pvParameters);
// void controlServo11(void *pvParameters);
// void controlServo12(void *pvParameters);
// void controlServo13(void *pvParameters);
// void controlServo14(void *pvParameters);
// void controlServo15(void *pvParameters);
// void controlServo16(void *pvParameters);
// void controlServo17(void *pvParameters);
// void controlServo18(void *pvParameters);
// void controlServo19(void *pvParameters);
// void controlServo20(void *pvParameters);
void controlModeRun(void *pvParameters);

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
    xMutexI2C = xSemaphoreCreateMutex();
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
        for(int j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            EEPROM.write(EEPROM_NAME_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j, 0);
        }
        //then save name
        for (int j = 0; j < dataConfig.namePreset[i].length(); ++j){
            EEPROM.write(EEPROM_NAME_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j, dataConfig.namePreset[i][j]);             
        }

        //speed
        dataConfig.speedPreset[i] = dataPreset[i]["2"];
        if(dataConfig.speedPreset[i] > MAX_PERCENT_SPEED_SERVO){
            dataConfig.speedPreset[i] = MAX_PERCENT_SPEED_SERVO;
            dataPreset[i]["2"] = dataConfig.speedPreset[i];
        }
        else if(dataConfig.speedPreset[i] < MIN_PERCENT_SPEED_SERVO){
            dataConfig.speedPreset[i] = MIN_PERCENT_SPEED_SERVO;
            dataPreset[i]["2"] = dataConfig.speedPreset[i];
        }
        EEPROM.write(EEPROM_SPEED_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i, dataConfig.speedPreset[i]);
        ECHO( dataConfig.speedPreset[i]);
        ECHO("-");

        //angle
        JsonArray& jsonArrayAngle = dataPreset[i]["3"].as<JsonArray&>();
        for(int j = 0; j < jsonArrayAngle.size(); j++){
            // uint8_t angle = jsonArrayAngle[j].as<int>();
            dataConfig.anglePreset[i][j] = jsonArrayAngle[j].as<int>();
            EEPROM.write(EEPROM_ANGLE_PRESET_BEGIN_HIGH + DISTANT_FROM_2_PRESET*i + 2*j, dataConfig.anglePreset[i][j] >> 8);
            EEPROM.write(EEPROM_ANGLE_PRESET_BEGIN_LOW + DISTANT_FROM_2_PRESET*i + 2*j, (uint8_t)dataConfig.anglePreset[i][j]);
            ECHO(dataConfig.anglePreset[i][j]);
            ECHO("-");
        }
        ECHOLN();
    }
    dataConfig.currentDataPreset.clear();
    dataPreset.printTo(dataConfig.currentDataPreset);
    ECHOLN(dataConfig.currentDataPreset);
    EEPROM.commit();
}

void saveDataTour(JsonObject& dataTour){
    JsonArray& jsonArrayMode1 = dataTour["1"].as<JsonArray&>();
    dataConfig.totalPresetOfTour1 = jsonArrayMode1.size();
    EEPROM.write(EEPROM_NUMBER_TOTAL_TOUR_MODE_1, dataConfig.totalPresetOfTour1);
    ECHO("Mode 1: ");
    for(uint8_t i = 0; i < dataConfig.totalPresetOfTour1; i++){
        dataConfig.namePresetOfTour1[i] = jsonArrayMode1[i]["1"].as<String>();
        ECHO("Name: ");
        ECHO(dataConfig.namePresetOfTour1[i]);
        ECHO(" - ");
        //Clear name EERPROM first
        for(int j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            EEPROM.write(EEPROM_NUMBER_NAME_BEGIN_MODE_1 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j, 0);
        }
        //then save name
        for (int j = 0; j < dataConfig.namePresetOfTour1[i].length(); ++j){
            EEPROM.write(EEPROM_NUMBER_NAME_BEGIN_MODE_1 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j, dataConfig.namePresetOfTour1[i][j]);             
        }

        dataConfig.timePresetOfTour1[i] = jsonArrayMode1[i]["2"].as<int>();
        EEPROM.write(EEPROM_NUMBER_TIME_TOUR_MODE_1 + DISTANT_FROM_2_PRESET_OF_TOUR*i,dataConfig.timePresetOfTour1[i]);
        ECHO("Time: ");
        ECHO(dataConfig.timePresetOfTour1[i]);
        ECHO(" --- ");
    }
    ECHOLN("");

    JsonArray& jsonArrayMode2 = dataTour["2"].as<JsonArray&>();
    dataConfig.totalPresetOfTour2 = jsonArrayMode2.size();
    EEPROM.write(EEPROM_NUMBER_TOTAL_TOUR_MODE_2, dataConfig.totalPresetOfTour2);
    ECHO("Mode 2: ");
    for(uint8_t i = 0; i < dataConfig.totalPresetOfTour2; i++){
        dataConfig.namePresetOfTour2[i] = jsonArrayMode2[i]["1"].as<String>();
        ECHO("Name: ");
        ECHO(dataConfig.namePresetOfTour2[i]);
        ECHO(" - ");
        //Clear name EERPROM first
        for(int j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            EEPROM.write(EEPROM_NUMBER_NAME_BEGIN_MODE_2 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j, 0);
        }
        //then save name
        for (int j = 0; j < dataConfig.namePresetOfTour2[i].length(); ++j){
            EEPROM.write(EEPROM_NUMBER_NAME_BEGIN_MODE_2 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j, dataConfig.namePresetOfTour2[i][j]);             
        }

        dataConfig.timePresetOfTour2[i] = jsonArrayMode2[i]["2"].as<int>();
        EEPROM.write(EEPROM_NUMBER_TIME_TOUR_MODE_2 + DISTANT_FROM_2_PRESET_OF_TOUR*i,dataConfig.timePresetOfTour2[i]);
        ECHO("Time: ");
        ECHO(dataConfig.timePresetOfTour2[i]);
        ECHO(" --- ");
    }
    ECHOLN("");

    JsonArray& jsonArrayMode3 = dataTour["3"].as<JsonArray&>();
    dataConfig.totalPresetOfTour3 = jsonArrayMode3.size();
    EEPROM.write(EEPROM_NUMBER_TOTAL_TOUR_MODE_3, dataConfig.totalPresetOfTour3);
    ECHO("Mode 3: ");
    for(uint8_t i = 0; i < dataConfig.totalPresetOfTour3; i++){
        dataConfig.namePresetOfTour3[i] = jsonArrayMode3[i]["1"].as<String>();
        ECHO("Name: ");
        ECHO(dataConfig.namePresetOfTour3[i]);
        ECHO(" - ");
        //Clear name EERPROM first
        for(int j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            EEPROM.write(EEPROM_NUMBER_NAME_BEGIN_MODE_3 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j, 0);
        }
        //then save name
        for (int j = 0; j < dataConfig.namePresetOfTour3[i].length(); ++j){
            EEPROM.write(EEPROM_NUMBER_NAME_BEGIN_MODE_3 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j, dataConfig.namePresetOfTour3[i][j]);             
        }

        dataConfig.timePresetOfTour3[i] = jsonArrayMode3[i]["2"].as<int>();
        EEPROM.write(EEPROM_NUMBER_TIME_TOUR_MODE_3 + DISTANT_FROM_2_PRESET_OF_TOUR*i,dataConfig.timePresetOfTour3[i]);
        ECHO("Time: ");
        ECHO(dataConfig.timePresetOfTour3[i]);
        ECHO(" --- ");
    }
    ECHOLN("");

    JsonArray& jsonArrayMode4 = dataTour["4"].as<JsonArray&>();
    dataConfig.totalPresetOfTour4 = jsonArrayMode4.size();
    EEPROM.write(EEPROM_NUMBER_TOTAL_TOUR_MODE_4, dataConfig.totalPresetOfTour4);
    ECHO("Mode 4: ");
    for(uint8_t i = 0; i < dataConfig.totalPresetOfTour4; i++){
        dataConfig.namePresetOfTour4[i] = jsonArrayMode4[i]["1"].as<String>();
        ECHO("Name: ");
        ECHO(dataConfig.namePresetOfTour4[i]);
        ECHO(" - ");
        //Clear name EERPROM first
        for(int j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            EEPROM.write(EEPROM_NUMBER_NAME_BEGIN_MODE_4 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j, 0);
        }
        //then save name
        for (int j = 0; j < dataConfig.namePresetOfTour4[i].length(); ++j){
            EEPROM.write(EEPROM_NUMBER_NAME_BEGIN_MODE_4 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j, dataConfig.namePresetOfTour4[i][j]);             
        }

        dataConfig.timePresetOfTour4[i] = jsonArrayMode4[i]["2"].as<int>();
        EEPROM.write(EEPROM_NUMBER_TIME_TOUR_MODE_4 + DISTANT_FROM_2_PRESET_OF_TOUR*i,dataConfig.timePresetOfTour4[i]);
        ECHO("Time: ");
        ECHO(dataConfig.timePresetOfTour4[i]);
        ECHO(" --- ");
    }
    ECHOLN("");
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
                    APP_FLAG_SET(SEND_DATA_PRESET);
                }
                else if(type == "save_tour"){
                    JsonObject& dataTour = rootData["data"].as<JsonObject&>();
                    dataConfig.currentDataTour.clear();
                    dataTour.printTo(dataConfig.currentDataTour);
                    ECHOLN(dataConfig.currentDataTour);
                    saveDataTour(dataTour);
                }
                else if(type == "control"){
                    String mode = rootData["mode"];
                    String data = rootData["data"];
                    DynamicJsonBuffer jsonBuffer(MAX_RESPONSE_LENGTH);
                    JsonObject& JsonObjectTour = jsonBuffer.parseObject(dataConfig.currentDataTour);
                    if(mode == "preset"){
                        dataConfig.currentModeRun = MODE_PRESET;
                        dataConfig.nameOfModeRunPreset = data;
                        //Clear name EERPROM first
                        for(int i = 0; i < DISTANT_FROM_2_PRESET_OF_TOUR; i++){
                            EEPROM.write(EEPROM_CURRENT_NAME_MODE_PRESET_BEGIN + i, 0);
                        }
                        //then save name
                        for (int i = 0; i < dataConfig.nameOfModeRunPreset.length(); ++i){
                            EEPROM.write(EEPROM_CURRENT_NAME_MODE_PRESET_BEGIN + i, dataConfig.nameOfModeRunPreset[i]);             
                        }
                        JsonObjectTour["5"] = dataConfig.nameOfModeRunPreset;
                        
                        for(int i = 0; i < dataConfig.totalPreset; i++){

                        }
                    }
                    else if(mode == "tour"){
                        if(data == "1"){
                            dataConfig.currentModeRun = MODE_TOUR_1;
                            JsonObjectTour["5"] = "Mode 1";
                        }
                        else if(data == "2"){
                            dataConfig.currentModeRun = MODE_TOUR_2;
                            JsonObjectTour["5"] = "Mode 2";
                        }
                        else if(data == "3"){
                            dataConfig.currentModeRun = MODE_TOUR_3;
                            JsonObjectTour["5"] = "Mode 3";
                        }
                        else if(data == "4"){
                            dataConfig.currentModeRun = MODE_TOUR_4;
                            JsonObjectTour["5"] = "Mode 4";
                        }
                    }
                    dataConfig.currentDataTour.clear();
                    JsonObjectTour.printTo(dataConfig.currentDataTour);
                    ECHOLN(dataConfig.currentDataTour);
                    APP_FLAG_SET(SEND_DATA_TOUR);
                    EEPROM.write(EEPROM_CURRENT_MODE_RUN, dataConfig.currentModeRun);
                    EEPROM.commit();
                }
            }
        }
        break;
    default:
        break;
    }
}



void sendDataToApp(String data){
    ECHOLN("sendDataToApp");
    for(int i = 0; i < data.length(); i++){
        SerialBT.write(data[i]);
    }
}

void loadDataFromEeprom(){
    //load data Preset
    DynamicJsonBuffer jsonBuffer(MAX_RESPONSE_LENGTH);
    JsonArray& dataArrayPreset = jsonBuffer.createArray();
    dataConfig.totalPreset = EEPROM.read(EEPROM_NUMBER_TOTAL_PRESET);
    if(dataConfig.totalPreset == 255){
        dataConfig.totalPreset = 0;
    }
    ECHO("Total Preset: ");
    ECHOLN(dataConfig.totalPreset);
    for(int i = 0; i < dataConfig.totalPreset; i++){
        // DynamicJsonBuffer jsonBuffer1(MAX_RESPONSE_LENGTH);
        // JsonObject& dataObject = jsonBuffer1.createObject();
        JsonObject& dataObject = dataArrayPreset.createNestedObject();
        dataConfig.namePreset[i] = "";
        for(int j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            if(EEPROM.read(EEPROM_NAME_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j) == 0){
                break;
            }
            dataConfig.namePreset[i] += char(EEPROM.read(EEPROM_NAME_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i + j));
        }
        dataObject["1"] = dataConfig.namePreset[i];
        ECHO(dataConfig.namePreset[i]);
        ECHO(" - ");
        dataConfig.speedPreset[i] = EEPROM.read(EEPROM_SPEED_PRESET_BEGIN + DISTANT_FROM_2_PRESET*i);
        dataObject["2"] = dataConfig.speedPreset[i];
        ECHO(dataConfig.speedPreset[i]);
        ECHO(" - ");
        JsonArray& dataArrayAngle = dataObject.createNestedArray("3");
        for(int j = 0; j < 20; j++){
            uint16_t angle_high = EEPROM.read(EEPROM_ANGLE_PRESET_BEGIN_HIGH + DISTANT_FROM_2_PRESET*i + 2*j);
            uint16_t angle_low = EEPROM.read(EEPROM_ANGLE_PRESET_BEGIN_LOW + DISTANT_FROM_2_PRESET*i + 2*j);
            dataConfig.anglePreset[i][j] = (angle_high << 8) | angle_low;
            dataArrayAngle.add(dataConfig.anglePreset[i][j]);
            ECHO(dataConfig.anglePreset[i][j]);
            ECHO("-");
        }
        ECHOLN();
    }

    // ECHOLN(dataConfig.currentDataPreset);
    dataConfig.currentDataPreset.clear();
    dataArrayPreset.printTo(dataConfig.currentDataPreset);
    ECHOLN(dataConfig.currentDataPreset);

    //load data Tour
    DynamicJsonBuffer jsonBufferTour(MAX_RESPONSE_LENGTH);
    JsonObject& dataObjectTour = jsonBufferTour.createObject();
    ECHO("Mode 1: ");
    dataConfig.totalPresetOfTour1 = EEPROM.read(EEPROM_NUMBER_TOTAL_TOUR_MODE_1);
    if(dataConfig.totalPresetOfTour1 == 255){
        dataConfig.totalPresetOfTour1 = 0;
    }
    JsonArray& dataArrayMode1 = dataObjectTour.createNestedArray("1");
    for(uint8_t i = 0; i < dataConfig.totalPresetOfTour1; i++){
        JsonObject& dataObjectNameMode1 = dataArrayMode1.createNestedObject();
        for(uint8_t j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            if(EEPROM.read(EEPROM_NUMBER_NAME_BEGIN_MODE_1 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j) == 0){
                break;
            }
            dataConfig.namePresetOfTour1[i] += char(EEPROM.read(EEPROM_NUMBER_NAME_BEGIN_MODE_1 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j));
        }
        dataObjectNameMode1["1"] = dataConfig.namePresetOfTour1[i];
        ECHO("Name: ");
        ECHO(dataConfig.namePresetOfTour1[i]);
        ECHO(" - ");

        dataConfig.timePresetOfTour1[i] = EEPROM.read(EEPROM_NUMBER_TIME_TOUR_MODE_1 + DISTANT_FROM_2_PRESET_OF_TOUR*i);
        dataObjectNameMode1["2"] = dataConfig.timePresetOfTour1[i];
        ECHO("Time: ");
        ECHO(dataConfig.timePresetOfTour1[i]);
        ECHO(" --- ");
        // dataArrayMode1.printTo(Serial);
        // ECHOLN("");
    }
    
    ECHO("Mode 2: ");
    dataConfig.totalPresetOfTour2 = EEPROM.read(EEPROM_NUMBER_TOTAL_TOUR_MODE_2);
    if(dataConfig.totalPresetOfTour2 == 255){
        dataConfig.totalPresetOfTour2 = 0;
    }
    JsonArray& dataArrayMode2 = dataObjectTour.createNestedArray("2");
    for(uint8_t i = 0; i < dataConfig.totalPresetOfTour2; i++){
        JsonObject& dataObjectNameMode2 = dataArrayMode2.createNestedObject();
        for(uint8_t j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            if(EEPROM.read(EEPROM_NUMBER_NAME_BEGIN_MODE_2 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j) == 0){
                break;
            }
            dataConfig.namePresetOfTour2[i] += char(EEPROM.read(EEPROM_NUMBER_NAME_BEGIN_MODE_2 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j));
        }
        dataObjectNameMode2["1"] = dataConfig.namePresetOfTour2[i];
        ECHO("Name: ");
        ECHO(dataConfig.namePresetOfTour2[i]);
        ECHO(" - ");

        dataConfig.timePresetOfTour2[i] = EEPROM.read(EEPROM_NUMBER_TIME_TOUR_MODE_2 + DISTANT_FROM_2_PRESET_OF_TOUR*i);
        dataObjectNameMode2["2"] = dataConfig.timePresetOfTour2[i];
        ECHO("Time: ");
        ECHO(dataConfig.timePresetOfTour2[i]);
        ECHO(" --- ");
    }
    ECHOLN("");

    ECHO("Mode 3: ");
    dataConfig.totalPresetOfTour3 = EEPROM.read(EEPROM_NUMBER_TOTAL_TOUR_MODE_3);
    if(dataConfig.totalPresetOfTour3 == 255){
        dataConfig.totalPresetOfTour3 = 0;
    }
    JsonArray& dataArrayMode3 = dataObjectTour.createNestedArray("3");
    for(uint8_t i = 0; i < dataConfig.totalPresetOfTour3; i++){
        JsonObject& dataObjectNameMode3 = dataArrayMode3.createNestedObject();
        for(uint8_t j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            if(EEPROM.read(EEPROM_NUMBER_NAME_BEGIN_MODE_3 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j) == 0){
                break;
            }
            dataConfig.namePresetOfTour3[i] += char(EEPROM.read(EEPROM_NUMBER_NAME_BEGIN_MODE_3 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j));
        }
        dataObjectNameMode3["1"] = dataConfig.namePresetOfTour3[i];
        ECHO("Name: ");
        ECHO(dataConfig.namePresetOfTour3[i]);
        ECHO(" - ");

        dataConfig.timePresetOfTour3[i] = EEPROM.read(EEPROM_NUMBER_TIME_TOUR_MODE_3 + DISTANT_FROM_2_PRESET_OF_TOUR*i);
        dataObjectNameMode3["2"] = dataConfig.timePresetOfTour3[i];
        ECHO("Time: ");
        ECHO(dataConfig.timePresetOfTour3[i]);
        ECHO(" --- ");
    }
    ECHOLN("");

    ECHO("Mode 4: ");
    dataConfig.totalPresetOfTour4 = EEPROM.read(EEPROM_NUMBER_TOTAL_TOUR_MODE_4);
    if(dataConfig.totalPresetOfTour4 == 255){
        dataConfig.totalPresetOfTour4 = 0;
    }
    JsonArray& dataArrayMode4 = dataObjectTour.createNestedArray("4");
    for(uint8_t i = 0; i < dataConfig.totalPresetOfTour4; i++){
        JsonObject& dataObjectNameMode4 = dataArrayMode4.createNestedObject();
        for(uint8_t j = 0; j < DISTANT_FROM_2_PRESET_OF_TOUR; j++){
            if(EEPROM.read(EEPROM_NUMBER_NAME_BEGIN_MODE_4 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j) == 0){
                break;
            }
            dataConfig.namePresetOfTour4[i] += char(EEPROM.read(EEPROM_NUMBER_NAME_BEGIN_MODE_4 + DISTANT_FROM_2_PRESET_OF_TOUR*i + j));
        }
        dataObjectNameMode4["1"] = dataConfig.namePresetOfTour4[i];
        ECHO("Name: ");
        ECHO(dataConfig.namePresetOfTour4[i]);
        ECHO(" - ");

        dataConfig.timePresetOfTour4[i] = EEPROM.read(EEPROM_NUMBER_TIME_TOUR_MODE_4 + DISTANT_FROM_2_PRESET_OF_TOUR*i);
        dataObjectNameMode4["2"] = dataConfig.timePresetOfTour4[i];
        ECHO("Time: ");
        ECHO(dataConfig.timePresetOfTour4[i]);
        ECHO(" --- ");
    }
    ECHOLN("");

    dataConfig.currentModeRun = EEPROM.read(EEPROM_CURRENT_MODE_RUN);
    dataConfig.previousModeRun = dataConfig.currentModeRun;
    ECHO("MODE RUN: ");
    ECHOLN(dataConfig.currentModeRun);
    if(dataConfig.currentModeRun == MODE_PRESET){
        for(uint8_t i = 0; i < DISTANT_FROM_2_PRESET_OF_TOUR; i++){
            if(EEPROM.read(EEPROM_CURRENT_NAME_MODE_PRESET_BEGIN + i) == 0){
                break;
            }
            dataConfig.nameOfModeRunPreset += char(EEPROM.read(EEPROM_CURRENT_NAME_MODE_PRESET_BEGIN + i));
        }
        ECHO("PRESET NAME: ");
        ECHOLN(dataConfig.nameOfModeRunPreset);
    }
    switch (dataConfig.currentModeRun)
    {
    case MODE_TOUR_1:
        dataObjectTour["5"] = "Mode 1";
        break;
    case MODE_TOUR_2:
        dataObjectTour["5"] = "Mode 2";
        break;
    case MODE_TOUR_3:
        dataObjectTour["5"] = "Mode 3";
        break;
    case MODE_TOUR_4:
        dataObjectTour["5"] = "Mode 4";
        break;
    case MODE_PRESET:
        dataObjectTour["5"] = dataConfig.nameOfModeRunPreset;
        break;
    default:
        break;
    }
    dataConfig.currentDataTour.clear();
    dataObjectTour.printTo(dataConfig.currentDataTour);
    ECHOLN(dataConfig.currentDataTour);
}

bool doneRunPreset(){
    for(int i = 0; i < MAX_SERVO; i++){
        if(dataConfig.statusCurrentServo[i] == SERVO_RUNNING){
            return false;
        }
    }
    return true;
}

void setupStartPositionServo(){
    for(int i = 0; i < MAX_SERVO; i++){
        dataConfig.value_old_pwm_servo[i] = (int)map(1500, PULSE_MS_SERVO_LOW, PULSE_MS_SERVO_HIGH, PWM_SERVO_LOW, PWM_SERVO_HIGH);
    }
    pwmController_1.setChannelPWM(PIN_SERVO_1, dataConfig.value_old_pwm_servo[SERVO_1]);
    pwmController_1.setChannelPWM(PIN_SERVO_2, dataConfig.value_old_pwm_servo[SERVO_2]);
    pwmController_1.setChannelPWM(PIN_SERVO_3, dataConfig.value_old_pwm_servo[SERVO_3]);
    pwmController_1.setChannelPWM(PIN_SERVO_4, dataConfig.value_old_pwm_servo[SERVO_4]);
    pwmController_1.setChannelPWM(PIN_SERVO_5, dataConfig.value_old_pwm_servo[SERVO_5]);
    pwmController_1.setChannelPWM(PIN_SERVO_6, dataConfig.value_old_pwm_servo[SERVO_6]);
    pwmController_1.setChannelPWM(PIN_SERVO_7, dataConfig.value_old_pwm_servo[SERVO_7]);
    pwmController_1.setChannelPWM(PIN_SERVO_8, dataConfig.value_old_pwm_servo[SERVO_8]);
    pwmController_1.setChannelPWM(PIN_SERVO_9, dataConfig.value_old_pwm_servo[SERVO_9]);
    pwmController_1.setChannelPWM(PIN_SERVO_10, dataConfig.value_old_pwm_servo[SERVO_10]);
    pwmController_1.setChannelPWM(PIN_SERVO_11, dataConfig.value_old_pwm_servo[SERVO_11]);
    pwmController_1.setChannelPWM(PIN_SERVO_12, dataConfig.value_old_pwm_servo[SERVO_12]);
    pwmController_2.setChannelPWM(PIN_SERVO_14, dataConfig.value_old_pwm_servo[SERVO_14]);
    pwmController_2.setChannelPWM(PIN_SERVO_15, dataConfig.value_old_pwm_servo[SERVO_15]);
    pwmController_2.setChannelPWM(PIN_SERVO_16, dataConfig.value_old_pwm_servo[SERVO_16]);
    pwmController_2.setChannelPWM(PIN_SERVO_17, dataConfig.value_old_pwm_servo[SERVO_17]);
    pwmController_2.setChannelPWM(PIN_SERVO_18, dataConfig.value_old_pwm_servo[SERVO_18]);
    pwmController_2.setChannelPWM(PIN_SERVO_19, dataConfig.value_old_pwm_servo[SERVO_19]);
    pwmController_2.setChannelPWM(PIN_SERVO_20, dataConfig.value_old_pwm_servo[SERVO_20]);

}

void controlServo1(void *pvParameters){
    for(;;){
        if(dataConfig.statusCurrentServo[SERVO_1] == SERVO_RUNNING){
            uint8_t time_delay = map(dataConfig.speedPreset[dataConfig.index_position_current_preset], MAX_PERCENT_SPEED_SERVO, MIN_PERCENT_SPEED_SERVO, MIN_DELAY_SPEED_SERVO, MAX_DELAY_SPEED_SERVO);
            // ECHOLN(time_delay);
            if(dataConfig.value_new_pwm_servo[SERVO_1] == dataConfig.value_old_pwm_servo[SERVO_1]){
                goto DONE_RUN_SERO_1;
            }
            if(dataConfig.speedPreset[dataConfig.index_position_current_preset] == MAX_PERCENT_SPEED_SERVO){
                xSemaphoreTake( xMutexI2C, portMAX_DELAY );
                pwmController_1.setChannelPWM(PIN_SERVO_1, dataConfig.value_new_pwm_servo[SERVO_1]);
                xSemaphoreGive( xMutexI2C );
                goto DONE_RUN_SERO_1;
            }
            if(dataConfig.value_new_pwm_servo[SERVO_1] >= dataConfig.value_old_pwm_servo[SERVO_1]){
                for(int i = dataConfig.value_old_pwm_servo[SERVO_1]; i <= dataConfig.value_new_pwm_servo[SERVO_1]; i+=5){
                    xSemaphoreTake( xMutexI2C, portMAX_DELAY );
                    pwmController_1.setChannelPWM(PIN_SERVO_1, i);
                    xSemaphoreGive( xMutexI2C );
                    if(dataConfig.currentModeRun != dataConfig.previousModeRun){
                        break;
                    }
                    vTaskDelay(time_delay/portTICK_RATE_MS);
                }
            }
            else{
                for(int i = dataConfig.value_old_pwm_servo[SERVO_1]; i >= dataConfig.value_new_pwm_servo[SERVO_1]; i-=5){
                    xSemaphoreTake( xMutexI2C, portMAX_DELAY );
                    pwmController_1.setChannelPWM(PIN_SERVO_1, i);
                    xSemaphoreGive( xMutexI2C );
                    if(dataConfig.currentModeRun != dataConfig.previousModeRun){
                        break;
                    }
                    vTaskDelay(time_delay/portTICK_RATE_MS);
                }
            }
DONE_RUN_SERO_1:
            dataConfig.value_old_pwm_servo[SERVO_1] = dataConfig.value_new_pwm_servo[SERVO_1];
            dataConfig.statusCurrentServo[SERVO_1] = SERVO_STOP;
        }

        vTaskDelay(10/portTICK_RATE_MS);
    }
}

void controlModeRun(void *pvParameters){
    for(;;){
        if(dataConfig.currentModeRun == MODE_PRESET){
            while(!doneRunPreset()){
               vTaskDelay(10/portTICK_RATE_MS);
            }
            dataConfig.previousModeRun = dataConfig.currentModeRun;
            DynamicJsonBuffer jsonBuffer(MAX_RESPONSE_LENGTH);
            JsonArray& JsonArrayPreset = jsonBuffer.parseArray(dataConfig.currentDataPreset);
            for(int i = 0; i < dataConfig.totalPreset; i++){
                String name = JsonArrayPreset[i]["1"].as<String>();
                if(dataConfig.nameOfModeRunPreset.equals(name)){
                    dataConfig.index_position_current_preset = i;
                    break;
                }
            }
        }
        for(int i = 0 ; i < MAX_SERVO; i++){
            dataConfig.value_new_pwm_servo[i] = dataConfig.anglePreset[dataConfig.index_position_current_preset][i];
            dataConfig.value_new_pwm_servo[i] = (int)map(dataConfig.value_new_pwm_servo[i], PULSE_MS_SERVO_LOW, PULSE_MS_SERVO_HIGH, PWM_SERVO_LOW, PWM_SERVO_HIGH);
            dataConfig.statusCurrentServo[i] = SERVO_RUNNING;
        }
        while(!doneRunPreset()){
            vTaskDelay(10/portTICK_RATE_MS); 
        }
        vTaskDelay(100/portTICK_RATE_MS);
    }
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
    digitalWrite(PIN_LED_R, LED_OFF);
    digitalWrite(PIN_LED_G, LED_OFF);
    digitalWrite(PIN_LED_B, LED_OFF);
	pca9685Init();
	bluetoothInit();
    loadDataFromEeprom();
    setupStartPositionServo();
    vTaskDelay(1000/portTICK_RATE_MS);
    xTaskCreatePinnedToCore(controlServo1,"controlServo1",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo2,"controlServo2",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo3,"controlServo3",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo4,"controlServo4",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo5,"controlServo5",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo6,"controlServo6",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo7,"controlServo7",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo8,"controlServo8",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo9,"controlServo9",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo10,"controlServo10",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo11,"controlServo11",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo12,"controlServo12",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo13,"controlServo13",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo14,"controlServo14",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo15,"controlServo15",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo16,"controlServo16",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo17,"controlServo17",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo18,"controlServo18",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo19,"controlServo19",1024,NULL,0,NULL,0);
    // xTaskCreatePinnedToCore(controlServo20,"controlServo20",1024,NULL,0,NULL,0);
    xTaskCreatePinnedToCore(controlModeRun,"controlModeRun",4096,NULL,0,NULL,0);


}

void loop() {
  	// put your main code here, to run repeatedly:
    if(APP_FLAG(SEND_DATA_PRESET)){
        APP_FLAG_CLEAR(SEND_DATA_PRESET);
        sendDataToApp(dataConfig.currentDataPreset + "#");
    }
    if(APP_FLAG(SEND_DATA_TOUR)){
        APP_FLAG_CLEAR(SEND_DATA_TOUR);
        sendDataToApp(dataConfig.currentDataTour + "%");
    }
    vTaskDelay(10/portTICK_RATE_MS);
}