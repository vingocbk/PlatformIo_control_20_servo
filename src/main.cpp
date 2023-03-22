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

String dataSendPreset = "[{\"1\":\"ngoc\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"tuyet\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"tuan\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hcchu\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhy\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"gxyxy\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhx\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"t8t7ry\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},{\"1\":\"hxhxucupc\",\"2\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}]#";
String dataSendTour = "{\"1\":[{\"1\":\"ngoc\",\"2\":50},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"2\":[{\"1\":\"tuyet\",\"2\":40},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"3\":[{\"1\":\"tuan\",\"2\":50},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"4\":[{\"1\":\"tuan\",\"2\":5},{\"1\":\"ngoc\",\"2\":128},{\"1\":\"ngoc\",\"2\":128}],\"5\":[3,1]}%";
void pca9685Init();
void bluetoothInit();
void callbackBluetooth(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void readPulseInMode3(void *pvParameters);
void sendDataToApp(String data);


void pca9685Init(){
	pwmController_1.resetDevices();       // Resets all PCA9685 devices on i2c line
    pwmController_1.init();               // Initializes module using default totem-pole driver mode, and default disabled phase balancer
    pwmController_1.setPWMFreqServo();    // 50Hz provides standard 20ms servo phase length

	pwmController_2.resetDevices();       // Resets all PCA9685 devices on i2c line
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
                    sendDataToApp(dataSendPreset);
                }
                else if(type == "sync_tour"){
                    sendDataToApp(dataSendTour);
                }
                else if(type == "save_preset"){
                    
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


void readPulseInMode3(void *pvParameters){
    bool check_start_connect_mode_run = false;
    unsigned long check_time_begin;
	for( ;; )
	{
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

void sendDataToApp(String data){
    ECHOLN("sendDataToApp");
    for(int i = 0; i < data.length(); i++){
        SerialBT.write(data[i]);
    }
}


void setup() {
	// put your setup code here, to run once:
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //Brownout detector was triggered
    Serial.begin (SERIAL_BAUDRATE);  
    Wire.begin (SDA_PIN, SCL_PIN);   // sda= GPIO_21 /scl= GPIO_22
    EEPROM.begin(MAX_SIZE_EEPROM_BUFFER);
	// pca9685Init();
	bluetoothInit();
    pinMode(0, INPUT_PULLUP);


	xTaskCreatePinnedToCore(
		readPulseInMode3,    /* Function to implement the task */
		"readPulseInMode3",  /* Name of the task */
		4096,             /* Stack size in words */
		NULL,             /* Task input parameter */
		1,                /* Priority of the task */
		NULL,             /* Task handle. */
		1);               /* Core where the task should run */
}

void loop() {
  	// put your main code here, to run repeatedly:
    if(digitalRead(0) == LOW){
        vTaskDelay(500/portTICK_RATE_MS);
        sendDataToApp(dataSendPreset);
    }
}