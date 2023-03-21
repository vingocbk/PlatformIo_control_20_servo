#ifndef CONFIG_H
#define CONFIG_H

#define SERIAL_BAUDRATE     115200

// #define SCL_PIN             22
// #define SDA_PIN             21

// #define BTN_IN_M1           4
// #define BTN_IN_M2           5
// #define BTN_IN_M3           34
// #define BTN_IN_M4           15
// #define BTN_IN_M5           35
// #define BTN_IN_M6           32
// #define BTN_IN_M7           33
// #define BTN_IN_M8           26
// #define BTN_IN_M9           0
// #define BTN_IN_M10          2
// #define BTN_IN_LED_1        12
// // #define BTN_IN_LED_2        25  //27
// #define BTN_IN_LED_2        27  //design

// #define BTN_MODE_SETUP      23
// // #define BTN_MODE_RUN        27  //25
// #define BTN_MODE_RUN        25  //design

// #define DATA_PIN_LED        16
// #define LATCH_PIN_LED       13  
// #define CLOCK_PIN_LED       14

// #define DATA_PIN_MOTOR      19
// #define LATCH_PIN_MOTOR     18      //17
// #define CLOCK_PIN_MOTOR     17      //18

//new version
#define SCL_PIN             22
#define SDA_PIN             21

// #define BTN_IN_LED_2        25  //27
#define BTN_IN_LED_2        27  //design
// #define INPUT_VOLTAGE       39  //SENSOR_VN
#define INPUT_VOLTAGE       2

#define BTN_MODE_SETUP      23
// #define BTN_MODE_RUN        27  //25
#define BTN_MODE_RUN        25  //design

#define ADDRESS_PCA9685_1         0x61    //97
#define ADDRESS_PCA9685_2         0x62    //98

#define MAX_RESPONSE_LENGTH     2048

#define MAX_SIZE_EEPROM_BUFFER      4096
#define EEPROM_MIN_CURRENT_1        11
#define EEPROM_REVERSE_MOTOR_1      21
#define EEPROM_OPEN_STEP_1_MTOR_1   31
#define EEPROM_OPEN_STEP_2_MTOR_1   41
#define EEPROM_OPEN_STEP_3_MTOR_1   51
#define EEPROM_CLOSE_STEP_1_MTOR_1  61
#define EEPROM_CLOSE_STEP_2_MTOR_1  71
#define EEPROM_CLOSE_STEP_3_MTOR_1  81
#define EEPROM_SELECT_MOTOR_1       91
#define EEPROM_SET_VOLTAGE_MOTOR_1  101
#define EEPROM_SELECT_SERVO_1       111
#define EEPROM_START_ANGLE_SERVO_1  121         //2 byte for each variable
#define EEPROM_END_ANGLE_SERVO_1    141         //2 byte for each variable
#define EEPROM_TIME_SERVO_1         161         //2 byte for each variable
#define EEPROM_MAX_CURRENT_1        181         //2 byte for each variable


// #define VALUE_CONVERT           10
#define MIN_CURRENT_MOTOR_CHECK_START       10          //mA


#define MIN_VALUE_READ_RX       500
#define MAX_VALUE_READ_RX       2500
#define ON_LED      true
#define OFF_LED     false

#define CONFIG_HOLD_TIME        3000
#define TIME_OUT_PULSEIN        50000
#define COUNT_READ_PULSEIN      5
// #define TIME_OUT_PULSEIN        5




#endif
