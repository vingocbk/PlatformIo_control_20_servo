#ifndef CONFIG_H
#define CONFIG_H

#define SERIAL_BAUDRATE     115200

#define SCL_PIN             22
#define SDA_PIN             21    
#define PIN_LED_R           27
#define PIN_LED_G           32
#define PIN_LED_B           26

#define LED_ON              LOW
#define LED_OFF             HIGH

#define ADDRESS_PCA9685_1         0x61    //97
#define ADDRESS_PCA9685_2         0x71    //113

#define MAX_RESPONSE_LENGTH     4096

#define MAX_SIZE_EEPROM_BUFFER              4096
#define EEPROM_NUMBER_TOTAL_PRESET          0
#define EEPROM_NAME_PRESET_BEGIN            1
#define EEPROM_SPEED_PRESET_BEGIN           30
#define EEPROM_ANGLE_PRESET_BEGIN           31

#define EEPROM_NUMBER_TOTAL_TOUR_MODE_1     2000
#define EEPROM_NUMBER_NAME_BEGIN_MODE_1     2001

#define EEPROM_NUMBER_TOTAL_TOUR_MODE_2     2500
#define EEPROM_NUMBER_NAME_BEGIN_MODE_2     2501

#define EEPROM_NUMBER_TOTAL_TOUR_MODE_3     3000
#define EEPROM_NUMBER_NAME_BEGIN_MODE_3     3001

#define EEPROM_NUMBER_TOTAL_TOUR_MODE_4     3500
#define EEPROM_NUMBER_NAME_BEGIN_MODE_4     3501

#define EEPROM_OPEN_TOUR_MODE_5             4000
#define EEPROM_CLOSE_TOUR_MODE_5            4001

#define DISTANT_FROM_2_PRESET               50
#define DISTANT_FROM_2_NAME_OF_PRESET_TOUR  20

#define MIN_VALUE_READ_RX       500
#define MAX_VALUE_READ_RX       2500

#define CONFIG_HOLD_TIME        3000
#define TIME_OUT_PULSEIN        50000
#define COUNT_READ_PULSEIN      5
// #define TIME_OUT_PULSEIN        5

#define PIN_SERVO_1                 0
#define PIN_SERVO_2                 1
#define PIN_SERVO_3                 2
#define PIN_SERVO_4                 3
#define PIN_SERVO_5                 4
#define PIN_SERVO_6                 5
#define PIN_SERVO_7                 6
#define PIN_SERVO_8                 7
#define PIN_SERVO_9                 11
#define PIN_SERVO_10                10
#define PIN_SERVO_11                9
#define PIN_SERVO_12                8
#define PIN_SERVO_13                0
#define PIN_SERVO_14                1
#define PIN_SERVO_15                2
#define PIN_SERVO_16                3
#define PIN_SERVO_17                4
#define PIN_SERVO_18                5
#define PIN_SERVO_19                6
#define PIN_SERVO_20                7


#endif
