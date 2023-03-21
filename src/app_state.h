#ifndef __APP_H
#define __APP_H

int app_flags = 0;
int app_flags_custom = 0;

/**
 * @name Flags for application
 * @{
 */
#define MODE_CONFIG               0x01          //config via bluetooh           --  LED BLUE On
#define MODE_WAIT_RUNNING         0x02          //Wait user control on or off   --  LED GREEN Blink 200ms
#define MODE_RUNNING              0x04          //RUNING                        -- LED GREEN On
#define SEND_DATA_DEFINE_CURRENT  0x08
#define SEND_DATA_STEP            0x10
#define SEND_CURRENT_MIN_MAX      0x20
#define SEND_DATA_VOLTAGE         0x40

/* Exported macros -----------------------------------------------------------*/
#define APP_FLAG(flag) (app_flags & flag)

#define APP_FLAG_SET(flag) (app_flags |= flag)
#define APP_FLAG_CLEAR(flag) (app_flags &= ~flag)

#define APP_FLAG_CUSTOM(flag) (app_flags_custom & flag)

#define APP_FLAG_SET_CUSTOM(flag) (app_flags_custom |= flag)
#define APP_FLAG_CLEAR_CUSTOM(flag) (app_flags_custom &= ~flag)


#endif /*__APP_H */