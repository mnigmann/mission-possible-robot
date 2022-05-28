#ifndef CONFIG_FILE

#define MOTOR_LEFT      17
#define MOTOR_RIGHT     23

#define SERVO_AZIMUTH   24
#define SERVO_ELEVATION 18

#define MOTOR_SHOVEL    15  // RXD
#define MOTOR_ARM       14  // TXD
#define MOTOR_FLAG      10  

#define MOTOR_ARM_ONTIME    1500000
#define MOTOR_SHOVEL_ONTIME 400000
#define MOTOR_SHOVEL_SPEED  50
#define MOTOR_FLAG_ONTIME   400000
#define MOTOR_FLAG_SPEED    -50

#define ENC_CLK         2
#define ENC_DIR         3
#define ENC_OFFMAX      10
#define ENC_LOWMAX      40
#define ENC_MEDMAX      60
#define ENC_INTSIZE     0.5
#define ENC_NUMINT      16

#define UI_SU           0       // Position of the "Subsystem" column
#define UI_ST           48      // Position of the "State" column
#define UI_VA           64      // Position of the "Value" column

#define CONFIG_FILE
#endif
