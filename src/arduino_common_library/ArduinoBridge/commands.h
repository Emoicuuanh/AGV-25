/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

// clang-format off
#define ANALOG_READ      'a'
#define GET_BAUDRATE     'b'
#define PIN_MODE         'c'
#define DIGITAL_READ     'd'
#define READ_ENCODERS    'e'
#define MOTOR_SPEEDS     'm'
#define CMD_VEL          'v'
#define PING_TEST        'p'
#define RESET_ENCODERS   'r'
#define MODULE_SETUP     's'
#define TOWER_LAMP       't'
#define READ_ULTRASONIC  'u'
#define DIGITAL_WRITE    'w'
#define ANALOG_WRITE     'x'
#define LOG_CONFIRM      'f'
#define INT_16_WRITE     'i'
#define LED_WRITE        'l'
#define LED_RGB          'n'
#define CARD_ID          'k'
#define LIFT             'g'
#define READ_PARAMETER   'z'
#define TURN_SIGNAL      'y'
#define CONVEYOR         'o'
#define STOPPER          'h'
// clang-format on

#endif
