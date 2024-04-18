#ifndef SERIAL_MESSAGE_H
#define SERIAL_MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#define SERIAL_MSG_MOTION_CONTROL_ID ((uint8_t)0xE0)
#define SERIAL_MSG_STATE_DETECT_ID ((uint8_t)0xED)
#define SERIAL_MSG_STATE_FEELBACK_ID ((uint8_t)0xEE)
#define ENABLE_NULL ((uint8_t)0b00000000)
#define ENABLE_A ((uint8_t)0b00000010)
#define ENABLE_B ((uint8_t)0b00000001)
#define ENABLE_AB ((uint8_t)0b00000011)
#define ZERO ((uint8_t)0x00)

/***************** Control messages *****************/

typedef struct {
  float linear_velocity;
  float angular_velocity;
  float lateral_velocity;
  float steering_angle;
} MotionCommandMessage;

/**************** Feedback messages *****************/
#define DETECT_SYSTEM_STATE_ID ((uint8_t)0x00)
#define DETECT_MOTOR_ANGLE_MAG_ID ((uint8_t)0x01)
#define DETECT_MOTOR_SPPED_ID ((uint8_t)0x02)
#define DETECT_MOTOR_CURRENT_ID ((uint8_t)0x03)
#define DETECT_MOTOR_ANGLE_MECH_ID ((uint8_t)0x04)
#define DETECT_POWER_VOLTAGE_ID ((uint8_t)0x05)
#define DETECT_TEMPETRUE_ID ((uint8_t)0x07)
#define DETECT_ERROR_STATE_ID ((uint8_t)0x08)
#define DETECT_MOTOR_POSITION_ID ((uint8_t)0x09)

#define ENCODER_MASK ((uint8_t)0x01)
#define HALL_MASK ((uint8_t)0x02)
#define MAG_ENCODER_MASK ((uint8_t)0x03)
#define SSI_MASK ((uint8_t)0x04)
#define ROTATE_MASK ((uint8_t)0x05)
#define HALL_ENCODER_OPEN_MASK ((uint8_t)0x07)
#define HALL_CLOSE_MASK ((uint8_t)0x08)
#define HALL_ENCODER_CLOSE_Z_MASK ((uint8_t)0x09)
#define HALL_ENCODER_CLOSE_U_MASK ((uint8_t)0x0A)
#define ABSOLUTE_ENCODER_MASK ((uint8_t)0x0B)
#define MODE_A_MASK ((uint8_t)0x10)
#define MODE_CAN_MASK ((uint8_t)0x20)
#define MODE_SCI_MASK ((uint8_t)0x30)
#define MODE_RC_MASK ((uint8_t)0x40)

typedef struct {
  uint8_t state_code_high;
  uint8_t state_code_low;
  uint8_t error_code_high_left;
  uint8_t error_code_low_left;
  uint8_t error_code_high_right;
  uint8_t error_code_low_right;
  uint8_t controller_t;
  uint8_t motor_left_t;
  uint8_t motor_right_t;
} SystemStateMessage;

#define MODE_SET_ERROR_MASK ((uint8_t)0x01)
#define HYBRID_DRIVE_ERROR_MASK ((uint8_t)0x02)
#define HOT_ERROR_MASK ((uint8_t)0x04)
#define HALL_ERROR_MASK ((uint8_t)0x08)
#define CURRRNT_SENSOR_ERROR_MASK ((uint8_t)0x10)
#define MOTOR_HOT_ERROR_MASK ((uint8_t)0x20)
#define CAN_ERROR_MASK ((uint8_t)0x40)
#define SCI_ERROR_MASK ((uint8_t)0x80)

typedef struct {
  float angle_left;
  float angle_right;
} MotorAngleMessage;

typedef struct {
  float speed_left;
  float speed_right;
} MotorSpeedMessage;

typedef struct {
  // command
  MotionCommandMessage motion_command_msg;
  SystemStateMessage system_state_msg;
  MotorAngleMessage motor_angle_msg;
  MotorSpeedMessage motor_speed_msg;
} RecMessage;
#ifdef __cplusplus
}
#endif

#endif
