/*
 * DrRobotCommConst.hpp
 *
 *  Created on: Mar 24, 2011
 *      Author: dri
 */

#ifndef DRROBOTCOMMCONST_HPP_
#define DRROBOTCOMMCONST_HPP_

//here is some parameters in communication protocol, please consult the Dr Robot API document
  const uint32_t MAX_AD_READING = 4095;

  // Likely stands for start transmission/exchange
  // These appear to be some kind of
  // standard header value for error checking
  const unsigned char COM_STX0 = 0x5E;
  const unsigned char COM_STX1 = 0x02;

  // These two define which location,
  // PC or Motorola board, that the message is for.
  const unsigned char COM_TYPE_PC = 0x00;
  const unsigned char COM_TYPE_MOT = 0x01;
  // ETX likely refers to end transmission/exchange
  // However, these seem to only be used as standard
  // tail values for error validation.
  const unsigned char COM_ETX0 = 0x5E;
  const unsigned char COM_ETX1 = 0x0D;    
  
  // Only used when sending single motor/server commands.
  // Not required.  Likely a flag to execute the sent command
  // for a certain duration.
  const unsigned char TIMEFLAG = 28;
  
  // Appears to be a flag for individual,
  // position based motor control.
  const unsigned char MOTORPOSITIONCTRL = 3;
  
  // Appears to be a flag for position
  // based control of all motors.
  const unsigned char MOTORPOSITIONCTRLALL = 4;

  // Appears to be a flag for individual,
  // PWM based motor control.
  const unsigned char MOTORPWMCTRL = 5;
  // Appears to be a flag for PWM
  // based control of all motors.
  const unsigned char MOTORPWMCTRLALL = 6;

  // Appears to be a flag for individual,
  // velocity based motor control.
  const unsigned char MOTORVELOCITYCTRL = 26;
  // Appears to be a flag for velocity
  // based control of all motors.
  const unsigned char MOTORVELOCITYCTRLALL = 27;  

  // Flag that the message is adjusting a parameter
  const unsigned char MOTORPARAMETERSETTING = 7;
  
  // These determine whether the paramater
  // is a position or velocity pid value.
  const unsigned char POSITIONPID = 7;
  const unsigned char VELOCITYPID = 8;

  // Appears to be a flag for
  // individual servo control.
  const unsigned char SERVOCTRL = 28;
  // Appears to be a flag for servo
  // control over all servos.
  const unsigned char SERVOCTRLALL = 29;
  
  // Flag detailing that the message is
  // for disabling a motor/servo.
  const unsigned char MOTORENABLE = 0x1e;

  // Flag detailing that the message is
  // for changing motor friction compensation.
  // Seems function sends for all motors.
  const unsigned char MOTORFRICCOMP = 31;
  
  // Flag for interaction with IO ports
  const unsigned char CUSTOMIO = 22;

  // ID's for pid parameters
  const unsigned char KP_ID = 1;
  const unsigned char KD_ID = 2;
  const unsigned char KI_ID = 3;

  // Number of failed checks for message
  const int  COMM_LOST_TH  = 200; // 10ms * 200 = 2s
  // Time to wait between checks for message
  const int  COMM_SLEEP_TM =  10000;   //10ms

  // Default connection port
  #define DEFAULT_PORT 10001
  // Index of msg where STX0 is located.
  #define INDEX_STX0              0

  // Index of msg where STX1 is located.
  #define INDEX_STX1              1

  // Index of msg where destination flag is located.
  #define INDEX_DES               2

  // Seems unused...
  // Index three is always zero anyway...
  #define INDEX_SN                3

  // Index of msg where msg type is located.
  #define INDEX_TYPE              4

  // Index of msg where lenght of data in msg is located.
  #define INDEX_LENGTH            5

  // Index of msg where data begins.
  #define INDEX_DATA              6

  // msg types
  // At index INDEX_TYPE
  #define COMTYPE_SYSTEM          0xFF
  #define COMTYPE_SENSOR          0x7F
  #define COMTYPE_MOTOR           40 // Why u no hex!?
  #define COMTYPE_MOTOR_SENSOR     0x7B
  #define COMTYPE_CUSTOM_SENSOR    0x7C
  #define COMTYPE_STANDARD_SENSOR   0x7D

  // valid data option for COMTYPE_SYSTEM messages.
  #define DATA_ACK                        0x01
  #define DATA_PING                       0x00
  #define DATA_URGENT_DATA        0x02
  #define DATA_SKIPPACKET         0x03

#endif /* DRROBOTCOMMCONST_HPP_ */
