/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 14:45:27
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#ifndef _DeviceDriverSet_xxx0_H_
#define _DeviceDriverSet_xxx0_H_
#include <arduino.h>

/* Motor Class */
// class definition for a servo object (created in DeviceDriverSet_xxx0.cpp)
class DeviceDriverSet_Motor
{
public:
  void DeviceDriverSet_Motor_Init(void);  // 

#if _Test_DeviceDriverSet
  void DeviceDriverSet_Motor_Test(void);  // 

#endif
  void DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //A组电机参数
                                     boolean direction_B, uint8_t speed_B, //B组电机参数
                                     boolean controlED                     //AB使能允许 true
  );                                                                       //电机控制
private:
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

public:
#define speed_Max 255
#define direction_just true
#define direction_back false
#define direction_void 3

#define Duration_enable true
#define Duration_disable false
#define control_enable true
#define control_disable false
};

//#include <NewPing.h>    // Elegoo's commented out code

/* ULTRASONIC Class*/
class DeviceDriverSet_ULTRASONIC
{
// these functions are defined in DeviceDriverSet_xxx0.cpp
public:
  void DeviceDriverSet_ULTRASONIC_Init(void); // Initialize
  
#if _Test_DeviceDriverSet
  void DeviceDriverSet_ULTRASONIC_Test(void); // Test calibration (Lab 6 Part 1)

#endif
  void DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get);  // Transmitter output pulse 

private:
#define TRIG_PIN 13      // Arduino pin 13 tied to trigger (transmitter) pin on the ultrasonic sensor (output)
#define ECHO_PIN 12      // Arduino pin 12 tied to echo (receiver) pin on the ultrasonic sensor (input)
#define MAX_DISTANCE 200 // Maximum distance we want to ping for in centimeters (maximum sensor distance is rated at 400cm)
};

/* Servo Class */
#include <Servo.h>
class DeviceDriverSet_Servo
{
public:
  void DeviceDriverSet_Servo_Init(unsigned int Position_angle);

#if _Test_DeviceDriverSet
  void DeviceDriverSet_Servo_Test(void);

#endif
  void DeviceDriverSet_Servo_control(unsigned int Position_angle);

private:
#define PIN_Servo_z 10    // Arduino pin 10 tied to servo motor (makes the ultrasonic sensor swivel)
};

#endif
