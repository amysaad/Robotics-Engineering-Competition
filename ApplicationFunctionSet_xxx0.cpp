/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;               // defined in DeviceDriverSet_xxx0.cpp

static boolean
// if: s <= x <= e
function_xxx(long x, long s, long e)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

// the 9 possible directions
enum SmartRobotCarMotionControl
{
  Forward,       //(0)
  Backward,      //(1)
  Left,          //(2)
  Right,         //(3)
  LeftForward,   //(4)
  LeftBackward,  //(5)
  RightForward,  //(6)
  RightBackward, //(7)
  stop_it        //(8)
};

enum SmartRobotCarFunctionalModel
{
  Standby_mode,           /*空闲模式*/
  TraceBased_mode,        /*循迹模式*/
  ObstacleAvoidance_mode, /*避障模式*/
  Follow_mode,            /*跟随模式*/
  Rocker_mode,            /*摇杆模式*/
};

struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};

Application_xxx Application_SmartRobotCarxxx0;;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);


// this is called in Demo2.ino under setup():
void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  // a loop that does nothing...?
  while (Serial.read() >= 0)
  {
    /*清空串口缓存...*/    // Elegoo commented out code
  }
  delay(2000);  // 2 second delay
  Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;   // this will allow the ApplicationFunctionSet_Obstacle(void) function's code to run
}

// motor adjustment calculations based on yaw (z-axis) data from MPU6050 
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; 
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;

  // possibly an odd way to periodically get yaw data (every 10 seconds or whenever the robot changes direction)?
  // first half of OR = directionRecord is not equal to previous directionRecord, second half of OR = a counter to count if 10 milliseconds has ellapsed
  if (en != directionRecord || millis() - is_time > 10)
  {
    // every ten milliseconds, or when directionRecord is not equal to 110 (example: if direction = forward, directionRecord will be set to 2):
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable); // Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);  
    
    is_time = millis(); // count the number of milliseconds passed since the Arduino board began running the current program
  }

  // if new direction or car on ground?
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }

  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }

  if (direction == Forward)
  {
    // R is speed for motor A, L is speed for motor B
    AppMotor.DeviceDriverSet_Motor_control(direction_just, R, direction_just, L, control_enable);
  }
  else if (direction == Backward)
  {
    // L is speed for motor A, R is speed for motor B
    AppMotor.DeviceDriverSet_Motor_control(direction_back, L, direction_back, R, control_enable);
  }
}

// Motor Control
// direction: (instead of numbers, use the variable names now like Forward, Backward, stop_it, etc.), speed: (0 - 255)
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;

  Kp = 2;
  UpperLimit = 180;

  switch (direction)
  {
  case Forward:
    // could probably comment out / carefully delete all code beloning to TraceBased_mode
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_just, speed, control_enable); // Motor control A, B
    }
    // for obstacle mode
    else
    {
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }
    break;

  case Backward:
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_back, speed, control_enable); 
    }
    else
    {
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }
    break;

  case Left:
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_back, speed, control_enable); 
    break;

  case Right:
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_just, speed, control_enable); 
    break;
    
  case LeftForward:
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_just, speed / 2, control_enable);
    break;

  case LeftBackward:
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_back, speed / 2, control_enable); 
    break;
    
  case RightForward:
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(direction_just, speed / 2, direction_just, speed, control_enable);
    break;
    
  case RightBackward:
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(direction_back, speed / 2, direction_back, speed, control_enable); 
    break;

  case stop_it:
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
    break;
    
  default:
    directionRecord = 10;
    break;
  }
}

/*
// this can just be replaced by a simple command "delay(_ms)" everywhere delay_xxx(_ms) is called
static void delay_xxx(uint16_t _ms)
{
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}
*/


// -------------------------- THIS IS THE FUNCTION CALLED IN LOOP --------------------------------------------------------------------------
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{
  // this boolean determines if the robot has entered obstacle avoidance mode yet 
  // (true = yes, first time, false = no it has already been in this mode since startup)
  static boolean first_is = true;

  // ObstacleAvoidance_mode set on line 95 above - this function WILL run in this demo
  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    uint8_t switc_ctrl = 0;   // seems to be unused in this demo
    uint16_t get_Distance;

    // code looks like: if car has not been picked up (is on the ground), then start with the motors turned off
    // but it does not turn the car off when it is picked up...but when set equal to true, the robot never moves (on or off ground)
    // TLDR; recommend leaving this if statement alone
    if (Car_LeaveTheGround == false)
    {
      // function defined above in this file: direction = 8 (stop), speed = 0
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }

    // robot is entering the obstacle avoidance mode for the first time
    // turn the servo motor back to 90 degrees (center position)
    if (first_is == true) 
    {
      // a DeviceDriverSet_Servo object
      AppServo.DeviceDriverSet_Servo_control(90);  // change position_angle to center
      first_is = false; // no longer the first time in obstacle avoidance mode
    }

    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance);  // get distance from ultrasonic
    Serial.println(get_Distance); // show distance in Serial Monitor (we will only see this if the USB cable is still attached while running)

    // OUTTER IF:
    // IF DISTANCE < 20 cm
    if (function_xxx(get_Distance, 0, 18)) // if: 0 <= get_Distance <= 20
    {
      // STOP
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

      // go from 1 to 5, step 2: 1, 3, 5
      for (int i = 1; i < 6; i += 2) //1、3、5 Omnidirectional detection of obstacle avoidance status
      {
        // change position_angle to 30 degrees (that's 60 degrees to the right) multiplied by loop iteration #
        AppServo.DeviceDriverSet_Servo_control(10 * i);   // i=1 swivel 60 deg right, i=3 swivel to center, i=5 swivel 60 deg left
        delay(1); // delay 1 millisecond // original: delay_xxx(1);

        // after changing servo position, check for obstacles again (get distance input data)
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance);

        // INNER IF:
        if (function_xxx(get_Distance, 0, 18))  // if: 0 <= get_Distance <= 20 (IF DISTANCE < 20 cm)
        {
          // STOP
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

          // on the last loop iteration          
          if (5 == i)
          {
            ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 100);
            delay(500); // original: delay_xxx(500);
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
            delay(50);  // original delay_xxx(50);
            first_is = true;
            break;  // unecessary
          }
        }
        // distance > 20 cm
        else
        {
          switc_ctrl = 0;  // seems to be unused in this demo

          // switch case for i value in for-loop (1, 3, or 5)
          switch (i)
          {
          case 1:
            // change direction to right at speed=150
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
            break; // unecessary
          case 3:
            // change direction to forward at speed=150
            ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
            break; // unecessary
          case 5:
            // change direction to left at speed=150
            ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
            break; // unecessary
          }

          delay(50);  // original: delay_xxx(50);
          first_is = true;  // this will allow the sensor to reset its position to center the next time this function is called
          break;
        }
      }
    }
    
    // if: get_Distance > 20 cm
    else //if (function_xxx(get_Distance, 20, 50))    // Elegoo commented out code
    {
      // if the obstacle is further than 20cm, keep going forward
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
    }

  }
  
  else
  {
    first_is = true;
  }
}
/* This code was for Test 1

void ApplicationFunctionSet::TestFunction(void){
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
  delay(5000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 100);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 150);
  delay(5000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 100);
  delay(1000);
  
}
*/
/* This code was for Test 2
void ApplicationFunctionSet::TestFunction(void)
{
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 110);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 120);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 130);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 140);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 140);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 130);
  delay(500);  
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 120);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 110);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 100);
}
*/
/* This code was for Test 3
void ApplicationFunctionSet::TestFunction(void){
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(LeftForward, 100);
  delay(3000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(RightForward, 100);
  delay(3000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 100);
}
*/
/*This code was for Test 4
void ApplicationFunctionSet::TestFunction(void){
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 100);
}
*/
