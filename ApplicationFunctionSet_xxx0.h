/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-19 15:46:13
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <arduino.h>

// related functions and variable for obstacle-avoidance mode
class ApplicationFunctionSet
{
public:

  // initialize
  void ApplicationFunctionSet_Init(void);

  // see diagram for details
  void ApplicationFunctionSet_Obstacle(void);    

  void TestFunction (void);       
  
private:
  volatile uint16_t UltrasoundData_mm; 
  volatile uint16_t UltrasoundData_cm; 
  boolean UltrasoundDetectionStatus = false;

public:
  boolean Car_LeaveTheGround = true;
  const int ObstacleDetection = 20;
};

extern ApplicationFunctionSet Application_FunctionSet;
#endif
