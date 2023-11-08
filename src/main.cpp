/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Student                                          */
/*    Created:      Fri Oct 27 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/* ---- START VEXCODE CONFIGURED DEVICES ----
Robot Configuration:
[Name]               [Type]        [Port(s)]
right_drive_motor     motor           10
left_drive_motor      motor            1
cam_motor             motor            7
arm_motor             motor            8
claw_motor            motor            3
con1                controller        N/A
VisionSensor         sensor            2
DistanceSensor       sensor            5
gps                   gps              4
InertialSensor       sensor            9      
 ---- END VEXCODE CONFIGURED DEVICES ---- */

/*
up arrow - move cam up
left arrow - move cam down
x - drive speed +25%
y - drive speed -25%
r1 - drive speed right stick
l1 - arm speed right stick
r2 - drive 50% speed
l2 - move arm up 50% speed
left x - turn 50% speed
down arrow - arm down 50% speed
b - reverse 50% speed
a - close claw for .5 sec (press again resets to fully open)
right arrow - close claw
*/


/*
set motor pos 3 to 0 (fully open)
a - close claw for .5 sec (press again resets to pos 0)
*/

#include "vex.h"
#include <algorithm>
#include <cmath>

using namespace vex;

vex::motor      right_drive_motor(vex::PORT10, vex::gearSetting::ratio18_1, false);
vex::motor      left_drive_motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor      cam_motor(vex::PORT7, vex::gearSetting::ratio18_1, false);
vex::motor      arm_motor(vex::PORT8, vex::gearSetting::ratio36_1, false);
vex::motor      claw_motor(vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::controller con1(vex::controllerType::primary);
vex::vision     VisionSensor(vex::PORT2);
vex::competition Competition;
vex::distance DistanceSensor(vex::PORT5);
vex::gps        gps(vex::PORT4, 0, turnType::right);
vex::inertial   InertialSensor(vex::PORT9);
//vex::rotation   RotationSensor(vex::PORT4);


void driving(void) {
  cam_motor.setVelocity(50, velocityUnits::pct);
  claw_motor.setVelocity(50, velocityUnits::pct);
  arm_motor.setBrake(hold);
  claw_motor.setBrake(hold);

  claw_motor.spinFor(directionType::rev, 0.5, sec);
  task::sleep(500);
  double clawOpen = claw_motor.position(rotationUnits::deg);

  double drivePctSpeed = 50;
  double drivePctTemp = 50;
  double armPctSpeed = 50;

  double leftDriveSpeed = 0;
  double rightDriveSpeed = 0;

  while(true) {

    con1.Screen.print(claw_motor.position(rotationUnits::deg));

    // camera
    if (con1.ButtonUp.pressing()) {
      cam_motor.spin(directionType::rev);
    } else if (con1.ButtonLeft.pressing()) {
      cam_motor.spin(directionType::fwd);
    } else {
      cam_motor.stop();
    }

    // claw
    if (con1.ButtonA.pressing()) {
      if (claw_motor.position(rotationUnits::deg) <= clawOpen) {
        claw_motor.spinFor(directionType::fwd, 0.5, sec);
      } else {
        // claw_motor.spinFor(directionType::rev, 0.1, sec);
        claw_motor.spinToPosition(clawOpen, rotationUnits::deg, true);
      }
    } else if (con1.ButtonRight.pressing()) {
      claw_motor.spin(fwd);
    } else {
      claw_motor.stop();
    }

    if(con1.ButtonX.pressing() && drivePctTemp < 100) {
      drivePctTemp += 25;
      task::sleep(300);     
    }
    if(con1.ButtonY.pressing() && drivePctTemp > 25) {
      drivePctTemp -= 25;
      task::sleep(300);
    }
    
    int leftX = con1.Axis4.position(pct);
    int leftY = con1.Axis3.position(pct);
    int rightX = con1.Axis1.position(pct);
    int rightY = con1.Axis2.position(pct);

    if(con1.ButtonR1.pressing() && !con1.ButtonL1.pressing()) {
      drivePctSpeed = rightY;
      armPctSpeed = 50.0;
    } else if (con1.ButtonL1.pressing() && !con1.ButtonR1.pressing()) {
      armPctSpeed = rightY;
      drivePctSpeed = drivePctTemp;
    } else if (con1.ButtonR1.pressing() && con1.ButtonL1.pressing()) {
      drivePctSpeed = 0.0;
      armPctSpeed = 0.0;
    } else {
      drivePctSpeed = drivePctTemp;
      armPctSpeed = 50.0;
    }


    if(con1.ButtonR2.pressing()) {
      if(leftX > 0){
        leftDriveSpeed = drivePctSpeed + leftX;
        rightDriveSpeed = drivePctSpeed - leftX*M_PI_2;
      } else if (leftX < 0) {
        leftDriveSpeed = drivePctSpeed + leftX*M_PI_2;
        rightDriveSpeed = drivePctSpeed - leftX;
      } else {
        leftDriveSpeed = drivePctSpeed;
        rightDriveSpeed = drivePctSpeed;
      }
      left_drive_motor.spin(directionType::fwd, leftDriveSpeed, velocityUnits::pct);
      right_drive_motor.spin(directionType::rev, rightDriveSpeed, velocityUnits::pct);
    } else if (con1.ButtonB.pressing()) {
      if(leftX > 0){
        leftDriveSpeed = drivePctSpeed - leftX;
        rightDriveSpeed = drivePctSpeed + leftX*M_PI_2;
      } else if (leftX < 0) {
        leftDriveSpeed = drivePctSpeed - leftX*M_PI_2;
        rightDriveSpeed = drivePctSpeed + leftX;
      } else {
        leftDriveSpeed = drivePctSpeed;
        rightDriveSpeed = drivePctSpeed;
      }
      left_drive_motor.spin(directionType::rev, leftDriveSpeed, velocityUnits::pct);
      right_drive_motor.spin(directionType::fwd, rightDriveSpeed, velocityUnits::pct);
    } else {
      left_drive_motor.spin(directionType::fwd, leftX, velocityUnits::pct);
      right_drive_motor.spin(directionType::fwd, leftX, velocityUnits::pct);
    }
    
    if (con1.ButtonL2.pressing()) {
      arm_motor.spin(directionType::fwd, armPctSpeed, velocityUnits::pct);
    } else if (con1.ButtonDown.pressing()) {
      arm_motor.spin(directionType::rev, armPctSpeed, velocityUnits::pct);
    } else {
      arm_motor.stop();
    }

  }
}

int main() {

  // InertialSensor.setHeading(0.0, rotationUnits::deg);
  // InertialSensor.setRotation(0.0, rotationUnits::deg);
  // InertialSensor.startCalibration();
  // while (InertialSensor.isCalibrating()) {
  //   task::sleep(10);
  // }
  // InertialSensor.setHeading(0.0, rotationUnits::deg);
  // InertialSensor.setRotation(0.0, rotationUnits::deg);
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //Competition.autonomous();
  Competition.drivercontrol(driving);
  
}
