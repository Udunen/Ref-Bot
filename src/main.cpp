/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Student                                          */
/*    Created:      Fri Oct 27 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

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
if lightsensor detect object in claw then close claw, light up green led, or vibrate controller

*/

/*
set motor pos 3 to 0 (fully open)
a - close claw for .5 sec (press again resets to pos 0)
*/

#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// cam_motor            motor         7               
// arm_motor            motor         8               
// claw_motor           motor         3               
// Controller1          controller                    
// DistanceSensor       distance      5               
// InertialSensor       inertial      9               
// LimitSwitch          limit         A               
// left_bumper          bumper        E               
// right_bumper         bumper        F               
// LightSensor          light         G               
// green                led           H               
// yellow               led           B               
// potentiometer        pot           D               
// red                  led           C               
// GPS                  gps           4               
// RotationSensor       rotation      11              
// left_drive_motor     motor         1               
// right_drive_motor    motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include <algorithm>
#include <cmath>

using namespace vex;

// vex::motor      right_drive_motor(vex::PORT10, vex::gearSetting::ratio18_1, false);
// vex::motor      left_drive_motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
// vex::motor      cam_motor(vex::PORT7, vex::gearSetting::ratio18_1, false);
// vex::motor      arm_motor(vex::PORT8, vex::gearSetting::ratio36_1, false);
// vex::motor      claw_motor(vex::PORT3, vex::gearSetting::ratio18_1, false);
// vex::controller Controller1(vex::controllerType::primary);
// vex::vision     VisionSensor(vex::PORT2);
vex::competition Competition;
// vex::distance   DistanceSensor(vex::PORT5);
// vex::gps        gps(vex::PORT5, 0, turnType::right);
// vex::inertial   InertialSensor(vex::PORT9);
// vex::rotation   RotationSensor(vex::PORT11);
// vex::limit      LimitSwitch(Brain.ThreeWirePort.A);
// vex::bumper     left_bumper(Brain.ThreeWirePort.E);
// vex::bumper     right_bumper(Brain.ThreeWirePort.F);
// vex::light      LightSensor(Brain.ThreeWirePort.G);
// vex::led        green(Brain.ThreeWirePort.H);
// vex::led        yellow(Brain.ThreeWirePort.B);
// vex::led        red(Brain.ThreeWirePort.C);
// vex::pot        potentiometer(Brain.ThreeWirePort.D);


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

    // camera
    if (Controller1.ButtonUp.pressing()) {
      cam_motor.spin(directionType::rev);
    } else if (Controller1.ButtonLeft.pressing()) {
      cam_motor.spin(directionType::fwd);
    } else {
      cam_motor.stop();
    }

    // claw
    if (Controller1.ButtonA.pressing()) {
      if (claw_motor.position(rotationUnits::deg) <= clawOpen) {
        claw_motor.spinFor(directionType::fwd, 0.5, sec);
      } else {
        // claw_motor.spinFor(directionType::rev, 0.1, sec);
        claw_motor.spinToPosition(clawOpen, rotationUnits::deg, true);
      }
    } else if (Controller1.ButtonRight.pressing()) {
      claw_motor.spin(fwd);
    } else {
      claw_motor.stop();
    }

    if(Controller1.ButtonX.pressing() && drivePctTemp < 100) {
      drivePctTemp += 25;
      task::sleep(300);     
    }
    if(Controller1.ButtonY.pressing() && drivePctTemp > 25) {
      drivePctTemp -= 25;
      task::sleep(300);
    }
    
    int leftX = Controller1.Axis4.position(pct);
    int leftY = Controller1.Axis3.position(pct);
    int rightX = Controller1.Axis1.position(pct);
    int rightY = Controller1.Axis2.position(pct);


    if(Controller1.ButtonR1.pressing() && !Controller1.ButtonL1.pressing() && !Controller1.ButtonR2.pressing()) {
      drivePctSpeed = rightY;
      armPctSpeed = 50.0;
    } else if (Controller1.ButtonL1.pressing() && !Controller1.ButtonR1.pressing() && !Controller1.ButtonL2.pressing()) {
      armPctSpeed = rightY;
      drivePctSpeed = drivePctTemp;
    } else if (Controller1.ButtonR1.pressing() && Controller1.ButtonL1.pressing()) {
      drivePctSpeed = 0.0;
      armPctSpeed = 0.0;
    } else {
      drivePctSpeed = drivePctTemp;
      armPctSpeed = 50.0;
    }


    if(Controller1.ButtonR2.pressing() ^ Controller1.ButtonR1.pressing()) {
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
    } else if (Controller1.ButtonB.pressing()) {
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
    
    if (Controller1.ButtonL2.pressing() && !Controller1.ButtonL1.pressing()) {
      arm_motor.spin(directionType::fwd, armPctSpeed, velocityUnits::pct);
    } else if (Controller1.ButtonL1.pressing() && !Controller1.ButtonL2.pressing()) {
      if(rightY != 0) {
        if (armPctSpeed < 0 && !LimitSwitch.pressing()) {
          arm_motor.spin(directionType::fwd, armPctSpeed, velocityUnits::pct);
        } else if (armPctSpeed > 0) {
          arm_motor.spin(directionType::fwd, armPctSpeed, velocityUnits::pct);
        } else {
          arm_motor.stop();
        }
      }
    } else if (Controller1.ButtonDown.pressing()) {
      if(!LimitSwitch.pressing()) {
        arm_motor.spin(directionType::rev, armPctSpeed, velocityUnits::pct);
      }
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
