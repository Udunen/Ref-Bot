#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftDriveSmart = motor(PORT1, ratio18_1, false);
motor RightDriveSmart = motor(PORT10, ratio18_1, true);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
motor cam_motor = motor(PORT7, ratio18_1, false);
motor arm_motor = motor(PORT8, ratio36_1, false);
motor claw_motor = motor(PORT3, ratio18_1, false);
controller Controller1 = controller(primary);
distance DistanceSensor = distance(PORT5);
inertial InertialSensor = inertial(PORT9);
limit LimitSwitch = limit(Brain.ThreeWirePort.A);
bumper left_bumper = bumper(Brain.ThreeWirePort.E);
bumper right_bumper = bumper(Brain.ThreeWirePort.F);
light LightSensor = light(Brain.ThreeWirePort.G);
led green = led(Brain.ThreeWirePort.H);
led yellow = led(Brain.ThreeWirePort.B);
pot potentiometer = pot(Brain.ThreeWirePort.D);
led red = led(Brain.ThreeWirePort.C);
gps GPS = gps(PORT4, 0.00, 0.00, mm, 180);
rotation RotationSensor = rotation(PORT11, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}