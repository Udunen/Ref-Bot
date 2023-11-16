#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
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
led greenLED = led(Brain.ThreeWirePort.H);
led yellowLED = led(Brain.ThreeWirePort.B);
pot potentiometer = pot(Brain.ThreeWirePort.D);
led redLED = led(Brain.ThreeWirePort.C);
gps GPS = gps(PORT4, 0.00, 0.00, mm, 180);
rotation RotationSensor = rotation(PORT11, false);
motor left_drive_motor = motor(PORT1, ratio18_1, false);
motor right_drive_motor = motor(PORT10, ratio18_1, false);
/*vex-vision-config:begin*/
signature VisionSensor__G_TRI = signature (1, -7961, -6469, -7215, -5759, -4507, -5133, 4.2, 0);
signature VisionSensor__R_TRI = signature (2, 6127, 8791, 7459, -1147, -413, -780, 3.3, 0);
signature VisionSensor__B_TRI = signature (3, -4915, -3729, -4322, 7583, 14129, 10856, 3.1, 0);
vision VisionSensor = vision (PORT2, 50, VisionSensor__G_TRI, VisionSensor__R_TRI, VisionSensor__B_TRI);
/*vex-vision-config:end*/

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