using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor cam_motor;
extern motor arm_motor;
extern motor claw_motor;
extern controller Controller1;
extern distance DistanceSensor;
extern inertial InertialSensor;
extern limit LimitSwitch;
extern bumper left_bumper;
extern bumper right_bumper;
extern light LightSensor;
extern led green;
extern led yellow;
extern pot potentiometer;
extern led red;
extern gps GPS;
extern rotation RotationSensor;
extern motor left_drive_motor;
extern motor right_drive_motor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );