using namespace vex;

extern brain Brain;

using signature = vision::signature;

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
extern led greenLED;
extern led yellowLED;
extern pot potentiometer;
extern led redLED;
extern gps GPS;
extern rotation RotationSensor;
extern motor left_drive_motor;
extern motor right_drive_motor;
extern signature VisionSensor__G_TRI;
extern signature VisionSensor__R_TRI;
extern signature VisionSensor__B_TRI;
extern signature VisionSensor__SIG_4;
extern signature VisionSensor__SIG_5;
extern signature VisionSensor__SIG_6;
extern signature VisionSensor__SIG_7;
extern vision VisionSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );