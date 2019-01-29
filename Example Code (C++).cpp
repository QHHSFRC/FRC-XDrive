#include <Math.h> //Absolutely necessary for this to function *at all*
#include <frc/WPILib.h>
class Robot : public frc::TimedRobot {
  frc::Joystick driveStick{0}; //Joystick used to control the Robot
  frc::ADXRS450_Gyro gyro{}; //The gyro that will return the robot's orientation (in order to make it field-centric)
  
  bool centricState{0};
  
  float angle{0}; //Can be an int if the gyro does not return decimals
  
  float joyX{0}; //Variable that will be used to store the X value of the control axis
  float joyY{0}; //Variable that will be used to store the Y value of the control axis
  float joyZ{0}; //Variable that will be used to store the Z value of the control axis
  
  float x2{0}; //Variable that will be used to store the translated X value of the control axis
  float y2{0}; //Variable that will be used to store the translated Y value of the control axis
  
  float vel{0}; //Variable that will be used to store the intended velocity regardless of direction
  
  float fL{0}; //Variable that will be used to store the value that will be passed to the Front Left motor
  float fR{0}; //Variable that will be used to store the value that will be passed to the Front Right motor
  float bR{0}; //Variable that will be used to store the value that will be passed to the Back Right motor
  float bL{0}; //Variable that will be used to store the value that will be passed to the Back Left motor

    void RobotInit() {
      gyro.Calibrate();
    }
    void AutonomousInit() {
      gyro.Reset();
    }
    void AutonomousPeriodic() {}
    void TeleopInit() {}
    void TeleopPeriodic() {
      DriveFunction(driveStick.GetRawAxis(4), driveStick.GetRawAxis(5), driveStick.GetRawAxis(0), driveStick.GetRawButton(5), driveStick.GetRawButton(6));
    }
};
