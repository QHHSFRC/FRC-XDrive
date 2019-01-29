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
      DriveFunction(driveStick.GetRawAxis(4), driveStick.GetRawAxis(5), driveStick.GetRawAxis(0), driveStick.GetRawButton(5));
    }
    void DriveFunction(float xAxis, float yAxis, float zAxis, bool switchButton1, bool switchButton2, bool resetButton) {
      if(resetButton)
      {
        gyro.Reset();
      }
      
      angle = -gyro.GetAngle();
      
      joyX = xAxis;
      joyY = -yAxis;
      joyZ = zAxis;
      
      vel = (sqrt((joyX * joyX) + (joyY * joyY)) / sqrt(2));
      
      if(switchButton1)
      {
        centricState = 0;
      }
      else if(switchButton2)
      {
        centricState = 1;
      }
      
      if(centricState)
      {
        x2 = joyX;
        y2 = joyY;
      }
      else
      {
        x2 = vel * (cos(atan2(joyY, joyX) - (angle * (M_PI / 180))));
        y2 = vel * (sin(atan2(joyY, joyX) - (angle * (M_PI / 180))));
      }
      
      if(x2 == 0)
      {
        x2 = 0.001;
      }
      
      fL = (sin(atan(y2 / x2) + (M_PI / 4)) * sqrt((x2 * x2) + (y2 * y2))) / 1.5;
      fR = (sin(atan(y2 / x2) + (3 * M_PI / 4)) * sqrt((x2 * x2) + (y2 * y2))) / 1.5;
      bR = (sin(atan(y2 / x2) + (5 * M_PI / 4)) * sqrt((x2 * x2) + (y2 * y2))) / 1.5;
      bL = (sin(atan(y2 / x2) + (7 * M_PI / 4)) * sqrt((x2 * x2) + (y2 * y2))) / 1.5;
      
      if(x2 < 0)
      {
        fL *= -1;
        fR *= -1;
        bR *= -1;
        bL *= -1;
      }
      
      fL += joyZ / 5;
      fR += joyZ / 5;
      bR += joyZ / 5;
      bL += joyZ / 5;
    }
  
    void TestPeriodic() {}
};
#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>(); }
#endif
