package frc.robot;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
public class Robot extends TimedRobot {
  Joystick driveStick;
  ADXRS450_Gyro gyro;

  boolean centricState = false;
  
  double angle = 0;

  double joyX = 0;
  double joyY = 0;
  double joyZ = 0;

  double x2 = 0;
  double y2 = 0;

  double vel = 0;

  double fL = 0;
  double fR = 0;
  double bR = 0;
  double bL = 0;

  @Override
  public void robotInit() {
    driveStick = new Joystick(0);
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();
  }

  @Override
  public void autonomousInit() {
    gyro.reset();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {
    DriveFunction(driveStick.getRawAxis(4), driveStick.getRawAxis(5), driveStick.getRawAxis(0), driveStick.getRawButton(5), driveStick.getRawButton(6), driveStick.getRawButton(8));
  }

  public void DriveFunction(double xAxis, double yAxis, double zAxis, boolean switchButton1, boolean switchButton2, boolean resetButton) {
    if(resetButton)
    {
      gyro.reset();
    }

    angle = -gyro.getAngle();

    joyX = xAxis;
    joyY = -yAxis;
    joyZ = zAxis;

    vel = (Math.sqrt((joyX * joyX) + (joyY * joyY)) / Math.sqrt(2));
  
    if(switchButton1)
    {
      centricState = false;
    }
    else if(switchButton2)
    {
      centricState = true;
    }

    if(centricState)
    {
      x2 = joyX;
      y2 = joyY;
    }
    else
    {
      x2 = vel * (Math.cos(Math.atan2(joyY, joyX) - (angle * (Math.PI / 180))));
      y2 = vel * (Math.sin(Math.atan2(joyY, joyX) - (angle * (Math.PI / 180))));  
    }

    if(x2 == 0)
    {
      x2 = 0.001;
    }
    
    fL = (Math.sin(Math.atan(y2 / x2) + (Math.PI / 4)) * Math.sqrt((x2 * x2) + (y2 * y2))) / 1.5;
    fR = (Math.sin(Math.atan(y2 / x2) + (3 * Math.PI / 4)) * Math.sqrt((x2 * x2) + (y2 * y2))) / 1.5;
    bR = (Math.sin(Math.atan(y2 / x2) + (5 * Math.PI / 4)) * Math.sqrt((x2 * x2) + (y2 * y2))) / 1.5;
    bL = (Math.sin(Math.atan(y2 / x2) + (7 * Math.PI / 4)) * Math.sqrt((x2 * x2) + (y2 * y2))) / 1.5;
  
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

  @Override
  public void testPeriodic() {
  }
}
