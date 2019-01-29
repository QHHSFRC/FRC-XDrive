package frc.robot;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
public class Robot extends TimedRobot {
  Joystick driveStick; //Joystick used to control the Robot
  ADXRS450_Gyro gyro; //The gyro that will return the robot's orientation (in order to make it field-centric)

  boolean centricState = false;
  
  double angle = 0;

  double joyX = 0; //Variable that will be used to store the X value of the control axis
  double joyY = 0; //Variable that will be used to store the Y value of the control axis
  double joyZ = 0; //Variable that will be used to store the Z value of the control axis

  double x2 = 0; //Variable that will be used to store the translated X value of the control axis
  double y2 = 0; //Variable that will be used to store the translated Y value of the control axis

  double vel = 0; //Variable that will be used to store the intended velocity regardless of direction

  double fL = 0; //Variable that will be used to store the value that will be passed to the Front Left motor
  double fR = 0; //Variable that will be used to store the value that will be passed to the Front Right motor
  double bR = 0; //Variable that will be used to store the value that will be passed to the Back Right motor
  double bL = 0; //Variable that will be used to store the value that will be passed to the Back Left motor

  @Override
  public void robotInit() {
    driveStick = new Joystick(0);
    gyro = new ADXRS450_Gyro();
    gyro.calibrate(); //Calibrate the gyro once the robot is on. This should be done while the robot *is not moving*
  }

  @Override
  public void autonomousInit() {
    gyro.reset(); //Reset the gyro to angle 0. This is assuming your robot will be facing the direction that you want to be "forward" for the field-centric program
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {
    DriveFunction(driveStick.getRawAxis(4), driveStick.getRawAxis(5), driveStick.getRawAxis(0), driveStick.getRawButton(5), driveStick.getRawButton(6), driveStick.getRawButton(8)); //During the driver operated period, continue looping the drive function
  }
/*
 The following are the parameters for the drive function in order
 The variable representing the x-axis of the joystick within the function (left/right)
 The variable representing the y-axis of the joystick within the function (forward/back)
 The variable representing the z-axis of the joystick within the function (turning)
 The boolean representing the button that puts the program into field-centric mode (controls are relative to the field)
 The boolean representing the button that puts the program into robot-centric mode (controls are relative to the robot)
 The boolean representing the button that resets the gyro angle to zero
*/
  public void DriveFunction(double xAxis, double yAxis, double zAxis, boolean switchButton1, boolean switchButton2, boolean resetButton) { //This is the drive function referenced in the driver operated period
    if(resetButton)  //Reset the gyro when the "resetButton" is pressed
    {
      gyro.reset();
    }

    angle = -gyro.getAngle(); //Our gyro's angle increased when going counter clockwise, which is not what we want; hence, the negative
    //If your gyro's angle increases when going clockwise, leave out the negative
    joyX = xAxis; //Set the xAxis container variable equal to the joystick's x-axis
    joyY = -yAxis; //Set the yAxis container variable equal to the joystick's y-axis (it's negative, because our "forward" was negative)
    joyZ = zAxis; //Set the xAxis container variable equal to the joystick's z-axis

    vel = (Math.sqrt((joyX * joyX) + (joyY * joyY)) / Math.sqrt(2)); //Set the velocity variable to that of the joystick
  
    if(switchButton1) //Put the robot in field-centric mode by setting the boolean to 0
    {
      centricState = false;
    }
    else if(switchButton2) //Put the robot in robot-centric mode by setting the boolean to 1
    {
      centricState = true;
    }

    if(centricState) //Test the mode that the robot is in (field/robot-centric)
    {
      x2 = joyX; //If it's in robot-centric mode, x2 and y2 will undertake no transformation and will simply reflect the joystick's axes
      y2 = joyY;
    }
    else
    {
      x2 = vel * (Math.cos(Math.atan2(joyY, joyX) - (angle * (Math.PI / 180)))); //If it's in field-centric mode, x2 and y2 will undertake a transformation based on the angle of the robot to reflect the change of direction
      y2 = vel * (Math.sin(Math.atan2(joyY, joyX) - (angle * (Math.PI / 180))));  
    }

    if(x2 == 0) //Because there are several instances where something is divided by x2, it being 0 would result in a runtime error. By setting it to the negligible 0.001, we avoid the runtime error without affecting performance
    {
      x2 = 0.001;
    }
    
    fL = (Math.sin(Math.atan(y2 / x2) + (Math.PI / 4)) * Math.sqrt((x2 * x2) + (y2 * y2))) / 1.5; //Set the motors to their appropriate speed based on the formula (each are offset by a factor of pi/2 to account for the 90 degree difference in the wheels)
    fR = (Math.sin(Math.atan(y2 / x2) + (3 * Math.PI / 4)) * Math.sqrt((x2 * x2) + (y2 * y2))) / 1.5; //The constant (1.5) can be adjusted to your driver's preference (though we recommend verifying that there is no instance where a motor controller would be passed a value outside of its range)
    bR = (Math.sin(Math.atan(y2 / x2) + (5 * Math.PI / 4)) * Math.sqrt((x2 * x2) + (y2 * y2))) / 1.5;
    bL = (Math.sin(Math.atan(y2 / x2) + (7 * Math.PI / 4)) * Math.sqrt((x2 * x2) + (y2 * y2))) / 1.5;
  
    if(x2 < 0) //Inverts the motors when x2 is less than 0 to account for the nonnegative sine curve
    {
      fL *= -1;
      fR *= -1;
      bR *= -1;
      bL *= -1;
    }

    fL += joyZ / 5; //Include the value of the turning axis in the output. We found it most comfortable to reduce the turning strength significantly, but the constant (5) can be adjusted to your driver's preference (though we recommend verifying that there is no instance where a motor controller would be passed a value outside of its range)
    fR += joyZ / 5;
    bR += joyZ / 5;
    bL += joyZ / 5;
  }

  @Override
  public void testPeriodic() {
  }
}
