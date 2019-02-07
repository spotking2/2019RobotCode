
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

  //DriveBase Initialization
  Talon frontLeft;
  Talon rearLeft;
  Talon frontRight;
  Talon rearRight;

  SpeedControllerGroup left;
  SpeedControllerGroup right;

  DifferentialDrive robot;
  Joystick j;

  //buttons and letUps
  final int rollerIn = 4, rollerOut = 5; //leftTrigger, rightTrigger

  boolean letUpRollerIn, letUpRollerOut;


  @Override
  public void robotInit() {
  j = new Joystick(0);

  frontLeft = new Talon(0);
  rearLeft = new Talon(1);
  frontRight = new Talon(2);
  rearRight = new Talon(3);

  left = new SpeedControllerGroup(frontLeft,rearLeft);
  right = new SpeedControllerGroup(frontRight,rearRight);

  robot = new DifferentialDrive(left,right);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    
    //drive code
    robot.arcadeDrive(-j.getRawAxis(1),j.getRawAxis(0));


    //roller code
    if(j.getRawButton(rollerIn) && letUpRollerIn) {
      //code
      letUpRollerIn = false;
    } else if(!j.getRawButton(rollerIn)) {
      letUpRollerIn = true;
    }

    if(j.getRawButton(rollerOut) && letUpRollerOut) {
      //code
      letUpRollerOut = false;
    } else if(!j.getRawButton(rollerOut)) {
      letUpRollerOut = true;
    }


    //manipulator code
        /*
          
        */


    //rail code


    //valve (climber and grip) code


    //climbing motor code


    //elevator code
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
