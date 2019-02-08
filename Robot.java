
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.SRF_PID;


public class Robot extends TimedRobot {

  //DriveBase Initialization
  Talon frontLeft;
  Talon rearLeft;
  Talon frontRight;
  Talon rearRight;

  VictorSP climberArm;
  Victor climberWinch;

  TalonSRX wrist;
  VictorSP roller;
  VictorSPX elevator;
  TalonSRX rail;

  Spark vacuumPump;
  Relay solenoidA; // define this crap ---------------------------------------------------------------------------------------
  Relay solenoidB;

  SpeedControllerGroup left;
  SpeedControllerGroup right;

  DifferentialDrive robot;
  Joystick j;

  //buttons and letUps
  final int rollerIn = 4, rollerOut = 5; //leftTrigger, rightTrigger

  boolean letUpRollerIn = true, letUpRollerOut = true;


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
      roller.set(0.35);
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
        //Pickup Hatch (Player Station)


        //Pickup Hatch (Floor)


        //Place Hatch


        //Pickup Cargo (Floor)


        //Place Cargo (Rocket Low)


        //Place Cargo (Rocket High)


        //Place Cargo (Cargo Ship)


    //valve (climber and grip) code
      //Bleed Vacuum??? (might be other places)


      //Switch between the two??? (that ^)


    //climbing motor code
      //bring down the arm


      //winch up the robot


    //elevator code
      //high


      //low


      //floor pickup
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
