
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


    //roller code - may need to be reversed
    if(j.getRawAxis(rollerIn) > 0.3) {
      roller.set(0.35);
    }else if(j.getRawAxis(rollerOut) > 0.3) {
      roller.set(-0.35);
    }else{
      roller.set(0);
    }


    //manipulator code
        //Pickup Hatch (Player Station)
            //if the operation hasn't been canceled-continue
            //apply slight forward motion??? (to allign with the wall) - probably should be implemented elsewhere
            //let out rail???
            //if rail is let out??? apply vacuum
            //if we have vacuum, send success code & retract rail
            //or if there is a timeout and we still don't have vacuum, cancel and send error message and return to standard running


        //Pickup Hatch (Floor)
            //if it hasn't been canceled-you may proceed
            //override movement probably - again, should probably be implemented elsewhere
            //flop wrist down
            //if wrist is down, lower the elevator to the proper spot
            //if the list, establish vacuum
            //if the new list, return to standard running
            //if we timeout and it isn't working - "Bad News Bears" (return to standard running)
    
    
        //Place Hatch
            //no cancel = good
            //???correct elevator position???
            //???if elevator position is correct??? then extend rail probably
            //if above, send success code (good vibes)
            //when canceled (also we should probably have a winch timeout in case of encoder failure) then bleed vacuum and retract rail
            //if it timed out we should have an error code
    

    /////////////////////////////////////////////////////
    //I'm thinking that maybe these should be manually selected modes that just changes these operational positions 
    
        //Pickup Cargo (Floor)
            //???proper elevator position???
            //???if above??? then flop wrist


        //Place Cargo (Rocket Low)
            //proper elevator height
            //flop wrist


        //Place Cargo (Rocket High)


        //Place Cargo (Cargo Ship)

/////////////////////////////////////////////////////////
    
    
    //valve (climber and grip) code
      //Bleed Vacuum??? (might be other places)


      //Switch between the two??? (that ^)


    //climbing motor code
      //bring down the arm
          //transition valves approriately (probably done elsewhere)
          //start bringing down the arm


      //winch up the robot
          //if we got vacuum send signal??? or maybe just start winching
          //either way winch w/ maybe a cancel

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
