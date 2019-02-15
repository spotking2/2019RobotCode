
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

  //buttons and progs
  int rollerIn = 4, rollerOut = 5, hatchPickupP = 2, hatchPickupF, hatchPlaceH, hatchPlaceL; //leftTrigger, rightTrigger
  int climbMotor, climbWinch, cargoPickup, cargoPlaceRL, cargoPlaceRH, cargoPlaceC;
  int elevatorH, elevatorL;

  boolean progHatchPickupP = false, progHatchPickupF = false, progHatchPlaceH = false, progCargoPickup = false;
  boolean progCargoPlaceRL = false, progCargoPlaceRH = false, progCargoPlaceC = false, progClimbMotor = false;
  boolean progClimbWinch = false, progElevatorH = false, progElevatorL = false, progHatchPlaceL = false;
  boolean progCancel = false;

  boolean[] inProgresses = new boolean[] {progHatchPickupP, progHatchPickupF, progHatchPlaceH, progCargoPickup, progCargoPlaceRL, progCargoPlaceRH, progCargoPlaceC, progClimbMotor, progClimbWinch, progElevatorH, progElevatorL, progHatchPlaceL};
  int progressCount;

  //PID
  double elevatorP, elevatorI, elevatorD, wristP, wristI, wristD;

  //misc
  double elevatorInput, wristInput;

  //place holders
  int elevatorNum, rollerNum, wristNum, railNum, vacuumPumpNum, solenoidANum, solenoidBNum;

  @Override
  public void robotInit() {
    j = new Joystick(0);

    //driveBase
    frontLeft = new Talon(0);
    rearLeft = new Talon(1);
    frontRight = new Talon(2);
    rearRight = new Talon(3);

    left = new SpeedControllerGroup(frontLeft,rearLeft);
    right = new SpeedControllerGroup(frontRight,rearRight);

    robot = new DifferentialDrive(left,right);

    //Motors
    roller = new VictorSP(rollerNum);

    rail = new TalonSRX(railNum);

    wrist = new TalonSRX(wristNum);
    wrist.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    wrist.config_kP(0, wristP);
    wrist.config_kI(0, wristI);
    wrist.config_kD(0, wristD);

    elevator = new VictorSPX(elevatorNum);
    elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    elevator.config_kP(0, elevatorP);
    elevator.config_kI(0, elevatorI);
    elevator.config_kD(0, elevatorD);

    vacuumPump = new Spark(vacuumPumpNum);

    solenoidA = new Relay(solenoidANum);

    solenoidB = new Relay(solenoidBNum);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    progressCount = 0;
  }

  @Override
  public void teleopPeriodic() {
    
    //drive code
    robot.arcadeDrive(-j.getRawAxis(1),j.getRawAxis(0));


    //roller code - may need to be reversed
    if(j.getRawAxis(rollerIn) > 0.3) {
      roller.set(0.35);
    } else if(j.getRawAxis(rollerOut) > 0.3) {
      roller.set(-0.35);
    } else{
      roller.set(0);
    }

    //manipulator code

    //Pickup Hatch (Player Station)
    if(j.getRawButton(hatchPickupP) && progHatchPickupP){//If B is pressed
      //vacuum code stuff
    }
            //if the operation hasn't been canceled-continue
            //apply slight forward motion??? (to allign with the wall) - probably should be implemented elsewhere
            //if we have vacuum, send success code
            //or if there is a timeout and we still don't have vacuum, cancel and send error message and return to standard running

   
    //Pickup Hatch (Floor)
    if(j.getRawButton(hatchPickupF) && progHatchPickupF) {

    }
            //if it hasn't been canceled-you may proceed
            //override movement probably - again, should probably be implemented elsewhere
            //flop wrist down
            //if wrist is down, lower the elevator to the proper spot
            //if the list, establish vacuum
            //if the new list, return to standard running
            //if we timeout and it isn't working - "Bad News Bears" (return to standard running)
    
    
        //Place Hatch (low)
    if(j.getRawButton(hatchPlaceL) && progHatchPlaceL) {

    }
            //no cancel = good
            //???correct elevator position???
            //???if elevator position is correct??? then extend rail probably
            //if above, send success code (good vibes)
            //when canceled (also we should probably have a winch timeout in case of encoder failure) then bleed vacuum and retract rail
            //if it timed out we should have an error code
    //place hatch (high)
    if(j.getRawButton(hatchPlaceH) && progHatchPlaceH) {

    }

    /////////////////////////////////////////////////////
    //I'm thinking that maybe these should be manually selected modes that just changes these operational positions 
    
    //Pickup Cargo
    if(j.getRawButton(cargoPickup) && progCargoPickup) {

    }
        //???proper elevator position???
        //???if above??? then flop wrist


    //Place Cargo (Rocket Low)
    if(j.getRawButton(cargoPlaceRL) && progCargoPlaceRL) {

    }
        //proper elevator height
        //flop wrist


    //Place Cargo (Rocket High)
    if(j.getRawButton(cargoPlaceRH) && progCargoPlaceRH) {

    }

    //Place Cargo (Cargo Ship)
    if(j.getRawButton(cargoPlaceC) && progCargoPlaceC) {

    }

/////////////////////////////////////////////////////////
    
    
    //valve (climber and grip) code
      //vacuum always on, open/keep manipulator at start of match


    //climbing motor code
    
      //bring down the arm
      if(j.getRawButton(climbMotor) && progClimbMotor) {

      }
          //transition valves approriately (probably done elsewhere)
          //start bringing down the arm


      //winch up the robot
      if(j.getRawButton(climbWinch) && progClimbWinch) {

      }
          //if we got vacuum send signal??? or maybe just start winching
          //either way winch w/ maybe a cancel
      
      //Cancel + Reset
      for(boolean b: inProgresses) {
        if(b) {
          progressCount++;
        }
        if(progressCount == 2) {
          progCancel = true;
          break;
        }
      }

      if(progCancel) {
        
      }

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
