
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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
/*
  added some new positions for elevator and wrist after talking with Nick
  have confirmation button press before release of hatch for all functions (tap button 2nd time)
*/

  //Initialization
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
  Relay solenoidA; 
  Relay solenoidB;

  SpeedControllerGroup left;
  SpeedControllerGroup right;

  DifferentialDrive robot;
  Joystick j;

  

  //Sensors
  DigitalInput elevatorHigh;
  DigitalInput elevatorLow;
  
  //Do we want encoder for each side???
  Encoder vacuumSensor; 
  Encoder leftSide;
  Encoder rightSide;

  //buttons and progs
  int rollerIn = 4, rollerOut = 5, hatchPickupP = 2, hatchPickupF, hatchPlaceH, hatchPlaceL; //leftTrigger, rightTrigger
  int climbMotor, cargoPickupF, cargoPickupP, cargoPlaceRL, cargoPlaceRH, cargoPlaceC;
  int elevatorH, elevatorL;

  boolean progHatchPickupP = false, progHatchPickupF = false, progHatchPlaceH = false, progCargoPickupF = false;
  boolean progCargoPlaceRL = false, progCargoPlaceRH = false, progCargoPlaceC = false, progClimbMotor = false;
  boolean progClimbWinch = false, progElevatorH = false, progElevatorL = false, progHatchPlaceL = false;
  boolean progCancel = false, progCargoPickupP;
  
  boolean letUpClimb = true, letUpHatchPickupF;//used for confirmation press

  boolean[] inProgresses = new boolean[] {progHatchPickupP, progHatchPickupF, progHatchPlaceH, progCargoPickupF, progCargoPickupP, progCargoPlaceRL, progCargoPlaceRH, progCargoPlaceC, progClimbMotor, progClimbWinch, progElevatorH, progElevatorL, progHatchPlaceL};
  int progressCount;

  //PID
  double elevatorP, elevatorI, elevatorD, wristP, wristI, wristD;

  //misc
  double elevatorInput, wristInput;
  NetworkTable table;

  //place holders
  int elevatorNum, rollerNum, wristNum, railNum, vacuumPumpNum, solenoidANum, solenoidBNum, vacuumThreshold = 280/*(?)*/, vacuumHatchThreshold = 589, railIn, railOut;
  double elevatorHighPoition/*cargo/hatch middle*/, elevatorMiddlePosition/*cargo cargoship */, elevatorSemiLowPosition/*prevent scraping off suction cups for floor pickup*/, elevatorLowPosition/*hatch/cargo floor pickup*/, wristultraLowPosition/* cargo pickup floor*/, wristLowPosition/*hatch pickup floor*/, wristMiddlePosition/* cargo low rocket place*/, wristHightPosition/*hatch cargoship/low rocket, cargo low*/, wristTolerance, elevatorTolerance;
 

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
    rail.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0 , 0);

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
    elevator.set(ControlMode.Position, elevatorLowPosition);

    vacuumPump = new Spark(vacuumPumpNum);

    //Spike direction choosen at random, may need to be changed/not be neccasary
    //Written with following presumption:
    //solenoidA forward = isolation valve for manipulater
    //solenoidA reverse = isolation valve for climber
    //solenoidB forward = manipulater release valve
    //solenoidB reverse = extra channel
    solenoidA = new Relay(solenoidANum, Relay.Direction.kForward);
    solenoidB = new Relay(solenoidBNum, Relay.Direction.kForward);

    //misc
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("tableModnar");
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
    if(j.getRawButton(hatchPickupP) && !progHatchPickupP){//If B is pressed
      progHatchPickupP = true;
    }
    if(progHatchPickupP) {
      solenoidA.set(Relay.Value.kOn);
      if(vacuumSensor.get() < vacuumHatchThreshold) {
        //rumble code
        progHatchPickupP = false;
      }
    }
            //if the operation hasn't been canceled-continue
            //apply slight forward motion??? (to allign with the wall) - probably should be implemented elsewhere
            //if we have vacuum, send success code
            //or if there is a timeout and we still don't have vacuum, cancel and send error message and return to standard running

   
    //Pickup Hatch (Floor) - I made a lovely tree :)))))))))))
    if(j.getRawButton(hatchPickupF) && !progHatchPickupF) {
      progHatchPickupF = true;
      letUpHatchPickupF = false;
    } else if (!j.getRawButton(hatchPickupF)) {
      letUpHatchPickupF = true;
    }
    if(progHatchPickupF) {
      elevator.set(ControlMode.Position, elevatorSemiLowPosition);

      if(Math.abs(elevator.getSelectedSensorPosition()-elevatorSemiLowPosition) < elevatorTolerance) {
        wrist.set(ControlMode.Position, wristLowPosition);

        if(Math.abs(wrist.getSelectedSensorPosition()-wristLowPosition) < wristTolerance) {
          
          if(j.getRawButton(hatchPickupF) && letUpHatchPickupF) {
            elevator.set(ControlMode.Position, elevatorLowPosition);
            if(Math.abs(elevator.getSelectedSensorPosition()) < elevatorTolerance) {
              solenoidA.set(Relay.Value.kOn);
              if(vacuumSensor.get() < vacuumHatchThreshold) {
              //rumble code
              //reset Code
              progHatchPickupF = false;
              }
            }
          }
        }
      }
    }
            //if it hasn't been canceled-you may proceed
            //override movement probably - again, should probably be implemented elsewhere
            //flop wrist down
            //if wrist is down, lower the elevator to the proper spot
            //if the list, establish vacuum
            //if the new list, return to standard running
            //if we timeout and it isn't working - "Bad News Bears" (return to standard running)
    
    
        //Place Hatch (low)
    if(j.getRawButton(hatchPlaceL) && !progHatchPlaceL) {
      progHatchPlaceL = true;
    }
    if(progHatchPlaceL) {
      progHatchPlaceL = false;
    }
            //no cancel = good
            //???correct elevator position???
            //???if elevator position is correct??? then extend rail probably
            //if above, send success code (good vibes)
            //when canceled (also we should probably have a winch timeout in case of encoder failure) then bleed vacuum and retract rail
            //if it timed out we should have an error code
    //place hatch (high)
    if(j.getRawButton(hatchPlaceH) && !progHatchPlaceH) {
      progHatchPlaceH = true;
    }
    if(progHatchPlaceH) {
      progHatchPlaceH = false;
    }

    /////////////////////////////////////////////////////
    //I'm thinking that maybe these should be manually selected modes that just changes these operational positions 
    
    //Pickup Cargo (Floor)
    if(j.getRawButton(cargoPickupF) && !progCargoPickupF) {
      progCargoPickupF = true;
    }
    if(progCargoPickupF) {
      progCargoPickupF = false;
    }
        //???proper elevator position???
        //???if above??? then flop wrist

    //Pickup cargo (Player station)
    if(j.getRawButton(cargoPickupP) && !progCargoPickupP) {
      progCargoPickupP = true;
    }
    if(progCargoPickupP) {
      progCargoPickupP = false;
    }

    //Place Cargo (Rocket Low)
    if(j.getRawButton(cargoPlaceRL) && !progCargoPlaceRL) {
      progCargoPlaceRL = true;
    }
    if(progCargoPlaceRL) {
      progCargoPlaceRL = false;
    }
        //proper elevator height
        //flop wrist


    //Place Cargo (Rocket High)
    if(j.getRawButton(cargoPlaceRH) && !progCargoPlaceRH) {
      progCargoPlaceRH = true;
    }
    if(progCargoPlaceRH) {
      progCargoPlaceRH = false;
    }

    //Place Cargo (Cargo Ship)
    if(j.getRawButton(cargoPlaceC) && !progCargoPlaceC) {
      progCargoPlaceC = true;
    }
    if(progCargoPlaceC) {
      progCargoPlaceC = false;
    }

/////////////////////////////////////////////////////////
    
    
    //valve (climber and grip) code
      //vacuum always on, open/keep manipulator at start of match
    /*
    switch directions to choose valve then set value
    solenoidA.setDirection(direction);
    solenoidA.set(value);
    */

    //climbing motor code
    
      //bring down the arm
      if(j.getRawButton(climbMotor) && !progClimbMotor && letUpClimb) {
        progClimbMotor = true;
        letUpClimb = false;
      } else if(!j.getRawButton(climbMotor)) {
        letUpClimb = true;
      }

      if(progClimbMotor){
        //transition valves approriately (probably done elsewhere)
        //start bringing down the arm
        //winch up the robot

        if(j.getRawButton(climbMotor) && letUpClimb) {// press button again to activate winch
          progClimbWinch = true;
        }

        if(progClimbWinch){
            //if we got vacuum send signal??? or maybe just start winching
            //either way winch w/ maybe a cancel
            progClimbMotor = false;
            progClimbWinch = false;
        }
      }

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
        progHatchPickupP = false;
        progHatchPickupF = false;
        progHatchPlaceH = false;
        progCargoPickupF = false;
        progCargoPickupP = false;
        progCargoPlaceRL = false;
        progCargoPlaceRH = false;
        progCargoPlaceC = false;
        progClimbMotor = false;
        progClimbWinch = false;
        progElevatorH = false;
        progElevatorL = false;
        progHatchPlaceL = false;

        solenoidA.setDirection(Relay.Direction.kForward);

        //set the set points back to standard running mode(finishlogic making them return to default in order(wrist, rail, elevator))
        wrist.set(ControlMode.Position, wristHightPosition);
        elevator.set(ControlMode.Position, elevatorLowPosition);
        if(rail.getSelectedSensorPosition() > railIn ) {
          //rail in
        }
      }

      //vacuum - presuming max speed?
      if(vacuumSensor.get() > vacuumThreshold) {
        vacuumPump.set(1.0);
      }

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
