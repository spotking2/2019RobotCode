
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
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


public class Robot extends TimedRobot {//2.0
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
  TalonSRX elevator;
  TalonSRX rail;

  Spark vacuumPump;
  Relay solenoidA; //let's get more symbolic names for these
  Relay solenoidB;

  SpeedControllerGroup left;
  SpeedControllerGroup right;

  DifferentialDrive robot;
  Joystick j;

  //Sensors
  DigitalInput elevatorHigh;
  DigitalInput elevatorLow;
  
  //Do we want encoder for each side???
  AnalogInput vacuumSensor;
  Encoder leftSide;
  //Encoder rightSide;

  //buttons and progs
  int rollerIn = 4, rollerOut = 5, hatchPickupP = 2, hatchPickupF, hatchPlaceH, hatchPlaceL; //leftTrigger, rightTrigger, B
  int climbMotor, cargoPickupF, cargoPickupP, cargoPlaceRL, cargoPlaceRH, cargoPlaceC;
  int elevatorH, elevatorL;

  int progHatchPickupP = 0, progHatchPickupF = 1, progHatchPlaceH = 2, progCargoPickupF = 3;//array indices for inProgresses
  int progCargoPlaceRL = 4, progCargoPlaceRH = 5, progCargoPlaceC = 6, progClimbMotor = 7;
  int progClimbWinch = 8, progElevatorH = 9, progElevatorL = 10, progHatchPlaceL = 11;
  int progCargoPickupP = 12;
  boolean progHatchPickupF2, progCancel = false;

  boolean[] inProgresses = new boolean[] {false, false, false, false, false, false, false, false, false, false, false, false, false};
  
  boolean letUpClimb = true, letUpHatchPickupF = true, letUpCargoPickupF = true, letUpCargoPickupP = true, letUpCargoPlaceC = true, letUpCargoPlaceRL = true, letUpCargoPlaceRH = true;//used for confirmation press
 
  int progressCount;

  //PID
  double elevatorP, elevatorI, elevatorD, wristP, wristI, wristD;

  //misc
  double elevatorInput, wristInput;
  NetworkTable table;

  //place holders
  int elevatorNum = 0, rollerNum = 7, wristNum = 2, railNum = 1, vacuumPumpNum = 4, vacuumSensorNum = 0, solenoidANum = 0, solenoidBNum = 1, climberArmNum = 6, climberWinchNum = 5, elevatorHighNum = 0, elevatorLowNum = 1, vacuumThreshold = 280/*(?)*/, vacuumHatchThreshold = 589, railIn, railOut;
  double elevatorHighPosition/*cargo/hatch middle*/, elevatorMiddlePosition/*cargo cargoship */, elevatorSemiLowPosition/*prevent scraping off suction cups for floor pickup*/, elevatorLowPosition/*hatch floor/playerstation pickup, hatch place cargoship/low rocket*/, wristUltraLowPosition/* cargo pickup floor*/, wristLowPosition/*hatch pickup floor*/, wristMiddlePosition/* cargo low rocket place*/, wristHighPosition/*hatch cargoship/low rocket, cargo low*/;
  double wristTolerance, elevatorTolerance, railTolerance;

  private static final boolean elevatorEnable = false;//a boolean that enables and disables the elevator motion (to handle pre-elevator movement)
  private static final boolean testMode = true;
  private int testSystem = 0;
  boolean letUpSystem = true;
  double testAxis;

  @Override
  public void robotInit() {
    elevatorHigh = new DigitalInput(elevatorHighNum);
    elevatorLow = new DigitalInput(elevatorLowNum);

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

    elevator = new TalonSRX(elevatorNum);
    elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    elevator.config_kP(0, elevatorP);
    elevator.config_kI(0, elevatorI);
    elevator.config_kD(0, elevatorD);
    elevator.set(ControlMode.Position, elevatorLowPosition);

    climberArm = new VictorSP(climberArmNum);

    climberWinch = new Victor(climberWinchNum);

    vacuumPump = new Spark(vacuumPumpNum);

    //Spike direction choosen at random, may need to be changed/not be neccasary
    //Written with following presumption:
    //solenoidA forward = isolation valve for manipulater
    //solenoidA reverse = isolation valve for climber
    //solenoidB forward = manipulater release valve
    //solenoidB reverse = extra channel (is currently disabled as solenoidB can only be run Forwards at the moment)
    solenoidA = new Relay(solenoidANum);
    solenoidB = new Relay(solenoidBNum, Relay.Direction.kForward);

    leftSide = new Encoder(2,3);
    vacuumSensor = new AnalogInput(vacuumSensorNum);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    SRF_Control();
  }

  @Override
  public void teleopInit() {
    progressCount = 0;
  }

  @Override
  public void teleopPeriodic() {
    if(testMode)
      SRF_Test();
    else
      SRF_Control();
  }

  void SRF_Test(){
    robot.arcadeDrive(-j.getRawAxis(1),j.getRawAxis(0));

    if(j.getRawButton(1) && letUpSystem) {
      testSystem++;
      if(testSystem == 6)
        testSystem = 0;
      letUpSystem = false;
    }
    else if(!j.getRawButton(1))
      letUpSystem = true;

    testAxis = j.getRawAxis(5);
    if(Math.abs(testAxis) > 0.1)
    {
      if(testSystem==0)
        climberArm.set(testAxis);
      else if(testSystem==1)
        wrist.set(ControlMode.PercentOutput, testAxis);
      else if(testSystem==2)
        roller.set(testAxis);
      else if(testSystem==3)
        elevator.set(ControlMode.PercentOutput,testAxis);
      else if(testSystem==4)
        rail.set(ControlMode.PercentOutput,testAxis);
      else
        vacuumPump.set(testAxis);
    }
  }


  //the thing we use to drive the robot
  public void SRF_Control() {
    
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
    if(j.getRawButton(hatchPickupP) && !inProgresses[progHatchPickupP]){//If B is pressed
      inProgresses[progHatchPickupP] = true;
    }
    if(inProgresses[progHatchPickupP]) {
      solenoidA.set(Relay.Value.kForward);//open valve to manipulator
      if(vacuumSensor.getValue() < vacuumHatchThreshold) {
        //rumblecode
        inProgresses[progHatchPickupP] = false;
      }
    }
            //if the operation hasn't been canceled-continue
            //apply slight forward motion??? (to allign with the wall) - probably should be implemented elsewhere
            //if we have vacuum, send success code
            //or if there is a timeout and we still don't have vacuum, cancel and send error message and return to standard running

    //we can only do floor pickup if the elevator is enabled


    //Pickup Hatch (Floor) - I made a lovely tree :)))))))))))
    if(j.getRawButton(hatchPickupF) && !inProgresses[progHatchPickupF] && elevatorEnable && letUpHatchPickupF) {
      inProgresses[progHatchPickupF] = true;
      letUpHatchPickupF = false;
    } else if (!j.getRawButton(hatchPickupF)) {
      letUpHatchPickupF = true;
    }//this else if might need to be a seperate if, we should break down logic to verify
    if(inProgresses[progHatchPickupF] && !progHatchPickupF2) { 
      elevator.set(ControlMode.Position, elevatorSemiLowPosition);

      if(Math.abs(elevator.getSelectedSensorPosition()-elevatorSemiLowPosition) < elevatorTolerance) {
        wrist.set(ControlMode.Position, wristLowPosition);

        if(Math.abs(wrist.getSelectedSensorPosition()-wristLowPosition) < wristTolerance) {
          
          if(j.getRawButton(hatchPickupF) && letUpHatchPickupF) {
            progHatchPickupF2 = true;
          }
        }
      }
    }
    if(progHatchPickupF2) {//I moved this out so that we wouldn't have the two elevator.set statements interfering with eachother - NEW
      elevator.set(ControlMode.Position, elevatorLowPosition);

      if(Math.abs(elevator.getSelectedSensorPosition()) < elevatorTolerance) {
        solenoidA.set(Relay.Value.kForward);

        if(vacuumSensor.getValue() < vacuumHatchThreshold) {
        //rumble code
          inProgresses[progHatchPickupF] = false;
          progHatchPickupF2 = false;
          progCancel = true;
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
    if(j.getRawButton(hatchPlaceL) && !inProgresses[progHatchPlaceL]) {
      inProgresses[progHatchPlaceL] = true;
    }
    if(inProgresses[progHatchPlaceL]) {
      rail.set(ControlMode.PercentOutput,0.35);

      if(rail.getSelectedSensorPosition() > railOut)
      {
        rail.set(ControlMode.PercentOutput, 0);
        solenoidA.set(Relay.Value.kOff);
        solenoidB.set(Relay.Value.kOn);

        if(vacuumSensor.getValue() > vacuumHatchThreshold) {//We might want this to be vacuum threshold but I'm not sure yet (also at tag RAS)-Scott
          inProgresses[progHatchPlaceL] = false;
          progCancel = true;
        }
      }
    }
            //no cancel = good
            //???correct elevator position???
            //???if elevator position is correct??? then extend rail probably
            //if above, send success code (good vibes)
            //when canceled (also we should probably have a winch timeout in case of encoder failure) then bleed vacuum and retract rail
            //if it timed out we should have an error code


    //place hatch (high)
    if(elevatorEnable && j.getRawButton(hatchPlaceH) && !inProgresses[progHatchPlaceH]) { //RAS
      inProgresses[progHatchPlaceH] = true;
    }
    if(inProgresses[progHatchPlaceH]) {
      if(elevatorEnable)
        elevator.set(ControlMode.Position, elevatorHighPosition);

      if(!elevatorEnable || Math.abs(elevator.getSelectedSensorPosition()-elevatorHighPosition) < elevatorTolerance) {

        if(Math.abs(rail.getSelectedSensorPosition() - railOut) < elevatorTolerance) {
          solenoidB.set(Relay.Value.kOn);
          solenoidA.set(Relay.Value.kOff);

          if(vacuumSensor.getValue() > vacuumHatchThreshold) {
            inProgresses[progHatchPlaceH] = false;
            progCancel = true;
          }
        } else if(rail.getSelectedSensorPosition() < railOut) {
          rail.set(ControlMode.PercentOutput, .35);
        } else {
          rail.set(ControlMode.PercentOutput, -.35);
        }
      }
    }

    /////////////////////////////////////////////////////
    //I'm thinking that maybe these should be manually selected modes that just changes these operational positions 
    //Or maybe it shouldn't ever be manually changed? 
    
    //Pickup Cargo (Floor)
    if(j.getRawButton(cargoPickupF) && !inProgresses[progCargoPickupF] && elevatorEnable) {
      inProgresses[progCargoPickupF] = true;
      letUpCargoPickupF = false;
    } else if(!j.getRawButton(cargoPickupF)) {
      letUpCargoPickupF = true;
    }
    if(inProgresses[progCargoPickupF]) {
      elevator.set(ControlMode.Position, elevatorSemiLowPosition);

      if(Math.abs(elevator.getSelectedSensorPosition()-elevatorSemiLowPosition) < elevatorTolerance) {
        wrist.set(ControlMode.Position, wristUltraLowPosition);

        if(Math.abs(wrist.getSelectedSensorPosition()-wristUltraLowPosition) < wristTolerance) {

          if(j.getRawButton(cargoPickupF) && letUpCargoPickupF) {
            inProgresses[progCargoPickupF] = false;
            progCancel = true;
          }
        }
      } 
    }

    //Pickup cargo (Player station)
    if(j.getRawButton(cargoPickupP) && !inProgresses[progCargoPickupP] && elevatorEnable) {
      inProgresses[progCargoPickupP] = true;
      letUpCargoPickupP = false;
    } else if(!j.getRawButton(cargoPickupP)) {
      letUpCargoPickupP = true;
    }
    if(inProgresses[progCargoPickupP]) {
      wrist.set(ControlMode.Position,wristMiddlePosition);
      if(Math.abs(wrist.getSelectedSensorPosition()-wristMiddlePosition) < wristTolerance) {

        if(j.getRawButton(cargoPickupP) && letUpCargoPickupP) {
          inProgresses[progCargoPickupP] = false;
          progCancel = true;
        }
      }
    }

    //Place Cargo (Rocket Low)
    if(j.getRawButton(cargoPlaceRL) && !inProgresses[progCargoPlaceRL] && letUpCargoPlaceRL) {
      inProgresses[progCargoPlaceRL] = true;
      letUpCargoPlaceRL = false;
    } else if(!j.getRawButton(cargoPlaceRL)) {
      letUpCargoPlaceRL = true;
    }
    if(inProgresses[progCargoPlaceRL]) {
      if(Math.abs(rail.getSelectedSensorPosition() - railOut) < elevatorTolerance) {
        wrist.set(ControlMode.Position,wristMiddlePosition);
        if(Math.abs(wrist.getSelectedSensorPosition() - wristMiddlePosition) < wristTolerance) {

          if(j.getRawButton(cargoPlaceRL) && letUpCargoPlaceRL) {
            inProgresses[progCargoPlaceRL] = false;
            progCancel = true;
          }
        }
      } else if(rail.getSelectedSensorPosition() < railOut) {
        rail.set(ControlMode.PercentOutput, .35);
      } else {
        rail.set(ControlMode.PercentOutput, -.35);
      }

    }
        //proper elevator height
        //flop wrist


    //Place Cargo (Rocket High)
    if(j.getRawButton(cargoPlaceRH) && !inProgresses[progCargoPlaceRH] && letUpCargoPlaceRH) {
      inProgresses[progCargoPlaceRH] = true;
      letUpCargoPlaceRH = false;
    } else if(!j.getRawButton(cargoPlaceRH)) {
      letUpCargoPlaceRH = true;
      
    }
    if(inProgresses[progCargoPlaceRH]) {
      elevator.set(ControlMode.Position, elevatorHighPosition);
      if(Math.abs(elevator.getSelectedSensorPosition() - elevatorHighPosition) < elevatorTolerance) {
        if(Math.abs(rail.getSelectedSensorPosition() - railOut) < elevatorTolerance) {
          if(j.getRawButton(cargoPlaceRH) && letUpCargoPlaceRH) {
           inProgresses[progCargoPlaceRH] = false;
           progCancel = true;
          }
        } else if(rail.getSelectedSensorPosition() < railOut) {
          rail.set(ControlMode.PercentOutput, .35);
        } else {
          rail.set(ControlMode.PercentOutput, -.35);
        }
      }
    }

    //Place Cargo (Cargo Ship)
    if(j.getRawButton(cargoPlaceC) && !inProgresses[progCargoPlaceC] && letUpCargoPlaceC) {
      inProgresses[progCargoPlaceC] = true;
      letUpCargoPlaceC = false;
    } else if(!j.getRawButton(cargoPlaceC)) {
      letUpCargoPlaceC = true;
    }
    if(inProgresses[progCargoPlaceC]) {
      elevator.set(ControlMode.Position, elevatorMiddlePosition);
      if(Math.abs(elevator.getSelectedSensorPosition() - elevatorMiddlePosition) < elevatorTolerance) {
        wrist.set(ControlMode.Position, wristLowPosition);
        if(Math.abs(wrist.getSelectedSensorPosition() - wristLowPosition) < wristTolerance) {
          if(j.getRawButton(cargoPlaceC) && letUpCargoPlaceC) {
            inProgresses[progCargoPlaceC] = false;
            progCancel = true;
          }
        }
      }
    }

/////////////////////////////////////////////////////////
    
    
    //valve (climber and grip) code

      if(j.getRawButton(climbMotor) && !inProgresses[progClimbMotor] && letUpClimb) {
        inProgresses[progClimbMotor] = true;
        letUpClimb = false;
      } else if(!j.getRawButton(climbMotor)) {
        letUpClimb = true;
      }

      if(inProgresses[progClimbMotor]){
        
        climberArm.set(j.getRawAxis(5));
        solenoidA.setDirection(Relay.Direction.kReverse);
        solenoidB.set(Relay.Value.kOn);
        if(vacuumSensor.getValue() < vacuumThreshold) {
          //controller rumble
          if(j.getRawButton(climbMotor) && letUpClimb) {// press button again to activate winch
            inProgresses[progClimbWinch] = true;
          }
          if(inProgresses[progClimbWinch]){
            climberWinch.set(j.getRawAxis(5));
            inProgresses[progClimbMotor] = false;
            inProgresses[progClimbWinch] = false;
          }
        }
      }

      //Cancel + Reset
      for(boolean b: inProgresses) {//does inProgresses ever actually get updated after initialization? If not this might not work
        if(b) {
          progressCount++;
        }
        if(progressCount == 2) {
          progCancel = true;
          break;
        }
      }

      //SRF_PID get the kP out of that

      //Exterior Rail control - XXX

      if(progCancel) { //add timeout here - XXX
        inProgresses[progHatchPickupP] = false;
        inProgresses[progHatchPickupF] = false;
        inProgresses[progHatchPlaceH] = false;
        inProgresses[progCargoPickupF] = false;
        inProgresses[progCargoPickupP] = false;
        inProgresses[progCargoPlaceRL] = false;
        inProgresses[progCargoPlaceRH] = false;
        inProgresses[progCargoPlaceC] = false;
        inProgresses[progClimbMotor] = false;
        inProgresses[progClimbWinch] = false;
        inProgresses[progElevatorH] = false;
        inProgresses[progElevatorL] = false;
        inProgresses[progHatchPlaceL] = false;

        solenoidA.setDirection(Relay.Direction.kForward);
        solenoidB.set(Relay.Value.kOff);
        //set the set points back to standard running mode(finishlogic making them return to default in order(wrist, rail, elevator))
        wrist.set(ControlMode.Position, wristHighPosition);
        if(Math.abs(wrist.getSelectedSensorPosition() - wristHighPosition) < wristTolerance) {
          rail.set(ControlMode.PercentOutput, 0);
            
          if(Math.abs(rail.getSelectedSensorPosition() - railIn) < railTolerance) {
            elevator.set(ControlMode.Position, elevatorLowPosition);
            
            if(Math.abs(elevator.getSelectedSensorPosition() - elevatorLowPosition) < elevatorTolerance) {
              progCancel = false;
            }
          }else if(rail.getSelectedSensorPosition() > railIn ) {
            rail.set(ControlMode.PercentOutput, -.35);
          } else {
            rail.set(ControlMode.PercentOutput, .35);
          }
        }
      }

      //vacuum - presuming max speed?
      if(vacuumSensor.getValue() > vacuumThreshold) {
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
