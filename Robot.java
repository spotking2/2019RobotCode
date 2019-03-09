
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SRF_PID;


public class Robot extends TimedRobot {//v1.5.11
/*
  added some new positions for elevator and wrist after talking with Nick
  have confirmation button press before release of hatch for all functions (tap button 2nd time)

  stuck in a thing for automated vacuum pump and changed B to opening isolation valve
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
  Relay isolationRelay;
  Relay bleedRelay;

  SpeedControllerGroup left;
  SpeedControllerGroup right;

  DifferentialDrive robot;
  Joystick j;
  Joystick joyTune = new Joystick(1);

  //Sensors
  DigitalInput elevatorHigh;
  DigitalInput elevatorLow;
  
  //Do we want encoder for each side???
  AnalogInput vacuumSensor;
  Encoder leftSide;
  Encoder rightSide;

  //buttons and progs
  int rollerIn = 2, rollerOut = 3, hatchPickupP = 2, hatchPickupF, hatchPlaceH = 4, hatchPlaceL = 3, hatchPlaceC = 1; //leftTrigger, rightTrigger, B
  int climbMotor, cargoPickupF, cargoPickupP, cargoPlaceRL, cargoPlaceRH, cargoPlaceC;
  int elevatorH, elevatorL;

  int progHatchPickupP = 0, progHatchPickupF = 1, progHatchPlaceH = 2, progCargoPickupF = 3;//array indices for inProgresses
  int progCargoPlaceRL = 4, progCargoPlaceRH = 5, progCargoPlaceC = 6, progClimbMotor = 7;
  int progClimbWinch = 8, progElevatorH = 9, progElevatorL = 10, progHatchPlaceL = 11;
  int progCargoPickupP = 12, progHatchPlaceC = 13;
  boolean progHatchPickupF2, progCancel = false;
  boolean stopDrive = false;

  boolean[] inProgresses = new boolean[] {false, false, false, false, false, false, false, false, false, false, false, false, false, false};
  
  boolean letUpB = true, letUpClimb = true, letUpHatchPickupF = true, letUpCargoPickupF = true, letUpCargoPickupP = true, letUpCargoPlaceC = true, letUpCargoPlaceRL = true, letUpCargoPlaceRH = true;//used for confirmation press
 
  int progressCount;

  //PID
  int elevatorTuner = 0, wristTuner = 1;
  int elevatorCycleCount = 0, wristCycleCount = 0;
  boolean letUpChangePID = true, letUpCompute = true, progCompute = false, letUpWristCycle = true, letUpElevatorCycle = true;

  double elevatorP, elevatorI, elevatorD, wristP = .0095, wristI = 0.0000101, wristD = 0.01;
  SRF_PID[] pids = new SRF_PID[] {new SRF_PID(joyTune,elevatorP,elevatorI,elevatorD), new SRF_PID(joyTune,wristP,wristI,wristD)};

  int pidCount = 1;
  double targetPositionElevator = 0;
  double targetPositionWrist = 0;
  double targetPositionWristTemp = 0;

  Timer wristPosition = new Timer();

  //misc
  double elevatorInput, wristInput;
  NetworkTable table;
  DigitalOutput testytest = new DigitalOutput(5);
  boolean boolTest = false;

  //place holders
  int elevatorNum = 1, rollerNum = 7, wristNum = 2, railNum = 0, vacuumPumpNum = 4, vacuumSensorNum = 0, isolationRelayNum = 0, bleedRelayNum = 1, climberArmNum = 6, climberWinchNum = 5, elevatorHighNum = 0, elevatorLowNum = 1, vacuumThreshold = 700/*(?)*/, vacuumHatchThreshold = 800, vacuumReleaseThreshold = 2500, railIn = 0, railOut = 214000;
  double elevatorHighPosition/*cargo/hatch middle*/, elevatorMiddlePosition/*cargo cargoship */, elevatorSemiLowPosition/*prevent scraping off suction cups for floor pickup*/, elevatorLowPosition/*hatch floor/playerstation pickup, hatch place cargoship/low rocket*/, wristUltraLowPosition/* cargo pickup floor*/, wristLowPosition = 116000/*hatch pickup floor*/, wristMiddlePosition = 56000/* cargo low rocket place*/, wristHighPosition = 0 /*hatch cargoship/low rocket, cargo low*/;
  double wristTolerance = 9001, elevatorTolerance, railTolerance;

  Timer rumbleTimer = new Timer();
  Timer timeoutTimer = new Timer();
  double startTime;
  boolean progCancelfirstTime = true;

  
  //enable flags
  private static final boolean elevatorEnable = false;//a boolean that enables and disables the elevator motion (to handle pre-elevator movement)
  private static final boolean wristEnable = true;
  private static final boolean railEnable = true;
  
  private static final boolean basicMode = true;
  private static final boolean testMode = false;
  private static final boolean tuneMode = false;
  private static final boolean homeMode = false;//resets encoders in teleop Periodic
  private final boolean debug = false;
  private final boolean wristEnablePID = false;

  private int testSystem = 0;
  boolean letUpSystem = true;
  double testAxis;
  int targetRailPosition = railIn;
  boolean relayOn = false;
  boolean continuedLetUp = false;
  

  double vacuumValue;
  int progIndex;

  boolean inTransition;
  boolean letUpY;

  boolean recharging;

  @Override
  public void robotInit() {
    elevatorHigh = new DigitalInput(elevatorHighNum);
    elevatorLow = new DigitalInput(elevatorLowNum);

    j = new Joystick(0);
    if(tuneMode)
      joyTune = new Joystick(1);

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
    wrist.setSelectedSensorPosition(0);

    elevator = new TalonSRX(elevatorNum);
    elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    elevator.config_kP(0, elevatorP);
    elevator.config_kI(0, elevatorI);
    elevator.config_kD(0, elevatorD);
    elevator.setSelectedSensorPosition(0);

    climberArm = new VictorSP(climberArmNum);

    climberWinch = new Victor(climberWinchNum);

    vacuumPump = new Spark(vacuumPumpNum);

    //Spike direction choosen at random, may need to be changed/not be neccasary
    //Written with following presumption:
    //isolationRelay forward = isolation valve for manipulater
    //isolationRelay reverse = isolation valve for climber
    //bleedRelay forward = manipulater release valve
    //bleedRelay reverse = extra channel (is currently disabled as bleedRelay can only be run Forwards at the moment)
    isolationRelay = new Relay(isolationRelayNum, Direction.kBoth);
    bleedRelay = new Relay(bleedRelayNum/*, Relay.Direction.kForward*/);

    bleedRelay.set(Value.kOff);
    isolationRelay.set(Value.kOff);

    leftSide = new Encoder(2,3);
    vacuumSensor = new AnalogInput(vacuumSensorNum);
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
  }

  @Override
  public void autonomousInit() {
    recharging = false;
  }

  @Override
  public void autonomousPeriodic() {
    if(basicMode)
      SRF_Basic();
    else
      SRF_Control();

      SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());
  }

  //starts the timer that controls the rumbler
  void startRumbleTimer() {
    rumbleTimer.reset();
    rumbleTimer.start();
  }


  //starts the timer that controls when to quit a task
  void startTimeoutTimer() {
    timeoutTimer.reset();
    timeoutTimer.start();
  }

  @Override
  public void teleopInit() {
    recharging = false;
    letUpY = true;
    inTransition = false;
    if(homeMode){
      wrist.setSelectedSensorPosition(0);
      elevator.setSelectedSensorPosition(0);
      rail.setSelectedSensorPosition(0);
    }
     j.setRumble(RumbleType.kLeftRumble, 0);
    progressCount = 0;
  }

  @Override
  public void teleopPeriodic() {
    if(testMode)//SRF_PID get the kP out of that for talon tuning
      SRF_Test();
    else if(basicMode)
      SRF_Basic();
    else
      SRF_Control();

      

 /*   SmartDashboard.putBoolean("Computing?", progCompute);
    SmartDashboard.putNumber("Elevator Cycle Number",elevatorCycleCount);
    SmartDashboard.putNumber("Wrist Cycle Number",wristCycleCount);*/  
    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());
    /*SmartDashboard.putBoolean("progCancel", progCancel);
    SmartDashboard.putNumber("Wrist Encoder", wrist.getSelectedSensorPosition());*/
    SmartDashboard.putNumber("Elevator Ennoder", elevator.getSelectedSensorPosition());
    //SmartDashboard.putNumber("Rail Enocder", rail.getSelectedSensorPosition());
    /*SmartDashboard.putNumber("kP", pids[pidCount].k[0]);
    SmartDashboard.putNumber("kI", pids[pidCount].k[1]*1000);
    SmartDashboard.putNumber("kD", pids[pidCount].k[2]);
    SmartDashboard.putNumber("P Multiplier", pids[pidCount].mult[0]);
    SmartDashboard.putNumber("I Multiplier", pids[pidCount].mult[1]);
    SmartDashboard.putNumber("D Multiplier", pids[pidCount].mult[2]);
    SmartDashboard.putNumber("CurrentMode", pids[pidCount].currentMode);
    SmartDashboard.putNumber("CurrentGain", pids[pidCount].currentGain);
    if(debug)SmartDashboard.putBoolean("HatchPlaceC", inProgresses[progHatchPlaceC]);
    SmartDashboard.putNumber("targetPosition wrist", targetPositionWrist);
    SmartDashboard.putNumber("Rail Encoder", rail.getSelectedSensorPosition());
*/
    //testytest.pulse(10);

  }

  void SRF_Basic(){

    //automated vacuum pump
    if(!recharging && vacuumSensor.getValue() > vacuumHatchThreshold)
      recharging = true;

    if(recharging){
      if(vacuumSensor.getValue() < vacuumThreshold) {
        recharging = false;
        vacuumPump.set(0);
      }
      else
        vacuumPump.set(1);
    }
    else
      vacuumPump.set(0);

    if(Math.abs(j.getRawAxis(0)) > 0.1 || Math.abs(j.getRawAxis(1)) > 0.1)
      if(j.getRawButton(2)) 
        robot.arcadeDrive(.8*j.getRawAxis(1),0.5*j.getRawAxis(0));
      else
        robot.arcadeDrive(j.getRawAxis(1),0.8*j.getRawAxis(0));
    else
      robot.arcadeDrive(0, 0);

    //left trigger - roller in, right trigger - roller out
    if(j.getRawAxis(rollerIn) > 0.3) {
      roller.set(0.4);
    } else if(j.getRawAxis(rollerOut) > 0.3) {
      roller.set(-0.65);
    } else{
      roller.set(0);
    }
    

    /*if(Math.abs(j.getRawAxis(5)) > 0.2 && wristEnable)
      wrist.set(ControlMode.PercentOutput, -0.5*j.getRawAxis(5));
    else
      wrist.set(ControlMode.PercentOutput, 0);*/

    //Newy
    if(j.getRawButton(3) && !(targetPositionWrist <= 0))
      targetPositionWrist += 100;
    if(j.getRawButton(4) && !(targetPositionWrist >= wristLowPosition))
      targetPositionWrist -= 100;
    wrist.set(ControlMode.Position, targetPositionWrist);

    //A - bleed valve
    if(j.getRawButton(1))
      bleedRelay.set(Value.kForward);
    else
      bleedRelay.set(Value.kOff);
    
    //B - Vaccum pump
    //Will also need to set up climber switching logic
    //use the second controller to switch isolation
    if(j.getRawButton(2))
      isolationRelay.set(Value.kForward);
    else
      isolationRelay.set(Value.kOff);
   /* if(j.getRawButton((2)))
      vacuumPump.set(1);
    else
      vacuumPump.set(0);*/

    //leftBumper - Down, rightBumper - Up
    if(j.getRawButton(5) && elevator.getSelectedSensorPosition() < 0) 
      elevator.set(ControlMode.PercentOutput, .3);//elevator down
    else if(j.getRawButton(6))
      elevator.set(ControlMode.PercentOutput, -.5);//elevator up
    else
      elevator.set(ControlMode.PercentOutput, 0);

    //start, back - Rail
      if(railEnable && j.getRawButton(7) /*&& rail.getSelectedSensorPosition() > railIn*/)
        rail.set(ControlMode.PercentOutput, 0.55);
      else if(railEnable && j.getRawButton(8) /*&& rail.getSelectedSensorPosition() < railOut*/)
        rail.set(ControlMode.PercentOutput, -0.25);
      else
        rail.set(ControlMode.PercentOutput, 0);
  }

  void SRF_Test(){
    //robot.arcadeDrive(j.getRawAxis(1),j.getRawAxis(0));

    if(j.getRawButton(6) && !tuneMode) 
      frontRight.set(.3);
    else
      frontRight.set(0);

    if(j.getRawButton(5)&& !tuneMode) 
      rearRight.set(.3);
    else
      rearRight.set(0);

    if(j.getRawButton(9))
      frontLeft.set(0.3);
    else
      frontLeft.set(0);

    if(j.getRawButton(10))
      rearLeft.set(0.3);
    else
      rearLeft.set(0);

   //roller code - may need to be reversed
   if(j.getRawAxis(rollerIn) > 0.3) {
    roller.set(0.35);
  } else if(j.getRawAxis(rollerOut) > 0.3) {
    roller.set(-0.55);
  } else{
    roller.set(0);
  }

   /* if(j.getRawButton(1) && letUpSystem) {
      testSystem++;
      if(testSystem == 6)
        testSystem = 0;
      letUpSystem = false;
    }
    else if(!j.getRawButton(1))
      letUpSystem = true;

    testAxis = j.getRawAxis(5);
    if(Math.abs(testAxis) > 0.2)
    {
      //if(testSystem==0)
        //climberArm.set(testAxis);
      //else if(testSystem==1 && wristEnable)
        //wrist.set(ControlMode.PercentOutput, testAxis);
      /*else*//* if(testSystem==2)
        roller.set(testAxis);
      else if(testSystem==3 && elevatorEnable)
        elevator.set(ControlMode.PercentOutput,testAxis);
      else if(testSystem==4 && railEnable)
        rail.set(ControlMode.PercentOutput,testAxis);
      else if(testSystem==5)
        vacuumPump.set(testAxis);
      


    }else {
      climberArm.set(0);
      wrist.set(ControlMode.PercentOutput, 0);
      roller.set(0);
      elevator.set(ControlMode.PercentOutput,0);
      rail.set(ControlMode.PercentOutput,0);
      vacuumPump.set(0);
    }*/

    /*if(j.getRawButton(5))
      climberArm.set(1);
    else
      climberArm.set(0);
*/
    if(j.getRawButton(6) && tuneMode)
      bleedRelay.set(Value.kForward);
    else
      bleedRelay.set(Value.kOff);

    if(!tuneMode)
    {
      if(j.getRawButton(2))
        isolationRelay.set(Value.kReverse);
      else if(j.getRawButton(3))
        isolationRelay.set(Value.kForward);
      else
        isolationRelay.set(Value.kOff);
    }

  /*  if(j.getRawButton(4))
      vacuumPump.set(1);
    else
      vacuumPump.set(0);
*/

   /* if(Math.abs(j.getRawAxis(5)) > 0.2 && wristEnable)
      wrist.set(ControlMode.PercentOutput, 0.5*j.getRawAxis(5));
    else
      wrist.set(ControlMode.PercentOutput, 0);
*/
/*  if(relayOn)
    bleedRelay.set(Value.kForward);
  else
    bleedRelay.set(Value.kOff);
*/
    if(tuneMode && j.getRawButton(3) && letUpCompute) {
      progCompute = true;
      letUpCompute = false;
    } else if (tuneMode && !j.getRawButton(3)) {
      letUpCompute = true;
      progCompute = false;
      wrist.set(ControlMode.PercentOutput, 0);
    }

    if(progCompute) {
      if(pidCount == 0) {
        elevator.config_kP(0, pids[0].k[0]);
        elevator.config_kI(0, pids[0].k[1]);
        elevator.config_kD(0, pids[0].k[2]);
        elevator.set(ControlMode.Position, targetPositionElevator);
      } else {
        wrist.config_kP(0, pids[1].k[0]);
        wrist.config_kI(0, pids[1].k[1]);
        wrist.config_kD(0, pids[1].k[2]);
        wrist.set(ControlMode.Position, targetPositionWrist);
      }
    }

    if(j.getRawButton(7) && letUpElevatorCycle) {
      letUpElevatorCycle = false ;
        if(elevatorCycleCount == 3)
          elevatorCycleCount = -1;
        elevatorCycleCount++;
        if(elevatorCycleCount == 0)
          targetPositionElevator = elevatorHighPosition;
        else if(elevatorCycleCount == 1)
          targetPositionElevator = elevatorMiddlePosition;
        else if(elevatorCycleCount == 2)
          targetPositionElevator = elevatorSemiLowPosition;
        else if(elevatorCycleCount == 3)
          targetPositionElevator = elevatorLowPosition;
    } else if(!j.getRawButton(7)) {
      letUpElevatorCycle = true; ;
    }

    if(j.getRawButton(8) && letUpWristCycle) {
      letUpWristCycle = false;
      if(wristCycleCount == 3)
        wristCycleCount = -1;
      wristCycleCount++;
      if(wristCycleCount == 0)
          targetPositionWristTemp = wristHighPosition;
        else if(wristCycleCount == 1)
          targetPositionWristTemp = wristMiddlePosition;
        else if(wristCycleCount == 2)
          targetPositionWristTemp = wristLowPosition;
        else if(wristCycleCount == 3)
          targetPositionWristTemp = wristUltraLowPosition;
    } else if(!j.getRawButton(8)) {
      letUpWristCycle = true ;
    }
    
    //MAYBE FOR SRF_CONTROL (well bits of it anyway)
    /////////////////////////////////////////////////////////
    
    if(j.getRawButton(4) && letUpY){
      letUpY = false;
      if(targetPositionWristTemp == wristMiddlePosition || Math.abs(wrist.getSelectedSensorPosition()-wristMiddlePosition) < wristTolerance){
        wrist.setIntegralAccumulator(0);
        targetPositionWrist = targetPositionWristTemp;
      }
      else{
        wrist.setIntegralAccumulator(0);
        targetPositionWrist = wristMiddlePosition;
        inTransition = true;
      }
    }
    else if(!j.getRawButton(4))
      letUpY = true;

    if(inTransition && Math.abs(wrist.getSelectedSensorPosition()-wristMiddlePosition) < wristTolerance)
    {
      wrist.setIntegralAccumulator(0);
      inTransition = false;
      targetPositionWrist = targetPositionWristTemp;
    }

    /////////////////////////////////////////////////////////

    if(j.getRawButton(1) && letUpChangePID) {
      if(pidCount == 1)
        pidCount = -1;
      pidCount++;
      letUpChangePID = false;        
    } else if(!j.getRawButton(1)) {
      letUpChangePID = true;
    }

    
   /* if(railEnable && j.getRawButton(9) && rail.getSelectedSensorPosition() < railOut)
      rail.set(ControlMode.PercentOutput, 0.25);
    else if(railEnable && j.getRawButton(10) && rail.getSelectedSensorPosition() > railIn)
      rail.set(ControlMode.PercentOutput, -0.45);
    else
      rail.set(ControlMode.PercentOutput, 0);
*/

    if(tuneMode)
      pids[pidCount].controlPID();    

    SmartDashboard.putNumber("Temp Target Wrist", targetPositionWristTemp);
    SmartDashboard.putNumber("testSystem",testSystem);
    SmartDashboard.putBoolean("inTransition", inTransition);
  }


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


  //the thing we use to drive the robot
  public void SRF_Control() {
    
    //drive code
    if(!stopDrive)
      robot.arcadeDrive(j.getRawAxis(1),0.8*j.getRawAxis(0));

    //roller code - may need to be reversed
    if(j.getRawAxis(rollerIn) > 0.3) {
      roller.set(0.35);
    } else if(j.getRawAxis(rollerOut) > 0.3) {
      roller.set(-0.35);
    } else{
      roller.set(0);
    }

    //manipulator code

    if(j.getRawButton(hatchPickupP))
      continuedLetUp = false;

    //Pickup Hatch (Player Station) #1
    if(j.getRawButton(hatchPickupP) && !inProgresses[progHatchPickupP] && letUpB){//If B is pressed
      if(debug)System.out.println("initial button press");
      inProgresses[progHatchPickupP] = true;
      letUpB = false;
    }
    else if(j.getRawButton(hatchPickupP) && inProgresses[progHatchPickupP] && letUpB){
      progCancel = true;
      if(debug)System.out.println("has been canceled");
    }
    else if(!j.getRawButton(hatchPickupP)){
      letUpB = true;
      if(!continuedLetUp && debug)
        System.out.println("button has been letup");
      continuedLetUp = true;
    }

    if(inProgresses[progHatchPickupP]) {
      isolationRelay.set(Value.kForward);//open valve to manipulator
      vacuumValue = vacuumSensor.getValue();

      if(vacuumValue < vacuumHatchThreshold) {
        if(debug)System.out.println("done:"+vacuumValue);
        startRumbleTimer();
        inProgresses[progHatchPickupP] = false;
        progCancel = true;
      }
    }
            //if the operation hasn't been canceled-continue
            //apply slight forward motion??? (to allign with the wall) - probably should be implemented elsewhere
            //if we have vacuum, send success code
            //or if there is a timeout and we still don't have vacuum, cancel and send error message and return to standard running

    //we can only do floor pickup if the elevator is enabled

    //BRB //#2
    //Pickup Hatch (Floor) - I made a lovely tree :)))))))))))
/*    if(j.getRawButton(hatchPickupF) && !inProgresses[progHatchPickupF] && elevatorEnable && wristEnable && letUpHatchPickupF) {
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
    if(progHatchPickupF2) {
      elevator.set(ControlMode.Position, elevatorLowPosition);

      if(Math.abs(elevator.getSelectedSensorPosition()) < elevatorTolerance) {
        isolationRelay.set(Relay.Value.kForward);

        if(vacuumSensor.getValue() < vacuumHatchThreshold) {
        startRumbleTimer();
          inProgresses[progHatchPickupF] = false;
          progHatchPickupF2 = false;
          progCancel = true;
        }
      }
    }
 */           //if it hasn't been canceled-you may proceed
            //override movement probably - again, should probably be implemented elsewhere
            //flop wrist down
            //if wrist is down, lower the elevator to the proper spot
            //if the list, establish vacuum
            //if the new list, return to standard running
            //if we timeout and it isn't working - "Bad News Bears" (return to standard running)
    
    //Place Hatch (cargoship)
    if(j.getRawButton(hatchPlaceC) && !inProgresses[progHatchPlaceC]) {
      if(debug)System.out.println("A was pressed");
      inProgresses[progHatchPlaceC] = true;
    }
    if(inProgresses[progHatchPlaceC]) {
      isolationRelay.set(Relay.Value.kForward);
      bleedRelay.set(Relay.Value.kForward);

      if(vacuumSensor.getValue() > vacuumReleaseThreshold) {//We might want this to be vacuum threshold but I'm not sure yet (also at tag RAS)-Scott
      if(debug)System.out.println("Done placing on cargoship");
        inProgresses[progHatchPlaceC] = false;
        progCancel = true;
      }
    }

        //BRB
        //Place Hatch (low)/* #3
    if(j.getRawButton(hatchPlaceL) && !inProgresses[progHatchPlaceL] && railEnable) {
      inProgresses[progHatchPlaceL] = true;
    }
    if(inProgresses[progHatchPlaceL]) {
      targetRailPosition = railOut;

      if(Math.abs(rail.getSelectedSensorPosition() - railOut) < railTolerance)
      {
        isolationRelay.set(Relay.Value.kForward);
        bleedRelay.set(Relay.Value.kForward);

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

    //BRB
    //place hatch (high) #4
  /*  if(elevatorEnable && j.getRawButton(hatchPlaceH) && !inProgresses[progHatchPlaceH] && elevatorEnable && railEnable) { //RAS
      inProgresses[progHatchPlaceH] = true;
    }
    if(inProgresses[progHatchPlaceH]) {
      if(elevatorEnable)
        elevator.set(ControlMode.Position, elevatorHighPosition);

      if(!elevatorEnable || Math.abs(elevator.getSelectedSensorPosition()-elevatorHighPosition) < elevatorTolerance) {
        targetRailPosition = railOut;

        if(Math.abs(rail.getSelectedSensorPosition() - railOut) < elevatorTolerance) {
          bleedRelay.set(Relay.Value.kForward);
          isolationRelay.set(Relay.Value.kForward);

          if(vacuumSensor.getValue() > vacuumHatchThreshold) {
            inProgresses[progHatchPlaceH] = false;
            progCancel = true;
          }
        }
      }
    }*/

    /////////////////////////////////////////////////////
    //I'm thinking that maybe these should be manually selected modes that just changes these operational positions 
    //Or maybe it shouldn't ever be manually changed? 
    
    //BRB
    //Pickup Cargo (Floor) #5
    /*if(j.getRawButton(cargoPickupF) && !inProgresses[progCargoPickupF] && elevatorEnable && wristEnable) {
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
    }*/

    //XXX - currently left out in favor of cargoship placement
    //Pickup cargo (Player station) #6
    /*if(j.getRawButton(cargoPickupP) && !inProgresses[progCargoPickupP] && elevatorEnable && wristEnable) {
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
    }*/
    //BRB - rail
    //Place Cargo (Rocket Low) #7
    /*if(j.getRawButton(cargoPlaceRL) && !inProgresses[progCargoPlaceRL] && letUpCargoPlaceRL && railEnable && wristEnable) {
      inProgresses[progCargoPlaceRL] = true;
      letUpCargoPlaceRL = false;
    } else if(!j.getRawButton(cargoPlaceRL)) {
      letUpCargoPlaceRL = true;
    }
    if(inProgresses[progCargoPlaceRL]) {
      targetRailPosition = railOut;
      if(Math.abs(rail.getSelectedSensorPosition() - railOut) < railTolerance) {
        wrist.set(ControlMode.Position,wristMiddlePosition);
        if(Math.abs(wrist.getSelectedSensorPosition() - wristMiddlePosition) < wristTolerance) {

          if(j.getRawButton(cargoPlaceRL) && letUpCargoPlaceRL) {
            inProgresses[progCargoPlaceRL] = false;
            progCancel = true;
          }
        }
      }
    }*/
        //proper elevator height
        //flop wrist

    //BRB
    //Place Cargo (Rocket High) #8
    /*if(j.getRawButton(cargoPlaceRH) && !inProgresses[progCargoPlaceRH] && letUpCargoPlaceRH && elevatorEnable && railEnable) {
      inProgresses[progCargoPlaceRH] = true;
      letUpCargoPlaceRH = false;
    } else if(!j.getRawButton(cargoPlaceRH)) {
      letUpCargoPlaceRH = true;
      
    }
    if(inProgresses[progCargoPlaceRH]) {
      elevator.set(ControlMode.Position, elevatorHighPosition);
      if(Math.abs(elevator.getSelectedSensorPosition() - elevatorHighPosition) < elevatorTolerance) {
        targetRailPosition = railOut;
        if(Math.abs(rail.getSelectedSensorPosition() - railOut) < elevatorTolerance) {
          if(j.getRawButton(cargoPlaceRH) && letUpCargoPlaceRH) {
           inProgresses[progCargoPlaceRH] = false;
           progCancel = true;
          }
        }
      }
    }*/

    //BRB
    //Place Cargo (Cargo Ship) #9
    /*if(j.getRawButton(cargoPlaceC) && !inProgresses[progCargoPlaceC] && letUpCargoPlaceC && elevatorEnable && wristEnable) {
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
    }*/

/////////////////////////////////////////////////////////
    
    //BRB
    //valve (climber and grip) code #10

      /*if(j.getRawButton(climbMotor) && !inProgresses[progClimbMotor] && letUpClimb) {
        inProgresses[progClimbMotor] = true;
        letUpClimb = false;
      } else if(!j.getRawButton(climbMotor)) {
        letUpClimb = true;
      }

      if(inProgresses[progClimbMotor]){
        
        climberArm.set(j.getRawAxis(5));
        isolationRelay.setDirection(Relay.Direction.kReverse);
        bleedRelay.set(Relay.Value.kForward);
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
      }*/

      progIndex = 0;
      progressCount = 0;
      //Cancel + Reset
      for(boolean b: inProgresses) {//does inProgresses ever actually get updated after initialization? If not this might not work
        if(b) {
          //if(debug)System.out.println(":"+progIndex);
          progressCount++;
        }
        if(progressCount == 2) {
          if(debug)System.out.println("extra progresses detected");
          progCancel = true;
          break;
        }
        progIndex++;
      }


      if(progCancel) { //add timeout here - XXX
        if(progCancelfirstTime) {
          progCancelfirstTime = false;
          timeoutTimer.start();
          startTime = timeoutTimer.get();
        }
        if(timeoutTimer.get() - startTime > 20) {
          progCancel = false;
          progCancelfirstTime = true;
          timeoutTimer.stop();
          timeoutTimer.reset();
          SmartDashboard.putString("WARNING ERROR","TIMEOUT IN ProgCancel");
        }
          
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
        inProgresses[progHatchPlaceC] = false;

        if(debug)System.out.println("progCancel");
        isolationRelay.set(Value.kForward);
        bleedRelay.set(Relay.Value.kOff);
        //set the set points back to standard running mode(finishlogic making them return to default in order(wrist, rail, elevator))
        if(wristEnable){
          if(wristEnablePID)
            wrist.set(ControlMode.Position, wristHighPosition);
          else{
            if(Math.abs(j.getRawAxis(5)) > 0.2)
              wrist.set(ControlMode.PercentOutput, 0.5*j.getRawAxis(5));
            else
              wrist.set(ControlMode.PercentOutput, 0);
          }
        }

        if(railEnable && Math.abs(wrist.getSelectedSensorPosition() - wristHighPosition) < wristTolerance) {
          targetRailPosition = railIn;
            
          if(Math.abs(rail.getSelectedSensorPosition() - railIn) < railTolerance) {
            elevator.set(ControlMode.Position, elevatorLowPosition);
            
            if(Math.abs(elevator.getSelectedSensorPosition() - elevatorLowPosition) < elevatorTolerance) {
              progCancel = false;
            }
          }
        }
        else if(!railEnable){
          if(debug)System.out.println("rail stuff skipped");
          if(elevatorEnable) {
            elevator.set(ControlMode.Position, elevatorLowPosition);
            
            if(Math.abs(elevator.getSelectedSensorPosition() - elevatorLowPosition) < elevatorTolerance) 
              progCancel = false;
              progCancelfirstTime = true;
              timeoutTimer.stop();
              timeoutTimer.reset();
          }
          else{
            progCancel = false;
            if(debug)System.out.println("elevator skipped: done");
          }
        }
      }
/*
      //controller rumbles for half a second after starRumbleTimer is called
      if(rumbleTimer.get() < 0.5)
        j.setRumble(RumbleType.kLeftRumble, 0.5);
      else
      {
        j.setRumble(RumbleType.kLeftRumble, 0);
        rumbleTimer.stop();
      }*/

      //vacuum - presuming max speed?
      if(vacuumSensor.getValue() > vacuumThreshold) {
        vacuumPump.set(1.0);
      }
      else
        vacuumPump.set(0);
    
/*//BRB
      if(Math.abs(rail.getSelectedSensorPosition() - targetRailPosition) < elevatorTolerance) {
        rail.set(ControlMode.PercentOutput, 0);
      } else if(rail.getSelectedSensorPosition() < targetRailPosition) {
        rail.set(ControlMode.PercentOutput, .35);
      } else if(rail.getSelectedSensorPosition() > targetRailPosition){
        rail.set(ControlMode.PercentOutput, -.35);
      }*/

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
