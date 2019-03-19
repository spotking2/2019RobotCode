
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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


public class Robot extends TimedRobot {//v1.6.4b
  /*
    added some new functionality (elevator raising when wrist goes down and some other stuff)
    also added trouble shooting mini over run timer and did a partial code review  
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
  //These were taken out when we added the untested DIO support
  //Relay isolationRelay;
  //Relay bleedRelay;

  SpeedControllerGroup left;
  SpeedControllerGroup right;

  DifferentialDrive robot;
  Joystick j;
  Joystick joyTune = new Joystick(1);

  //Sensors
  DigitalInput elevatorHigh;
  DigitalInput elevatorLow;
  
  //Encoders are currently unimplemented
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

  boolean progSmallElevatorRaise = false;

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
  DigitalOutput bleedTest = new DigitalOutput(6);
  boolean boolTest = false;

  DigitalOutput bleedOutput = new DigitalOutput(7);
  DigitalOutput isolateManipulator = new DigitalOutput(8);
  DigitalOutput isolateClimber = new DigitalOutput(9);

  //place holders
  int elevatorNum = 1, rollerNum = 7, wristNum = 2, railNum = 0, vacuumPumpNum = 4, vacuumSensorNum = 0, isolationRelayNum = 0, bleedRelayNum = 1, climberArmNum = 6, climberWinchNum = 5, elevatorHighNum = 0, elevatorLowNum = 1, vacuumThreshold = 1000/*(?)*/, vacuumHatchThreshold = 2000, vacuumReleaseThreshold = 2500, railIn = 0, railOut = 500000;
  double elevatorHighPosition/*cargo/hatch middle*/, elevatorMiddlePosition/*cargo cargoship */, elevatorSemiLowPosition/*prevent scraping off suction cups for floor pickup*/, elevatorLowPosition/*hatch floor/playerstation pickup, hatch place cargoship/low rocket*/, wristUltraLowPosition/* cargo pickup floor*/, wristLowPosition = 116000/*hatch pickup floor*/, wristMiddlePosition = 56000/* cargo low rocket place*/, wristHighPosition = 0 /*hatch cargoship/low rocket, cargo low*/;
  double wristTolerance = 9001, elevatorTolerance, railTolerance;

  Timer rumbleTimer = new Timer();
  Timer timeoutTimer = new Timer();
  Timer loopTimer = new Timer();
  double lastLoopTime;
  long SRF_OverrunCount; //this should serve as a rough metric to gauge what's causing loop time to spike
  double startTime;
  boolean progCancelfirstTime = true;

  
  //enable flags
  private static final boolean elevatorEnable = false;//a boolean that enables and disables the elevator motion (to handle pre-elevator movement)
  private static final boolean wristEnable = true;
  private static final boolean railEnable = true;
  
  private static final boolean testMode = false;
  private static final boolean tuneMode = true;//should be left on as long as we're in basic to facilitate climbing
  private static final boolean homeMode = true;//resets encoders in teleop Periodic, may need to be true when testing outside of match play
  private final boolean debug = false;
  private final boolean wristEnablePID = false;

  private int testSystem = 0;
  boolean letUpSystem = true;
  double testAxis;
  int targetRailPosition = railIn;
  boolean relayOn = false;
  boolean continuedLetUp = false;
  
  boolean bleedIsSet;

  double vacuumValue;
  int progIndex;

  boolean inTransition;
  boolean letUpY;

  boolean recharging;

  boolean isolationHatch;
  boolean isolationClimb;

  boolean vacuumAchieved;

  boolean disableVacPump = false;
  boolean elevatorRaiseDone = false;
  int elevatorStopPos = 0;
  boolean elevatorMoving = false;

  boolean closeEnough;

  @Override
  public void robotInit() {
    elevatorHigh = new DigitalInput(elevatorHighNum);
    elevatorLow = new DigitalInput(elevatorLowNum);

    j = new Joystick(0);
    if(tuneMode)//no issue with climber as long as we leave it on and stay in basic
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
    elevator.setNeutralMode(NeutralMode.Brake);

    climberArm = new VictorSP(climberArmNum);

    climberWinch = new Victor(climberWinchNum);

    vacuumPump = new Spark(vacuumPumpNum);

    //All the relay stuff is currently obselete because DIO

    //Spike direction choosen at random, may need to be changed/not be neccasary
    //Written with following presumption:
    //isolationRelay forward = isolation valve for manipulater
    //isolationRelay reverse = isolation valve for climber
    //bleedRelay forward = manipulater release valve
    //bleedRelay reverse = extra channel (is currently disabled as bleedRelay can only be run Forwards at the moment)
    //isolationRelay = new Relay(isolationRelayNum, Direction.kBoth);
    //bleedRelay = new Relay(bleedRelayNum/*, Relay.Direction.kForward*/);

    //setBleed(Value.kOff);//bleedRelay.set(Value.kOff);
    //isolationRelay.set(Value.kOff);

    leftSide = new Encoder(2,3);
    vacuumSensor = new AnalogInput(vacuumSensorNum);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void autonomousInit() {
    closeEnough = false;
    bleedIsSet = false;
    recharging = false;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void autonomousPeriodic() {
    SRF_Basic();

    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());  
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////

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


 public void setBleed(boolean v){
    bleedTest.pulse(1000);
    //bleedTest.set(true);
    //bleedRelay.set(bleeding);
    bleedOutput.set(v);
    //bleedTest.set(false);
  }


  public void setIsolation(boolean manV, boolean climbV) {
    isolateManipulator.set(manV);
    isolateClimber.set(climbV);
  }


 ////////////////////////////////////////////////////////////////////////////////////////////////////////


  @Override
  public void teleopInit() {
    readyToClimb = false;

    loopTimer.reset();
    loopTimer.start();
    SRF_OverrunCount = 0;

    closeEnough = false;
    bleedIsSet = false;
    recharging = false;
    letUpY = true;
    inTransition = false;
    if(homeMode){
      wrist.setSelectedSensorPosition(0);
      elevator.setSelectedSensorPosition(0);
      rail.setSelectedSensorPosition(0);
      SRF_OverrunCount = 0;
    }
     j.setRumble(RumbleType.kLeftRumble, 0);
    progressCount = 0;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

 

  @Override
  public void teleopPeriodic() {
    lastLoopTime = loopTimer.get();

    testytest.set(true);
    if(testMode)//SRF_PID get the kP out of that for talon tuning
      SRF_Test();
    else
      SRF_Basic();
      

 /*   SmartDashboard.putBoolean("Computing?", progCompute);
    SmartDashboard.putNumber("Elevator Cycle Number",elevatorCycleCount);
    SmartDashboard.putNumber("Wrist Cycle Number",wristCycleCount);*/  
    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());
    //SmartDashboard.putBoolean("progCancel", progCancel);
    SmartDashboard.putNumber("Wrist Encoder", wrist.getSelectedSensorPosition());
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
  */SmartDashboard.putNumber("Rail Encoder", rail.getSelectedSensorPosition());
    SmartDashboard.putNumber("Overrun count", SRF_OverrunCount);
    SmartDashboard.putBoolean("Correct Vacuum level", vacuumAchieved);
    SmartDashboard.putBoolean("Close Enough", closeEnough);
    //SmartDashboard.putBoolean("progSmallElevatorRaise", progSmallElevatorRaise);
    //SmartDashboard.putBoolean("elevatorRaiseDone", elevatorRaiseDone);

    testytest.set(false);
    
    if(loopTimer.get() > 0.02)
      SRF_OverrunCount++;
  }


  void SRF_Basic(){
    //setIsolation(true, false);

    if(Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)) < 4)
      closeEnough = true;
    else
      closeEnough = false;    


    if(tuneMode) {
      if(joyTune.getRawButton(1))//press A to open climber
        setIsolation(false, true);

      if(joyTune.getRawAxis(2) > 0.2) {
        //they move one way
      }
      else if(joyTune.getRawAxis(3) > 0.2) {
        //they move the other way
      }
      else {
        if(Math.abs(joyTune.getRawAxis(1)) > 0.1)
          climberArm.set(joyTune.getRawAxis(1));
        else
          climberArm.set(0);
        
        if(Math.abs(joyTune.getRawAxis(5)) > 0.1)
          climberWinch.set(joyTune.getRawAxis(5));
        else
          climberWinch.set(0);
      }
    }

    //automated vacuum pump
    if(!recharging && vacuumSensor.getValue() > vacuumHatchThreshold && !disableVacPump)
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
    
    if(Math.abs(j.getRawAxis(0)) > 0.1 || Math.abs(j.getRawAxis(1)) > 0.1) {
      //if(j.getRawButton(2)) //remember to change when B gets messed with, also may just need it straight up taken out for practice bot
      //  robot.arcadeDrive(.8*j.getRawAxis(1),0.5*j.getRawAxis(0));
      //else
        robot.arcadeDrive(j.getRawAxis(1),0.8*j.getRawAxis(0));
    } else
      robot.arcadeDrive(0, 0);

    //left trigger - roller in, right trigger - roller out
    if(j.getRawAxis(rollerIn) > 0.3) {
      roller.set(0.55);
    } else if(j.getRawAxis(rollerOut) > 0.1) {
      roller.set(-j.getRawAxis(rollerOut));
    } else {
      roller.set(.1);
    }
    
    if(Math.abs(j.getRawAxis(5)) > 0.2 && wristEnable) {
      if(wrist.getSelectedSensorPosition() < 142000 || j.getRawAxis(5) > 0)
        wrist.set(ControlMode.PercentOutput, -0.3*j.getRawAxis(5));
      else
        wrist.set(ControlMode.PercentOutput, 0);

      if(!progSmallElevatorRaise && wrist.getSelectedSensorPosition() > 100000 && !elevatorRaiseDone)
        progSmallElevatorRaise = true;
    } else
      wrist.set(ControlMode.PercentOutput, 0);
    
    if(progSmallElevatorRaise) {
      if(elevator.getSelectedSensorPosition() > -120000/*target - 120000 */)
        elevator.set(ControlMode.PercentOutput, .6);
      else {
        elevator.set(ControlMode.PercentOutput, 0);
        progSmallElevatorRaise = false;
        elevatorRaiseDone = true;
      }
    } else if(elevatorRaiseDone && wrist.getSelectedSensorPosition() < 74000){
      elevatorRaiseDone = false;
    }  

    //XXX-try taking out bleedIsSet!!!
    //A - bleed valve
    if(j.getRawButton(1) && !bleedIsSet){
      setBleed(true);
      bleedIsSet = true;
    }
    else if(!j.getRawButton(1) && bleedIsSet){
      setBleed(false);
      bleedIsSet = false;
    }

    //B - Open Manipulator isolation
    if(j.getRawButton(2)) {
      setIsolation(true, false);
    } else {
      setIsolation(false,false);
    }

    //B - Vaccum pump
    /*if(j.getRawButton(2)) //This is pointless as long as we ain't got no relays
      isolationRelay.set(Value.kForward);
    else
      isolationRelay.set(Value.kOff);*/
    /*if(j.getRawButton((2)))
      vacuumPump.set(1);
    else
      vacuumPump.set(0);*/

    //leftBumper - Down, rightBumper - Up
    if(j.getRawButton(5) /*&& elevator.getSelectedSensorPosition() < 0*/) {
      elevator.set(ControlMode.PercentOutput, -.3);//elevator down
    } else if(j.getRawButton(6)){
      elevator.set(ControlMode.PercentOutput, .5);//elevator up
    } else if(!progSmallElevatorRaise) {
      elevator.set(ControlMode.PercentOutput, 0);
    }

    //start, back - Rail
      if(railEnable && j.getRawButton(7) && rail.getSelectedSensorPosition() < railOut)//railOut
        rail.set(ControlMode.PercentOutput, 0.55);
      else if(railEnable && j.getRawButton(8) /*&& rail.getSelectedSensorPosition() > 0*/)//railIn
        rail.set(ControlMode.PercentOutput, -0.6);
      else
        rail.set(ControlMode.PercentOutput, 0);

      if(vacuumSensor.getValue() < vacuumHatchThreshold)
        vacuumAchieved = true;
      else
        vacuumAchieved = false;

      
  }


  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////


  void SRF_Test(){//we can't move in this mode
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

    /*if(j.getRawButton(5))
      climberArm.set(1);
    else
      climberArm.set(0);
*/
    if(j.getRawButton(6) && tuneMode)
      setBleed(true);
    else
      setBleed(false);

    if(!tuneMode)
    {
      if(j.getRawButton(2))
        setIsolation(true, false);
      else if(j.getRawButton(3))
        setIsolation(false, true);
      else
        setIsolation(false, false);
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
      letUpElevatorCycle = false;
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

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    if(j.getRawButton(1))
      bleedOutput.set(true);
    else
      bleedOutput.set(false);
    
    if(j.getRawButton(2))
      isolateManipulator.set(true);
    else
      isolateManipulator.set(false);

    if(j.getRawButton(3))
      isolateClimber.set(true);
    else
      isolateClimber.set(false);
  }

}


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


public class Robot extends TimedRobot {//v1.6.4a
  /*
    added some new functionality (elevator raising when wrist goes down and some other stuff)
    also added trouble shooting mini over run timer and did a partial code review  
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
  //These were taken out when we added the untested DIO support
  //Relay isolationRelay;
  //Relay bleedRelay;

  SpeedControllerGroup left;
  SpeedControllerGroup right;

  DifferentialDrive robot;
  Joystick j;
  Joystick joyTune = new Joystick(1);

  //Sensors
  DigitalInput elevatorHigh;
  DigitalInput elevatorLow;
  
  //Encoders are currently unimplemented
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

  boolean progSmallElevatorRaise = false;

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
  DigitalOutput bleedTest = new DigitalOutput(6);
  boolean boolTest = false;

  DigitalOutput bleedOutput = new DigitalOutput(7);
  DigitalOutput isolateManipulator = new DigitalOutput(8);
  DigitalOutput isolateClimber = new DigitalOutput(9);

  //place holders
  int elevatorNum = 1, rollerNum = 7, wristNum = 2, railNum = 0, vacuumPumpNum = 4, vacuumSensorNum = 0, isolationRelayNum = 0, bleedRelayNum = 1, climberArmNum = 6, climberWinchNum = 5, elevatorHighNum = 0, elevatorLowNum = 1, vacuumThreshold = 1000/*(?)*/, vacuumHatchThreshold = 2000, vacuumReleaseThreshold = 2500, railIn = 0, railOut = 500000;
  double elevatorHighPosition/*cargo/hatch middle*/, elevatorMiddlePosition/*cargo cargoship */, elevatorSemiLowPosition/*prevent scraping off suction cups for floor pickup*/, elevatorLowPosition/*hatch floor/playerstation pickup, hatch place cargoship/low rocket*/, wristUltraLowPosition/* cargo pickup floor*/, wristLowPosition = 116000/*hatch pickup floor*/, wristMiddlePosition = 56000/* cargo low rocket place*/, wristHighPosition = 0 /*hatch cargoship/low rocket, cargo low*/;
  double wristTolerance = 9001, elevatorTolerance, railTolerance;

  Timer rumbleTimer = new Timer();
  Timer timeoutTimer = new Timer();
  Timer loopTimer = new Timer();
  double lastLoopTime;
  long SRF_OverrunCount; //this should serve as a rough metric to gauge what's causing loop time to spike
  double startTime;
  boolean progCancelfirstTime = true;

  
  //enable flags
  private static final boolean elevatorEnable = false;//a boolean that enables and disables the elevator motion (to handle pre-elevator movement)
  private static final boolean wristEnable = true;
  private static final boolean railEnable = true;
  
  private static final boolean testMode = false;
  private static final boolean tuneMode = true;//should be left on as long as we're in basic to facilitate climbing
  private static final boolean homeMode = true;//resets encoders in teleop Periodic, may need to be true when testing outside of match play
  private final boolean debug = false;
  private final boolean wristEnablePID = false;

  private int testSystem = 0;
  boolean letUpSystem = true;
  double testAxis;
  int targetRailPosition = railIn;
  boolean relayOn = false;
  boolean continuedLetUp = false;
  
  boolean bleedIsSet;

  double vacuumValue;
  int progIndex;

  boolean inTransition;
  boolean letUpY;

  boolean recharging;

  boolean isolationHatch;
  boolean isolationClimb;

  boolean vacuumAchieved;

  boolean disableVacPump = false;
  boolean elevatorRaiseDone = false;
  int elevatorStopPos = 0;
  boolean elevatorMoving = false;

  boolean closeEnough;

  @Override
  public void robotInit() {
    elevatorHigh = new DigitalInput(elevatorHighNum);
    elevatorLow = new DigitalInput(elevatorLowNum);

    j = new Joystick(0);
    if(tuneMode)//no issue with climber as long as we leave it on and stay in basic
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
    elevator.setNeutralMode(NeutralMode.Brake);

    climberArm = new VictorSP(climberArmNum);

    climberWinch = new Victor(climberWinchNum);

    vacuumPump = new Spark(vacuumPumpNum);

    //All the relay stuff is currently obselete because DIO

    //Spike direction choosen at random, may need to be changed/not be neccasary
    //Written with following presumption:
    //isolationRelay forward = isolation valve for manipulater
    //isolationRelay reverse = isolation valve for climber
    //bleedRelay forward = manipulater release valve
    //bleedRelay reverse = extra channel (is currently disabled as bleedRelay can only be run Forwards at the moment)
    //isolationRelay = new Relay(isolationRelayNum, Direction.kBoth);
    //bleedRelay = new Relay(bleedRelayNum/*, Relay.Direction.kForward*/);

    //setBleed(Value.kOff);//bleedRelay.set(Value.kOff);
    //isolationRelay.set(Value.kOff);

    leftSide = new Encoder(2,3);
    vacuumSensor = new AnalogInput(vacuumSensorNum);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void autonomousInit() {
    closeEnough = false;
    bleedIsSet = false;
    recharging = false;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void autonomousPeriodic() {
    SRF_Basic();

    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());  
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////

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


 public void setBleed(boolean v){
    bleedTest.pulse(1000);
    //bleedTest.set(true);
    //bleedRelay.set(bleeding);
    bleedOutput.set(v);
    //bleedTest.set(false);
  }


  public void setIsolation(boolean manV, boolean climbV) {
    isolateManipulator.set(manV);
    isolateClimber.set(climbV);
  }


 ////////////////////////////////////////////////////////////////////////////////////////////////////////


  @Override
  public void teleopInit() {
    loopTimer.reset();
    loopTimer.start();
    SRF_OverrunCount = 0;

    closeEnough = false;
    bleedIsSet = false;
    recharging = false;
    letUpY = true;
    inTransition = false;
    if(homeMode){
      wrist.setSelectedSensorPosition(0);
      elevator.setSelectedSensorPosition(0);
      rail.setSelectedSensorPosition(0);
      SRF_OverrunCount = 0;
    }
     j.setRumble(RumbleType.kLeftRumble, 0);
    progressCount = 0;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

 

  @Override
  public void teleopPeriodic() {
    lastLoopTime = loopTimer.get();

    testytest.set(true);
    if(testMode)//SRF_PID get the kP out of that for talon tuning
      SRF_Test();
    else
      SRF_Basic();
      

 /*   SmartDashboard.putBoolean("Computing?", progCompute);
    SmartDashboard.putNumber("Elevator Cycle Number",elevatorCycleCount);
    SmartDashboard.putNumber("Wrist Cycle Number",wristCycleCount);*/  
    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());
    //SmartDashboard.putBoolean("progCancel", progCancel);
    SmartDashboard.putNumber("Wrist Encoder", wrist.getSelectedSensorPosition());
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
  */SmartDashboard.putNumber("Rail Encoder", rail.getSelectedSensorPosition());
    SmartDashboard.putNumber("Overrun count", SRF_OverrunCount);
    SmartDashboard.putBoolean("Correct Vacuum level", vacuumAchieved);
    SmartDashboard.putBoolean("Close Enough", closeEnough);
    //SmartDashboard.putBoolean("progSmallElevatorRaise", progSmallElevatorRaise);
    //SmartDashboard.putBoolean("elevatorRaiseDone", elevatorRaiseDone);

    testytest.set(false);
    
    if(loopTimer.get() > 0.02)
      SRF_OverrunCount++;
  }


  void SRF_Basic(){
    //setIsolation(true, false);

    if(Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)) < 4)
      closeEnough = true;
    else
      closeEnough = false;

    if(tuneMode) {
      if(joyTune.getRawAxis(2) > 0.2) {
        //they move one way
      }
      else if(joyTune.getRawAxis(3) > 0.2) {
        //they move the other way
      }
      else {
        if(Math.abs(joyTune.getRawAxis(1)) > 0.1)
          climberArm.set(joyTune.getRawAxis(1));
        else
          climberArm.set(0);
        
        if(Math.abs(joyTune.getRawAxis(5)) > 0.1)
          climberWinch.set(joyTune.getRawAxis(5));
        else
          climberWinch.set(0);
      }
    }

    //automated vacuum pump
    if(!recharging && vacuumSensor.getValue() > vacuumHatchThreshold && !disableVacPump)
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
    
    if(Math.abs(j.getRawAxis(0)) > 0.1 || Math.abs(j.getRawAxis(1)) > 0.1) {
      //if(j.getRawButton(2)) //remember to change when B gets messed with, also may just need it straight up taken out for practice bot
      //  robot.arcadeDrive(.8*j.getRawAxis(1),0.5*j.getRawAxis(0));
      //else
        robot.arcadeDrive(j.getRawAxis(1),0.8*j.getRawAxis(0));
    } else
      robot.arcadeDrive(0, 0);

    //left trigger - roller in, right trigger - roller out
    if(j.getRawAxis(rollerIn) > 0.3) {
      roller.set(0.55);
    } else if(j.getRawAxis(rollerOut) > 0.1) {
      roller.set(-j.getRawAxis(rollerOut));
    } else {
      roller.set(.1);
    }
    
    if(Math.abs(j.getRawAxis(5)) > 0.2 && wristEnable) {
      if(wrist.getSelectedSensorPosition() < 142000 || j.getRawAxis(5) > 0)
        wrist.set(ControlMode.PercentOutput, -0.3*j.getRawAxis(5));
      else
        wrist.set(ControlMode.PercentOutput, 0);

      if(!progSmallElevatorRaise && wrist.getSelectedSensorPosition() > 100000 && !elevatorRaiseDone)
        progSmallElevatorRaise = true;
    } else
      wrist.set(ControlMode.PercentOutput, 0);
    
    if(progSmallElevatorRaise) {
      if(elevator.getSelectedSensorPosition() > -120000/*target - 120000 */)
        elevator.set(ControlMode.PercentOutput, .6);
      else {
        elevator.set(ControlMode.PercentOutput, 0);
        progSmallElevatorRaise = false;
        elevatorRaiseDone = true;
      }
    } else if(elevatorRaiseDone && wrist.getSelectedSensorPosition() < 74000){
      elevatorRaiseDone = false;
    }  

    //XXX-try taking out bleedIsSet!!!
    //A - bleed valve
    if(j.getRawButton(1) && !bleedIsSet){
      setBleed(true);
      bleedIsSet = true;
    }
    else if(!j.getRawButton(1) && bleedIsSet){
      setBleed(false);
      bleedIsSet = false;
    }

    //B - Open Manipulator isolation
    if(j.getRawButton(2)) {
      setIsolation(true, false);
    } else {
      setIsolation(false,false);
    }

    //B - Vaccum pump
    /*if(j.getRawButton(2)) //This is pointless as long as we ain't got no relays
      isolationRelay.set(Value.kForward);
    else
      isolationRelay.set(Value.kOff);*/
    /*if(j.getRawButton((2)))
      vacuumPump.set(1);
    else
      vacuumPump.set(0);*/

    //leftBumper - Down, rightBumper - Up
    if(j.getRawButton(5) /*&& elevator.getSelectedSensorPosition() < 0*/) {
      elevator.set(ControlMode.PercentOutput, -.3);//elevator down
    } else if(j.getRawButton(6)){
      elevator.set(ControlMode.PercentOutput, .5);//elevator up
    } else if(!progSmallElevatorRaise) {
      elevator.set(ControlMode.PercentOutput, 0);
    }

    //start, back - Rail
      if(railEnable && j.getRawButton(7) && rail.getSelectedSensorPosition() < railOut)//railOut
        rail.set(ControlMode.PercentOutput, 0.55);
      else if(railEnable && j.getRawButton(8) /*&& rail.getSelectedSensorPosition() > 0*/)//railIn
        rail.set(ControlMode.PercentOutput, -0.6);
      else
        rail.set(ControlMode.PercentOutput, 0);

      if(vacuumSensor.getValue() < vacuumHatchThreshold)
        vacuumAchieved = true;
      else
        vacuumAchieved = false;

      
  }


  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////


  void SRF_Test(){//we can't move in this mode
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

    /*if(j.getRawButton(5))
      climberArm.set(1);
    else
      climberArm.set(0);
*/
    if(j.getRawButton(6) && tuneMode)
      setBleed(true);
    else
      setBleed(false);

    if(!tuneMode)
    {
      if(j.getRawButton(2))
        setIsolation(true, false);
      else if(j.getRawButton(3))
        setIsolation(false, true);
      else
        setIsolation(false, false);
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
      letUpElevatorCycle = false;
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

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    if(j.getRawButton(1))
      bleedOutput.set(true);
    else
      bleedOutput.set(false);
    
    if(j.getRawButton(2))
      isolateManipulator.set(true);
    else
      isolateManipulator.set(false);

    if(j.getRawButton(3))
      isolateClimber.set(true);
    else
      isolateClimber.set(false);
  }

}
