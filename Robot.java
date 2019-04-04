
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SRF_PID;


public class Robot extends TimedRobot {//v1.6.9
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

  double elevatorP = 0.00001, elevatorI = 0.0000001, elevatorD = 0.0000001, wristP = .0095, wristI = 0.0000101, wristD = 0.01;
  SRF_PID[] pids = new SRF_PID[] {new SRF_PID(joyTune,elevatorP,elevatorI,elevatorD), new SRF_PID(joyTune,wristP,wristI,wristD)};
  int elev = 0;

  int pidCount = 0;
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
  DigitalOutput isolateManipulator = new DigitalOutput(9);
  DigitalOutput isolateClimber = new DigitalOutput(8);

  //place holders
  int elevatorNum = 1, rollerNum = 7, wristNum = 2, railNum = 0, vacuumPumpNum = 4, vacuumSensorNum = 0, isolationRelayNum = 0, bleedRelayNum = 1, climberArmNum = 6, climberWinchNum = 5, elevatorHighNum = 0, elevatorLowNum = 1, vacuumThreshold = 1000/*(?)*/, vacuumHatchThreshold = 2000, vacuumReleaseThreshold = 2500, railIn = 0, railOut = 700000/*222000*/;
  int elevatorHighPosition = -540000/*hatch middle*/, elevatorMiddlePosition/*cargo cargoship */, elevatorSemiLowPosition/*prevent scraping off suction cups for floor pickup*/, elevatorLowPosition = 0/*hatch floor/playerstation pickup, hatch place cargoship/low rocket*/, wristUltraLowPosition/* cargo pickup floor*/, wristLowPosition = 116000/*hatch pickup floor*/, wristMiddlePosition = 56000/* cargo low rocket place*/, wristHighPosition = 0 /*hatch cargoship/low rocket, cargo low*/;
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
  private static final boolean homeMode = true;;//resets encoders in teleop Periodic, may need to be true when testing outside of match play
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

  boolean elevatorControlMode = false;
  int elevatorControlModeTarget;
  final int elevatorCargoShip = -490000, elevatorMidHatch = 0;//Set to actual values
  int elevatorThreshold = 10000;
  
  SendableChooser singleInit;

  double elevatorSpeed = 0;

  boolean wristControl;
  double wristSpeed = 0;
  int wristTarget;
  boolean elevatorControlModeDone = true;

  boolean codeCripple = false;
  boolean letUpCripple = false;

  @Override
  public void robotInit() {
    elevatorHigh = new DigitalInput(elevatorHighNum);
    elevatorLow = new DigitalInput(elevatorLowNum);

    j = new Joystick(0);
    //if(tuneMode)//no issue with climber as long as we leave it on and stay in basic
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
    pids[elev].setTalon(elevator);
    pids[elev].setPID(elevatorP, elevatorI, elevatorD);
    elevator.setSelectedSensorPosition(0);
    //elevator.setNeutralMode(NeutralMode.Brake);

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
  
    singleInit = new SendableChooser();
    singleInit.addDefault("Competition",0);
    singleInit.addObject("Test",1);
    SmartDashboard.putData(singleInit);
  }

  public void SRF_Init() {

  }

  @Override
  public void autonomousInit() {
    wristControl = false;
    wristSpeed = 0;
    SRF_Init();
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

  public void disabledPeriodic() {
    SmartDashboard.putNumber("Rail Encoder", rail.getSelectedSensorPosition());
    SmartDashboard.putNumber("Wrist Encoder", wrist.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Ennoder", elevator.getSelectedSensorPosition());
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
    wristControl = false;
    wristSpeed = 0;

    if((int)singleInit.getSelected() == 1)
      SRF_Init();
    loopTimer.reset();
    loopTimer.start();
    SRF_OverrunCount = 0;

    closeEnough = false;
    bleedIsSet = false;
    recharging = false;
    letUpY = true;
    inTransition = false;
    elevatorControlMode = false;

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

    //testytest.set(true);
    if(testMode)//SRF_PID get the kP out of that for talon tuning
      SRF_Test();
    else if(codeCripple)
      SRF_Cripple();
    else
      SRF_Basic();
    
    if(joyTune.getRawButton(2) && letUpCripple) {
      letUpCripple = false;
      
      if(codeCripple)
        codeCripple = false;
      else
        codeCripple = true;
    }
    else if(!letUpCripple)
      letUpCripple = true;


    SmartDashboard.putBoolean("Wrist Control", wristControl);
    //SmartDashboard.putBoolean("Computing?", progCompute);
    //SmartDashboard.putNumber("Elevator Cycle Number",elevatorCycleCount);
    //SmartDashboard.putNumber("Wrist Cycle Number",wristCycleCount);  
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
    SmartDashboard.putNumber("CurrentGain", pids[pidCount].currentGain);*/
    //if(debug)SmartDashboard.putBoolean("HatchPlaceC", inProgresses[progHatchPlaceC]);
    //SmartDashboard.putNumber("targetPosition wrist", targetPositionWrist);
    SmartDashboard.putNumber("Rail Encoder", rail.getSelectedSensorPosition());
    SmartDashboard.putNumber("Overrun count", SRF_OverrunCount);
    SmartDashboard.putBoolean("Correct Vacuum level", vacuumAchieved);
    //SmartDashboard.putBoolean("Close Enough", closeEnough);
    //SmartDashboard.putBoolean("progSmallElevatorRaise", progSmallElevatorRaise);
    //SmartDashboard.putBoolean("elevatorRaiseDone", elevatorRaiseDone);
    SmartDashboard.putBoolean("Control Mode", elevatorControlMode);
    SmartDashboard.putNumber("Wrist Target", wristTarget);

    //testytest.set(false);
    
    if(loopTimer.get() > lastLoopTime + 0.02)
      SRF_OverrunCount++;
  }


  void SRF_Cripple(){
    //Climber Code
    /*if(joyTune.getRawAxis(5) > .9)
      climberArm.set(.5);
    else if(joyTune.getRawAxis(5) < -.9)
      climberArm.set(-.5);*/
    if(Math.abs(joyTune.getRawAxis(5)) > 0.2)
      climberArm.set(joyTune.getRawAxis(5));
    else
      climberArm.set(0);

    //vacuum pump
    if(j.getRawButton(2))
      vacuumPump.set(1);
    else
      vacuumPump.set(0);

    /* Pointless without being on front instead of inside robot
    if(j.getRawButton(9))
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    else
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    */

    if(Math.abs(j.getRawAxis(0)) > 0.1 || Math.abs(j.getRawAxis(1)) > 0.1) {
      //if(j.getRawButton(2)) //remember to change when B gets messed with, also may just need it straight up taken out for practice bot
      //  robot.arcadeDrive(.8*j.getRawAxis(1),0.5*j.getRawAxis(0));
      //else
        robot.arcadeDrive(j.getRawAxis(1),j.getRawAxis(0));
    } else
      robot.arcadeDrive(0, 0);

    //left trigger - roller in, right trigger - roller out
    if(j.getRawAxis(rollerIn) > 0.3) {
      roller.set(-0.55);
    } else if(j.getRawAxis(rollerOut) > 0.1) {
      roller.set(j.getRawAxis(rollerOut));
    } else {
      roller.set(-.1);
    }

    //Right Stick Y axis - controls reversed
    if(Math.abs(j.getRawAxis(5)) > 0.2 && wristEnable) {
      if(j.getRawAxis(5) > 0) {
        wristSpeed = -0.5*j.getRawAxis(5);//comp -.3 - COMPY
        //wrist.set(ControlMode.PercentOutput, -0.3*j.getRawAxis(5));
      } else
        wristSpeed = 0;
    } else {
      wristSpeed = 0;
      //wrist.set(ControlMode.PercentOutput, 0);
    }
    
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
    if(j.getRawButton(2)) 
      setIsolation(true, false);
    else
      setIsolation(false,false);

    //leftBumper - Down, rightBumper - Up
    if(j.getRawButton(5)) {
      elevator.set(ControlMode.PercentOutput, .3);//elevator down
    } else if(j.getRawButton(6)){
      elevator.set(ControlMode.PercentOutput, -.5);//elevator up
    } else if(!progSmallElevatorRaise) {
      elevator.set(ControlMode.PercentOutput, 0);
    }

    //start, back - Rail
    if(railEnable && j.getRawButton(7))//railIn
      rail.set(ControlMode.PercentOutput, 0.8);
    else if(railEnable && j.getRawButton(8))//railOut
      rail.set(ControlMode.PercentOutput, -0.3); 
    else
      rail.set(ControlMode.PercentOutput, 0);

    //Auto vacuum checking
    if(vacuumSensor.getValue() < vacuumHatchThreshold)
      vacuumAchieved = true;
    else
      vacuumAchieved = false;

    //Wrist
    wrist.set(ControlMode.PercentOutput, wristSpeed);
  }

  ///////////////////////////////////////////////
  ///////////////////////////////////////////////

  void SRF_Basic(){
    /* Won't work without it being on the front
    if(Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)) < 4)
      closeEnough = true;
    else
      closeEnough = false;    
    */

    //Climber Code
    /*if(joyTune.getRawAxis(5) > .9)
      climberArm.set(.5);
    else if(joyTune.getRawAxis(5) < -.9)
      climberArm.set(-.5);*/
    if(Math.abs(joyTune.getRawAxis(5)) > 0.2)
      climberArm.set(joyTune.getRawAxis(5));
    else
      climberArm.set(0);

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
    

    /* Pointless without being on front instead of inside robot
    if(j.getRawButton(9))
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    else
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    */

    if(Math.abs(j.getRawAxis(0)) > 0.1 || Math.abs(j.getRawAxis(1)) > 0.1) {
      //if(j.getRawButton(2)) //remember to change when B gets messed with, also may just need it straight up taken out for practice bot
      //  robot.arcadeDrive(.8*j.getRawAxis(1),0.5*j.getRawAxis(0));
      //else
        robot.arcadeDrive(j.getRawAxis(1),j.getRawAxis(0));
    } else
      robot.arcadeDrive(0, 0);

    //left trigger - roller in, right trigger - roller out
    if(j.getRawAxis(rollerIn) > 0.3) {
      roller.set(-0.55);
    } else if(j.getRawAxis(rollerOut) > 0.1) {
      roller.set(j.getRawAxis(rollerOut));
    } else {
      roller.set(-.1);
    }

    //Right Stick Y axis - controls reversed
    if(Math.abs(j.getRawAxis(5)) > 0.2 && wristEnable) {
      wristControl = false;

      if(wrist.getSelectedSensorPosition() < 142000 || j.getRawAxis(5) > 0) {
        wristSpeed = -0.5*j.getRawAxis(5);//comp -.3 - COMPY
        //wrist.set(ControlMode.PercentOutput, -0.3*j.getRawAxis(5));
      } else if(wrist.getSelectedSensorPosition() > 148000)
        wristSpeed = -0.3;
      else
        wristSpeed = 0;
    }
    else if(wrist.getSelectedSensorPosition() > 148000) {
      wristSpeed = -0.3;
      //wrist.set(ControlMode.PercentOutput, -.2);
    }
    else {
      wristSpeed = 0;
      //wrist.set(ControlMode.PercentOutput, 0);
    }

    //Raises elevator a few inches when wrist goes down a bit
    if(!progSmallElevatorRaise && wrist.getSelectedSensorPosition() > 100000 && !elevatorRaiseDone) {
        progSmallElevatorRaise = true;
    }
    
    if(progSmallElevatorRaise) {
      if(elevator.getSelectedSensorPosition() > -120000/*target - 120000 */)
        elevator.set(ControlMode.PercentOutput, -.6);
      else {
        elevator.set(ControlMode.PercentOutput, 0);
        progSmallElevatorRaise = false;
        elevatorRaiseDone = true;
        elevatorControlModeTarget = -120000;
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
    if(j.getRawButton(2)) 
      setIsolation(true, false);
    else
      setIsolation(false,false);

    //X
    if(j.getRawButton(3)) {
      elevatorControlMode = true;
      elevatorControlModeDone = false;
      elevatorControlModeTarget = elevatorLowPosition;
      //wristControl = true;
      //wristTarget = 140000;
    }
    //Y
    else if(j.getRawButton(4)) {
      elevatorControlMode = true;
      elevatorControlModeTarget = elevatorHighPosition;
    }

    //leftBumper - Down, rightBumper - Up
    if(j.getRawButton(5) /*&& elevator.getSelectedSensorPosition() < 0*/) {
      elevatorSpeed = 0.3;
      elevatorControlMode = false;
      // elevator.set(ControlMode.PercentOutput, .3);//elevator down
    } else if(j.getRawButton(6) && elevator.getSelectedSensorPosition() > -590000){
      elevatorSpeed = -0.5;
      elevatorControlMode = false;
      //elevator.set(ControlMode.PercentOutput, -.5);//elevator up
    } else if(!progSmallElevatorRaise) {
      elevatorSpeed = 0;
      //elevator.set(ControlMode.PercentOutput, 0);
    }

    //start, back - Rail
    if(railEnable && j.getRawButton(7) /*&& rail.getSelectedSensorPosition() > 0*/)//railIn
      rail.set(ControlMode.PercentOutput, 0.8);
    else if(railEnable && j.getRawButton(8) && rail.getSelectedSensorPosition() < railOut)//railOut
      rail.set(ControlMode.PercentOutput, -0.3); 
    else
      rail.set(ControlMode.PercentOutput, 0);

    //Auto vacuum checking
    if(vacuumSensor.getValue() < vacuumHatchThreshold)
      vacuumAchieved = true;
    else
      vacuumAchieved = false;

    //Auto Elevator
    if(!progSmallElevatorRaise)
    {
      if(elevatorControlMode) {
        //Elevator PID(?) Code
        //elevator.set(ControlMode.Position, elevatorControlModeTarget);
        elevatorControl(elevatorControlModeTarget);
      }
      else
        elevator.set(ControlMode.PercentOutput, elevatorSpeed);
    }

    //Auto Wrist
    if(wristControl && elevatorControlModeDone == true) {
      if(Math.abs(wrist.getSelectedSensorPosition()-wristTarget) < 8000)
        wrist.set(ControlMode.PercentOutput, 0);
      else if(wrist.getSelectedSensorPosition() < wristTarget)
        wrist.set(ControlMode.PercentOutput, 0.4);
      else
        wrist.set(ControlMode.PercentOutput, -0.4);
    }
    else
      wrist.set(ControlMode.PercentOutput, wristSpeed);

    /*if(tuneMode){
      pids[elev].controlPID();
      elevator.config_kP(0, pids[0].k[0]);
      elevator.config_kI(0, pids[0].k[1]);
      elevator.config_kD(0, pids[0].k[2]);
    }*/
  }

  void elevatorControl(int target) {
    if(Math.abs(elevator.getSelectedSensorPosition() - target) < elevatorThreshold) {
      elevatorControlModeDone = true;
      elevator.set(ControlMode.PercentOutput, -0.05);
    } else if(elevator.getSelectedSensorPosition() > target)
      elevator.set(ControlMode.PercentOutput, -1);
    else
      elevator.set(ControlMode.PercentOutput, 0.7);
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////


  void SRF_Test(){//we can't move in this mode

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
    elevator.setSelectedSensorPosition(0);
    loopTimer.reset();
    //loopTimer.start();
    SRF_OverrunCount = 0;
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  @Override
  public void testPeriodic() {
    if(j.getRawButton(1))
      elevator.set(ControlMode.PercentOutput, 0.3);
    else if(j.getRawButton(2))
      elevator.set(ControlMode.PercentOutput, -0.3);
    else
      elevator.set(ControlMode.PercentOutput, 0);

    SmartDashboard.putNumber("elevator position", elevator.getSelectedSensorPosition());
  }
}

