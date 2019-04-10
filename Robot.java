
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
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SRF_PID;


public class Robot extends TimedRobot {//v1.6.9a
  /*
    added some new functionality (elevator raising when wrist goes down and some other stuff)
    also added trouble shooting mini over run timer and did a partial code review  
 */

  //PROB - probably good to removed

  //Initialization
  Talon frontLeft;//Drivebase motorcontrollers
  Talon rearLeft;
  Talon frontRight;
  Talon rearRight;

  SpeedControllerGroup left;//controller groups that are defined based on the above controllers
  SpeedControllerGroup right;

  DifferentialDrive robot;//our collective robot drive base

  VictorSP climberArm;//The ramp arm
  Victor climberWinch;//PROB

  TalonSRX wrist;//manipulator controllers
  VictorSP roller;
  TalonSRX elevator;
  TalonSRX rail;

  Spark vacuumPump;//the controller for getting the vacuum
  //These were taken out when we added the DIO support
  //Relay isolationRelay;
  //Relay bleedRelay;

  Joystick j;//primary
  Joystick joyTune;//secondary (sometimes used for PID tuning)

  //Sensors (we don't have the flags on currently)
  DigitalInput elevatorHigh;
  DigitalInput elevatorLow;
  AnalogInput vacuumSensor;

  //Drivebase encoders are currently unimplemented
  Encoder leftSide;
  Encoder rightSide;

  //buttons
  int rollerIn = 2, rollerOut = 3; //leftTrigger and rightTrigger

  boolean letUpB = true, letUpClimb = true, letUpCargoPickupF = true, letUpCargoPickupP = true, letUpCargoPlaceC = true, letUpCargoPlaceRL = true, letUpCargoPlaceRH = true;//used for confirmation press
 
  boolean progSmallElevatorRaise = false;

  //PID
  int elevatorTuner = 0, wristTuner = 1;//PROB-check logic elsewhere
  int elevatorCycleCount = 0, wristCycleCount = 0;
  boolean letUpChangePID = true, letUpCompute = true, progCompute = false, letUpWristCycle = true, letUpElevatorCycle = true;

  double elevatorP = 0.00001, elevatorI = 0.0000001, elevatorD = 0.0000001, wristP = .0095, wristI = 0.0000101, wristD = 0.01;
  SRF_PID[] pids = new SRF_PID[] {new SRF_PID(joyTune,elevatorP,elevatorI,elevatorD), new SRF_PID(joyTune,wristP,wristI,wristD)};
  int elev = 0;

  int pidCount = 0;
  double targetPositionElevator = 0;
  double targetPositionWrist = 0;
  double targetPositionWristTemp;

  Timer wristPosition = new Timer();

  //misc
  double elevatorInput, wristInput;
  NetworkTable table;
  DigitalOutput testytest = new DigitalOutput(5);
  DigitalOutput bleedTest = new DigitalOutput(6);
  boolean boolTest = false;

  DigitalOutput bleedVacuum = new DigitalOutput(8);
  DigitalOutput isolateManipulator = new DigitalOutput(9);

  //place holders
  int elevatorNum = 1, rollerNum = 7, wristNum = 2, railNum = 0, vacuumPumpNum = 4, vacuumSensorNum = 0, isolationRelayNum = 0, bleedRelayNum = 1, climberArmNum = 6, climberWinchNum = 5/*PROB*/, elevatorHighNum = 0, elevatorLowNum = 1, vacuumThreshold = 1000/*(?)*/, vacuumHatchThreshold = 2000, vacuumReleaseThreshold = 2500, railIn = 0, railOut = 222000/*700000*/;
  int elevatorHighPosition = -585000/*hatch middle*/, elevatorMiddlePosition = -470000/*cargo cargoship */, elevatorLowPosition = 10000/*resting position*/, elevatorSmallRaisePosition/*prevent scraping off suction cups for floor pickup*/, wristHighPosition = 43000 /*cargo Middle*/;
  double wristTolerance = 9001, elevatorTolerance, railTolerance;

  Timer rumbleTimer = new Timer();
  Timer timeoutTimer = new Timer();
  Timer loopTimer = new Timer();
  double lastLoopTime;
  long SRF_OverrunCount; //this should serve as a rough metric to gauge what's causing loop time to spike
  double startTime;

  
  //enable flags
  private static final boolean elevatorEnable = false;//a boolean that enables and disables the elevator motion (to handle pre-elevator movement)
  private static final boolean wristEnable = true;
  private static final boolean railEnable = true;
  
  private static final boolean testMode = false;
  private static final boolean tuneMode = false;
  private static final boolean homeMode = false;//resets encoders in teleop Periodic, may need to be true when testing outside of match play
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

  boolean closeEnough;

  boolean elevatorControlMode = false;
  int elevatorControlModeTarget;
  int elevatorThreshold = 10000;
  
  SendableChooser singleInit;

  double elevatorSpeed = 0;

  boolean wristControl;//PROB
  double wristSpeed = 0;
  int wristTarget;//PROB
  boolean elevatorControlModeDone = true;

  boolean codeCripple = false;
  boolean letUpCripple = false;

  boolean letUpTest = true;
  boolean quickTestMode = true;

  SRF_Button testButtonA = new SRF_Button(j,1);
  SRF_Button testButtonB = new SRF_Button(j,2);
  SRF_Button testButtonX = new SRF_Button(j,3);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //          Misc Functions          //
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////



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
    //bleedTest.pulse(1000);
    bleedVacuum.set(v);
  }


  public void setIsolation(boolean man) {
    isolateManipulator.set(man);
  }


  void elevatorControl(int target) {
    if(Math.abs(elevator.getSelectedSensorPosition() - target) < elevatorThreshold) {
      elevatorControlModeDone = true;
      if(target == elevatorLowPosition)
        elevator.set(ControlMode.PercentOutput, 0);
      else
        elevator.set(ControlMode.PercentOutput, -0.05);
    } else if(elevator.getSelectedSensorPosition() > target)
      elevator.set(ControlMode.PercentOutput, -.8);
    else
      elevator.set(ControlMode.PercentOutput, 0.4);
  }



  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////



  @Override
  public void robotInit() {
    elevatorHigh = new DigitalInput(elevatorHighNum);
    elevatorLow = new DigitalInput(elevatorLowNum);

    j = new Joystick(0);
    //if(tuneMode)//no issue with climber as long as we leave it on and stay in basic
    joyTune = new Joystick(1);

    //driveBase - initializes drivebase motor controllers, the controller groups and the drivebase in succession
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

    climberWinch = new Victor(climberWinchNum);//PROB

    vacuumPump = new Spark(vacuumPumpNum);

    leftSide = new Encoder(2,3);
    vacuumSensor = new AnalogInput(vacuumSensorNum);

    targetPositionWristTemp = wristHighPosition;

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  
    singleInit = new SendableChooser();
    singleInit.addOption("Competion",0);
    singleInit.addOption("Test",1);
    
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
    if(quickTestMode)
      testPeriodic();
    else
      SRF_Basic();

    if(joyTune.getRawButton(1) && letUpTest) {
      letUpTest = false;
      quickTestMode = !quickTestMode;
    }
    else if(!joyTune.getRawButton(1))
      letUpTest = true;

    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());  
  }



  public void disabledPeriodic() {
    SmartDashboard.putNumber("Rail Encoder", rail.getSelectedSensorPosition());
    SmartDashboard.putNumber("Wrist Encoder", wrist.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Ennoder", elevator.getSelectedSensorPosition());
    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());
  }



  @Override
  public void teleopInit() {
    wristControl = false;
    wristSpeed = 0;

    //if((int)singleInit.getSelected() == 1)
      //SRF_Init();
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
     //j.setRumble(RumbleType.kLeftRumble, 0);
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
    else if(quickTestMode)
      testPeriodic();
    else
      SRF_Basic();
    
    if(joyTune.getRawButton(2) && letUpCripple) {
      letUpCripple = false;
      codeCripple = !codeCripple;
    }
    else if(!joyTune.getRawButton(2))
      letUpCripple = true;

    if(joyTune.getRawButton(1) && letUpTest) {
      letUpTest = false;
      quickTestMode = !quickTestMode;
    }
    else if(!joyTune.getRawButton(1))
      letUpTest = true;

    SmartDashboard.putBoolean("Wrist Control", wristControl);
    //SmartDashboard.putBoolean("Computing?", progCompute);
    //SmartDashboard.putNumber("Elevator Cycle Number",elevatorCycleCount);
    //SmartDashboard.putNumber("Wrist Cycle Number",wristCycleCount);  
    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());
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



  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////



void SRF_Basic(){
    /* Won't work without it being on the front
    if(Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)) < 4)
      closeEnough = true;
    else
      closeEnough = false;    
    */

    //Climber Code
    if(Math.abs(joyTune.getRawAxis(5)) > 0.2)
      climberArm.set(.75*joyTune.getRawAxis(5));
    else
      climberArm.set(0);

    if(!tuneMode && joyTune.getRawButton(3)) {
      wrist.setSelectedSensorPosition(0);
      elevator.setSelectedSensorPosition(0);
      rail.setSelectedSensorPosition(0);
      SRF_OverrunCount = 0;
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
      roller.set(-.2);
    }

    //Right Stick Y axis - controls reversed
    if(Math.abs(j.getRawAxis(5)) > 0.2 && wristEnable) {
      wristControl = false;

      if(wrist.getSelectedSensorPosition() < 142000 || j.getRawAxis(5) > 0) {
        wristSpeed = -0.5*j.getRawAxis(5);
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
        elevatorControlMode = true;
        elevatorControlModeTarget = elevator.getSelectedSensorPosition();
      }
    } else if(elevatorRaiseDone && wrist.getSelectedSensorPosition() < 74000){
      elevatorRaiseDone = false;
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
      setIsolation(true);
    else
      setIsolation(false);

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

    //Right Bumper - elevator Middle position
    /*if(j.getRawButton(6)) {
      //elevatorControlMode = true;
      elevatorControlModeTarget = elevatorMiddlePosition;
    }*/
    
        //Manual Elevator Code PROB
    if(j.getRawButton(5)) {//elevator down
      elevatorSpeed = 0.3;
      elevatorControlMode = false;
      // elevator.set(ControlMode.PercentOutput, .3);
    } else if(j.getRawButton(6) && elevator.getSelectedSensorPosition() > -590000){//elevator up
      elevatorSpeed = -0.5;
      elevatorControlMode = false;
      //elevator.set(ControlMode.PercentOutput, -.5);
    } else if(!progSmallElevatorRaise) {
      //if(elevatorSpeed != 0)
      //  elevatorControlModeTarget = elevator.getSelectedSensorPosition();
      elevatorSpeed = 0;
      //elevatorControlMode = true; 
      //elevator.set(ControlMode.PercentOutput, 0);
    }
    
    //start, back - Rail
    if(railEnable && j.getRawButton(7) /*&& rail.getSelectedSensorPosition() > 0*/)//railIn
      rail.set(ControlMode.PercentOutput, -1);
    else if(railEnable && j.getRawButton(8) && rail.getSelectedSensorPosition() < railOut)//railOut
      rail.set(ControlMode.PercentOutput, 0.3); 
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
        elevator.set(ControlMode.Position, elevatorControlModeTarget);
        elevatorControl(elevatorControlModeTarget);
      }
      else
        elevator.set(ControlMode.PercentOutput, elevatorSpeed);
    }

    //Auto Wrist
    if(wristControl && elevatorControlModeDone) {
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



  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////



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
      roller.set(-.2);
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
      setIsolation(true);
    else
      setIsolation(false);

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
      rail.set(ControlMode.PercentOutput, -1);
    else if(railEnable && j.getRawButton(8))//railOut
      rail.set(ControlMode.PercentOutput, 0.3); 
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

  

  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////



  void SRF_Test(){//we can't move in this mode
  if(testButtonA.whileHeld())
    roller.set(.5);
  else
    roller.set(0);

  if(testButtonB.whenPressed())
    SmartDashboard.putBoolean("button Test",true);
  else
    SmartDashboard.putBoolean("button Test",false);

  if(testButtonX.whenReleased())
    SmartDashboard.putBoolean("button Test2",true);
  else
    SmartDashboard.putBoolean("button Test2",false);

  /*
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
        setIsolation(true);
      else
        setIsolation(false);
    }
    */
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
          targetPositionElevator = elevatorLowPosition;
    } else if(!j.getRawButton(7)) {
      letUpElevatorCycle = true;
    }

    /*if(j.getRawButton(8) && letUpWristCycle) {
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
      letUpWristCycle = true;
    }*/ 
   
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


  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////



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
    //Climber Code
    if(Math.abs(joyTune.getRawAxis(5)) > 0.2)
      climberArm.set(.75*joyTune.getRawAxis(5));
    else
      climberArm.set(0);

    if(!tuneMode && joyTune.getRawButton(3)) {
      wrist.setSelectedSensorPosition(0);
      elevator.setSelectedSensorPosition(0);
      rail.setSelectedSensorPosition(0);
      SRF_OverrunCount = 0;
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
      roller.set(-.2);
    }

    //Right Stick Y axis - controls reversed
    if(Math.abs(j.getRawAxis(5)) > 0.2 && wristEnable) {
      wristControl = false;

      if(wrist.getSelectedSensorPosition() < 142000 || j.getRawAxis(5) > 0) {
        wristSpeed = -0.5*j.getRawAxis(5);
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
        //elevatorControlMode = true;
        elevatorControlModeTarget = elevator.getSelectedSensorPosition();
      }
    } else if(elevatorRaiseDone && wrist.getSelectedSensorPosition() < 74000){
      elevatorRaiseDone = false;
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
      setIsolation(true);
    else
      setIsolation(false);

    //X
    if(j.getRawButton(3)) {
      //elevatorControlMode = true;
      elevatorControlModeDone = false;
      elevatorControlModeTarget = elevatorLowPosition;
      //wristControl = true;
      //wristTarget = 140000;
    }
    //Y
    else if(j.getRawButton(4)) {
      //elevatorControlMode = true;
      elevatorControlModeTarget = elevatorHighPosition;
    }

    //Right Bumper - elevator Middle position
    if(j.getRawButton(6)) {
      //elevatorControlMode = true;
      elevatorControlModeTarget = elevatorMiddlePosition;
    }
    
    //Left Bumper - Wrist High Position
    if(j.getRawButton(5)) {
      wristTarget = wristHighPosition;
      wristControl = true;
    }

    /*    //Manual Elevator Code PROB
    if(j.getRawButton(5)) {//elevator down
      elevatorSpeed = 0.3;
      elevatorControlMode = false;
      // elevator.set(ControlMode.PercentOutput, .3);
    } else if(j.getRawButton(6) && elevator.getSelectedSensorPosition() > -590000){//elevator up
      elevatorSpeed = -0.5;
      elevatorControlMode = false;
      //elevator.set(ControlMode.PercentOutput, -.5);
    } else if(!progSmallElevatorRaise) {
      //if(elevatorSpeed != 0)
      //  elevatorControlModeTarget = elevator.getSelectedSensorPosition();
      elevatorSpeed = 0;
      //elevatorControlMode = true; 
      //elevator.set(ControlMode.PercentOutput, 0);
    }*/
    
    //start, back - Rail
    if(railEnable && j.getRawButton(7) /*&& rail.getSelectedSensorPosition() > 0*/)//railIn
      rail.set(ControlMode.PercentOutput, -1);
    else if(railEnable && j.getRawButton(8) && rail.getSelectedSensorPosition() < railOut)//railOut
      rail.set(ControlMode.PercentOutput, 0.3); 
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
      //if(elevatorControlMode) {
      //  elevator.set(ControlMode.Position, elevatorControlModeTarget);
        elevatorControl(elevatorControlModeTarget);
      //}
      //else
      //  elevator.set(ControlMode.PercentOutput, elevatorSpeed);
    }

    //Auto Wrist
    if(wristControl && elevatorControlModeDone) {
      if(Math.abs(wrist.getSelectedSensorPosition()-wristTarget) < 8000)
        wrist.set(ControlMode.PercentOutput, 0.05);
      else if(wrist.getSelectedSensorPosition() < wristTarget)
        wrist.set(ControlMode.PercentOutput, 0.4);
      else
        wrist.set(ControlMode.PercentOutput, -0.4);
    }
    else
      wrist.set(ControlMode.PercentOutput, wristSpeed);

    SmartDashboard.putNumber("elevator position", elevator.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rail Encoder", rail.getSelectedSensorPosition());
    SmartDashboard.putNumber("Wrist Encoder", wrist.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Ennoder", elevator.getSelectedSensorPosition());
    SmartDashboard.putNumber("vacuumSensor", vacuumSensor.getValue());
  }
}

