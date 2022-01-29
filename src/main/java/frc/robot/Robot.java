/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commands.AutonomousCommandGroups.AutonomousShooterToTrenchSequence;
import frc.robot.models.AutonomousSequences;
import frc.robot.models.AutonomousTrajectories;
import frc.robot.models.PathSelecter;
import frc.robot.subsystems.*;


//changes
//changes

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

 //To deploy this, run (gradlew deploy -x test) in the command prompt Hi- Mark
public class Robot extends TimedRobot {
  public static OI oi;
  private static final double UPDATE_DT = 5e-3; // 5 ms

  Command autonomousCommand;

  public static DrivetrainSubsystem drivetrainSubsystem;

  public static Vision vision;
  public static ObjectTrackerSubsystem objectTrackerSubsystem;

  public static ElevatorSubsystem elevatorSubsystem;
  public static IntakeSubsystem intakeSubsystem;
  public static ShooterSubsystem shooterSubsystem;


  SendableChooser<CommandGroup> m_chooser;
  

  private SubsystemManager subsystemManager;

  public HolonomicDriveCommand driveCommand;
  ZeroFieldOrientedCommand zeroCommand;
  ZeroFieldOrientedCommand reverseZeroCommand;

  ElevatorCommand elevatorUpCommand;
  ElevatorCommand elevatorDownCommand;

  ElevatorIndexCommand elevatorIndexUpCommand;
  ElevatorIndexCommand elevatorIndexDownCommand;

  ShooterCommand shooterWithVisionCommand;
  ShooterCommand shooterNoVisionCommand;
  
  IndexZoneCommand indexZoneCommand;

  IntakeCommand  intakeInCommand;
  IntakeCommand intakeOutCommand;

  IntakeDetectCommand intakeDetectCommand;

  IntakeDetectToElevatorIndexCommand intakeDetectToElevatorIndexCommand;


  IntakeActuateCommand lowerIntakeCommand;
  IntakeActuateCommand raiseIntakeCommand;
  IntakeToggleCommand intakeToggleCommand;

  ShooterActuateCommand raiseShooterCommand;
  ShooterActuateCommand lowerShooterCommand;
  ShooterToggleCommand shooterToggleCommand;


  VisionLightCommand visionLightCommand;
  VisionRotationDriveCommand visionRotationDriveCommand;
  RobotRotateCommand robotRotateCommand;

  SnapshotCommand snapshotCommand;



  boolean autoHappened;

  // IntakeCommandGroup intakeCommandGroup;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

    public static int circularBufferSize = 50;
    public static int bufferSlotNumber = 0;
    public static double[] time;
    public static double[] angle;

  @Override
public void robotInit() { 
    time = new double[circularBufferSize]; 
    angle =  new double[circularBufferSize];

    autoHappened = false;
    SmartDashboard.putNumber("motor1Speed", RobotMap.SHOOTER_MOTOR_HIGH_DEFAULT_SPEED);
    SmartDashboard.putNumber("motor2Speed", RobotMap.SHOOTER_MOTOR_HIGH_DEFAULT_SPEED * .75);
    SmartDashboard.putNumber("Object detection latency", RobotMap.OBJECT_DETECTION_LATENCY);
    SmartDashboard.putNumber("angleP", 0.25);
    SmartDashboard.putNumber("angleI", 0.0);
    SmartDashboard.putNumber("angleD", 0.0);

    SmartDashboard.putNumber("strafeP", 0.005);
    SmartDashboard.putNumber("strafeI", 0.0);
    SmartDashboard.putNumber("strafeD", 0.0);
    SmartDashboard.putNumber("Forward Speed", 0.5);

    
    oi = new OI();
    //m_chooser.setDefaultOption("Default Auto", new AutonomousCommand());
    initSubsystems();
    initCommands();
    initButtons();
    initChooser();
    vision.ledOff();




  }

private void initSubsystems() {
  vision = new Vision();
  drivetrainSubsystem = new DrivetrainSubsystem();
  elevatorSubsystem = new ElevatorSubsystem();
  shooterSubsystem = new ShooterSubsystem();
  subsystemManager = new SubsystemManager(drivetrainSubsystem);
  intakeSubsystem = new IntakeSubsystem();
  objectTrackerSubsystem = new ObjectTrackerSubsystem();
}

private void initCommands() {
    // recordCommand = new ToggleDriveRecordCommand();
    zeroCommand = new ZeroFieldOrientedCommand(drivetrainSubsystem);
    reverseZeroCommand = new ZeroFieldOrientedCommand(drivetrainSubsystem, 180);
    driveCommand = new HolonomicDriveCommand(DrivetrainSubsystem.ControlMode.DualStick);
    visionLightCommand = new VisionLightCommand();
    visionRotationDriveCommand = new VisionRotationDriveCommand();
    robotRotateCommand = new RobotRotateCommand(90);
    shooterWithVisionCommand = new ShooterCommand(true);
    shooterNoVisionCommand = new ShooterCommand(false);
    elevatorUpCommand = new ElevatorCommand(false);
    elevatorDownCommand = new ElevatorCommand(true);

    elevatorIndexUpCommand = new ElevatorIndexCommand(true, 70);
    elevatorIndexDownCommand = new ElevatorIndexCommand(false, 70);


    intakeInCommand = new IntakeCommand(false);
    intakeOutCommand = new IntakeCommand(true);

    intakeDetectCommand = new IntakeDetectCommand();

    intakeDetectToElevatorIndexCommand = new IntakeDetectToElevatorIndexCommand();

    lowerIntakeCommand = new IntakeActuateCommand(false, 3);
    raiseIntakeCommand = new IntakeActuateCommand(true, 3);
    intakeToggleCommand = new IntakeToggleCommand();

    raiseShooterCommand = new ShooterActuateCommand(false, 3);
    lowerShooterCommand = new ShooterActuateCommand(true, 3);
    shooterToggleCommand = new ShooterToggleCommand();

    snapshotCommand = new SnapshotCommand();
    indexZoneCommand = new IndexZoneCommand();
 }

private void initButtons() {
    //oi.bedForwardButton.toggleWhenPressed(bedForwardCommand);
    // oi.toggleDriveRecordButton.toggleWhenPressed(recordCommand);
    oi.visionDriveButton.whileHeld(visionRotationDriveCommand);
    
    //oi.intakeButton.whileHeld(intakeCommandGroup);
     oi.elevatorUpButton.whileHeld(elevatorUpCommand);
    oi.elevatorDownButton.whileHeld(elevatorDownCommand);

    oi.elevatorIndexUpButton.whenPressed(elevatorIndexUpCommand);
    oi.elevatorIndexDownButton.whenPressed(elevatorIndexDownCommand);

     oi.intakeInButton.whileHeld(intakeInCommand);
     oi.intakeOutButton.whileHeld(intakeOutCommand);
     oi.intakeDetectButton.whileHeld(intakeDetectToElevatorIndexCommand);

     oi.intakeElevationButton.toggleWhenPressed(intakeToggleCommand);
     
     oi.shooterElevationButton.toggleWhenPressed(shooterToggleCommand);

    //oi.helloArcButton.whileHeld(robotRotateCommand);
    oi.referenceResetButton.whenPressed(zeroCommand);
    oi.shooterNoVisionButton.whileHeld(shooterNoVisionCommand);
    oi.shooterVisionButton.whileHeld(shooterWithVisionCommand);
    oi.indexZoneButton.whenPressed(indexZoneCommand);
    //oi.snapShotButton.whenPressed(snapshotCommand);

}

private void initChooser() {
  
  m_chooser = new SendableChooser<>();

//  m_chooser.addOption("Shoot, Collect Right",s AutonomousSequences.ShootThenCollectRight());
//  m_chooser.addOption("Shoot, Collect Right, Shoot Again ", AutonomousSequences.ShootThenCollectRight_ThenShootAgain());
//  m_chooser.addOption("Leave Initiation Line", AutonomousSequences.backAwayFromInitiationLine());
//  m_chooser.addOption("Shoot from Right, Collect Right, Shoot Again", AutonomousSequences.ShootFromRight_Of_Optimal_Then_Collect());
//  m_chooser.addOption("Shoot Then Leave Initiation Line", AutonomousSequences.shootThenBackAwayFromInitiationLine());
//  m_chooser.addOption("Shoot, Collect Left", AutonomousSequences.ShootThenCollectLeft());
 m_chooser.addOption("Rotate 90 degrees", AutonomousSequences.RotateTest());
 m_chooser.addOption("Galactic-Search Red A", AutonomousSequences.GalacticSearchRedPathA());
 m_chooser.addOption("Galactic-Search Red B", AutonomousSequences.GalacticSearchRedPathB());
//  m_chooser.addOption("DriveTwoFeetTwice", AutonomousSequences.DriveTwoFeetTwice());
m_chooser.addOption("DriveStraightAndBack", AutonomousSequences.DriveStraightForwardAndBack());
 m_chooser.addOption("DriveLeftAndBack", AutonomousSequences.DriveLeftThenRight());
//  m_chooser.addOption("IntakeTest", AutonomousSequences.IntakeTest());
 m_chooser.addOption("GoFetchTest", AutonomousSequences.GoFetchTest());
 m_chooser.addOption("Galactic-Search Blue A", AutonomousSequences.GalacticSearchBluePathA());
 m_chooser.addOption("Galactic-Search Blue B", AutonomousSequences.GalacticSearchBluePathB());
 m_chooser.addOption("Galactic-Search Red A Rotate", AutonomousSequences.GalacticSearchRedPathARotate());
 //m_chooser.addOption("GalacticSearch command", AutonomousSequences.GalacticSearch());
 //m_chooser.addOption("Barrel path", AutonomousSequences.barrelRacing());
 m_chooser.addOption("Arc test", AutonomousSequences.PathArcTest());
 m_chooser.addOption("Barrel Racing 1", AutonomousSequences.barrelRacing());
 m_chooser.addOption("Barrel Racing 2", AutonomousSequences.barrelRacing2());
 m_chooser.addOption("Bounce Path", AutonomousSequences.bounce());
 m_chooser.addOption("Bounce2 path", AutonomousSequences.bounce2());
//  m_chooser.addOption("Slalom Path", AutonomousSequences.slalom());
 m_chooser.addOption("Slalom2 Path", AutonomousSequences.slalom2());

 m_chooser.addOption("Slalom3 Path", AutonomousSequences.slalom3());
//  m_chooser.addOption("Fetch Power Cell Command", AutonomousSequences.)
m_chooser.addOption("Fetch Power Cell PID Tuning", AutonomousSequences.FetchPowerCellPidTest()); 
  SmartDashboard.putData("Auto mode", m_chooser);
}

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    time[bufferSlotNumber] = Timer.getFPGATimestamp(); 
    angle[bufferSlotNumber] = drivetrainSubsystem.getGyroscope().getAngle().toRadians();
    bufferSlotNumber = (bufferSlotNumber++) % circularBufferSize;

    drivetrainSubsystem.outputToSmartDashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.;
   */
  @Override
  public void disabledInit() {
    Robot.drivetrainSubsystem.getFollower().cancel();

    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0.0, true);

    subsystemManager.disableKinematicLoop();
    vision.ledOff();
    autoHappened = false;

    
  }

  

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    if(vision.getLed() != 1)
    vision.ledOff();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    autoHappened = true;
    subsystemManager.enableKinematicLoop(UPDATE_DT);
    zeroCommand.start();
    autonomousCommand = m_chooser.getSelected();

    // String chosenPath = PathSelecter.choosePath();
    // SmartDashboard.putString("Path", chosenPath);
    
    // switch (chosenPath) {
    //   case "PathRedA":
    //     //autonomousCommand = AutonomousSequences.GalacticSearchRedPathA();
    //     break;
    //   case "PathBlueA":
    //     //autonomousCommand = AutonomousSequences.GalacticSearchBluePathA();
    //     break;
    //   case "PathRedB":
    //     //autonomousCommand = AutonomousSequences.GalacticSearchRedPathB();
    //     break;
    //   case "PathBlueB":
    //     //autonomousCommand = AutonomousSequences.GalacticSearchBluePathB();
    //     break;
    // }
    //autonomousCommand = AutonomousSequences.GalacticSearchBluePathA();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */
  
    
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //SmartDashboard.putNumber("Navx angle:", Robot.drivetrainSubsystem.getGyroscope().getAngle().toRadians());
    //SmartDashboard.putNumber("Unadjusted angle:", Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle().toRadians());
    //SmartDashboard.putString("Hey dummy", "don't forget to uncomment zero command");
    Scheduler.getInstance().run();
    //System.out.println(drivetrainSubsystem.getGyroscope().getRate());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // if(autoHappened){
       reverseZeroCommand.start(); // *** uncomment for reversing bot so shooter is "front"
    // }
    Robot.drivetrainSubsystem.getFollower().cancel();

    SmartDashboard.putNumber("ShooterMotor1", RobotMap.SHOOTER_MOTOR_HIGH_DEFAULT_SPEED);
    

    subsystemManager.enableKinematicLoop(UPDATE_DT);
    // zeroCommand.start(); // *** comment out for reversing so shooter is "front"
   // SmartDashboard.putString("Path", PathSelecter.choosePath());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    Scheduler.getInstance().run();
    
    //Vector2 vec = drivetrainSubsystem.getKinematicPosition();
    //SmartDashboard.putNumber("Current Pose X", vec.x);
    //SmartDashboard.putNumber("Current Pose Y", vec.y);
    
   

    //drivetrainSubsystem.outputToSmartDashboard();
  }
@Override
public void testInit(){
  //subsystemManager.enableKinematicLoop(UPDATE_DT);

}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    //drivetrainSubsystem.updateKinematics(UPDATE_DT);
    drivetrainSubsystem.outputToSmartDashboard();
  }
}
