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

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.*;
import frc.robot.models.AutonomousSequences;
import frc.robot.subsystems.*;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;


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

  public static IntakeSubsystem intakeSubsystem;
  public static ShooterSubsystem shooterSubsystem;
  public static ClimberSubsystem climberSubsystem;
  public static ColorDetectorSubsystem colorDetectorSubsystem;



  SendableChooser<CommandGroup> m_chooser;
  

  private SubsystemManager subsystemManager;

  public HolonomicDriveCommand driveCommand;
  ZeroFieldOrientedCommand zeroCommand;
  ZeroFieldOrientedCommand reverseZeroCommand;


  ShooterCommand shooterWithVisionCommand;
  ShooterCommand shooterNoVisionCommand;
  ShooterIdleCommand shooterIdleCommand;
  ShooterIdleCommand shooterIdleCommand_ZERO_RPM;
  ShooterIdleCommand reverseShooterCommand; 

  
  // IndexZoneCommand indexZoneCommand;

  ClimberActuateCommand extendClimberCommand;
  ClimberActuateCommand retractClimberCommand;

  IntakeCommand  intakeInCommand;
  IntakeCommand intakeOutCommand;


  IntakeActuateCommand extendIntakeCommand;
  IntakeRetractCommand2 retractIntakeCommand;
  IntakeRetractCommand midStateIntakeCommand;


  VisionRotationDriveCommand visionRotationDriveCommand;
  RobotRotateCommand robotRotateCommand;


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
  // Robot.vision.ledOn();
    time = new double[circularBufferSize]; 
    angle =  new double[circularBufferSize];
    autoHappened = false;
    // SmartDashboard.putNumber("motor1Speed", RobotMap.SHOOTER_MOTOR_HIGH_DEFAULT_SPEED);
    // SmartDashboard.putNumber("motor2Speed", RobotMap.SHOOTER_MOTOR_HIGH_DEFAULT_SPEED * .75);
    // SmartDashboard.putNumber("Object detection latency", RobotMap.OBJECT_DETECTION_LATENCY);
    // SmartDashboard.putNumber("angleP", 0.25);
    // SmartDashboard.putNumber("angleI", 0.0);
    // SmartDashboard.putNumber("angleD", 0.0);

    // SmartDashboard.putNumber("strafeP", 0.005);
    // SmartDashboard.putNumber("strafeI", 0.0);
    // SmartDashboard.putNumber("strafeD", 0.0);
    // SmartDashboard.putNumber("Forward Speed", 0.5);
    // colorDetectorSubsystem.ColorMatcher();
    // climberSubsystem.retractClimber();


    oi = new OI();
    //m_chooser.setDefaultOption("Default Auto", new AutonomousCommand());
    initSubsystems();
    initCommands();
    initButtons();
    initChooser();
    vision.ledOn();




  }

private void initSubsystems() {
  vision = new Vision();
  drivetrainSubsystem = new DrivetrainSubsystem();
  climberSubsystem = new ClimberSubsystem();
  // drivetrainSubsystem = new DrivetrainSubsystem();
  shooterSubsystem = new ShooterSubsystem();
  subsystemManager = new SubsystemManager(drivetrainSubsystem);
  intakeSubsystem = new IntakeSubsystem();
  objectTrackerSubsystem = new ObjectTrackerSubsystem();
  colorDetectorSubsystem = new ColorDetectorSubsystem();
}

private void initCommands() {
    // recordCommand = new ToggleDriveRecordCommand();
    zeroCommand = new ZeroFieldOrientedCommand(drivetrainSubsystem);
    driveCommand = new HolonomicDriveCommand(DrivetrainSubsystem.ControlMode.DualStick);
    visionRotationDriveCommand = new VisionRotationDriveCommand();
    robotRotateCommand = new RobotRotateCommand(90);

    shooterWithVisionCommand = new ShooterCommand(true);
    shooterNoVisionCommand = new ShooterCommand(false);
    // shooterIdleCommand = new ShooterIdleCommand();
    // shooterIdleCommand_ZERO_RPM = new ShooterIdleCommand(0.0);
    reverseShooterCommand = new ShooterIdleCommand(-1000);
  
    extendClimberCommand = new ClimberActuateCommand(false, 3);
    retractClimberCommand = new ClimberActuateCommand(true, 3);


    intakeInCommand = new IntakeCommand(false);
    intakeOutCommand = new IntakeCommand(true);
    // intakeInCommand.isRunning();

    extendIntakeCommand = new IntakeActuateCommand(true, 3); // extend
    retractIntakeCommand = new IntakeRetractCommand2(3); // retract 
    midStateIntakeCommand = new IntakeRetractCommand(3, true);
    // middleIntakeCommand = new IntakeActuateCommand(IntakePosition.Middle, 3);

    // indexZoneCommand = new IndexZoneCommand();
 }

private void initButtons() {
    //oi.bedForwardButton.toggleWhenPressed(bedForwardCommand);
    // oi.toggleDriveRecordButton.toggleWhenPressed(recordCommand);
    oi.visionDriveButton.whileHeld(visionRotationDriveCommand);
    
    //oi.intakeButton.whileHeld(intakeCommandGroup);
    oi.intakeActuateUpButton.whenPressed(retractIntakeCommand);
    oi.intakeActuateDownButton.whileHeld(extendIntakeCommand); // TODO should these by whenPresed? noticed 3/20 (Megan)
    oi.intakeMidStateButton.whileHeld(midStateIntakeCommand);
    // oi.intakeActuateMiddleButton.whenPressed(lowerIntakeCommand);


    oi.intakeInButton.whileHeld(intakeInCommand);
    oi.intakeOutButton.whileHeld(intakeOutCommand);
    //  oi.intakeDetectButton.whileHeld(intakeDetectToElevatorIndexCommand);

     

    //oi.helloArcButton.whileHeld(robotRotateCommand);
    oi.referenceResetButton.whenPressed(zeroCommand);
    oi.shooterNoVisionButton.whileHeld(shooterNoVisionCommand);
    oi.shooterVisionButton.whileHeld(shooterWithVisionCommand);
    // oi.indexZoneButton.whenPressed(indexZoneCommand);
    //oi.snapShotButton.whenPressed(snapshotCommand);
    // oi.shooterIdleButton.whenPressed(shooterIdleCommand);
    // oi.shooterZERORPMButton.whenPressed(shooterIdleCommand_ZERO_RPM);
    oi.extendClimberButton.whenPressed(extendClimberCommand);
    oi.retractClimberButton.whenPressed(retractClimberCommand);
    oi.shooterReverseButton.whileHeld(reverseShooterCommand);

}

private void initChooser() {
  m_chooser = new SendableChooser<>();

  // vision auto TESTS
  /*
  m_chooser.addOption("VISION - FetchCargoCommand test RED cargo", AutonomousSequences.testFetchCargoCommand2RED());
  m_chooser.addOption("VISION - FetchCargoCommand test BLUE cargo", AutonomousSequences.testFetchCargoCommand2BLUE());
  m_chooser.addOption("fcc new command", AutonomousSequences.testVisionDriveToPickUpCargo());
  m_chooser.addOption("fcc OLD command", AutonomousSequences.testfccOLD());
  m_chooser.addOption("fcc rotate fcc", AutonomousSequences.twoFccOld()); 
  */

  // m_chooser.addOption("fcc OLD command test RED", AutonomousSequences.testfccOLD());


  // traditional auto
  m_chooser.addOption("Two Ball Auto Arc", AutonomousSequences.twoBallAutoWithArc(false));
  m_chooser.addOption("Two Ball Auto Arc VISION", AutonomousSequences.twoBallAutoWithArc(true));

  m_chooser.addOption("Shoot Collect Right", AutonomousSequences.shootCollectRight());
  m_chooser.addOption("Shoot Collect Left", AutonomousSequences.shootCollectLeft());
  m_chooser.addOption("Shoot and Collect and Shoot two Cargo", AutonomousSequences.shootCollectShootTwoCargo());
  m_chooser.addOption("Drive Straight Then Back", AutonomousSequences.driveStraightThenBack());
  m_chooser.addOption("Rotate 360", AutonomousSequences.rotate360());
  m_chooser.addOption("arc test", AutonomousSequences.arcTest());
  m_chooser.addOption("Shoot Arc Collect Shoot", AutonomousSequences.shootCollectRightNoRotation());
  m_chooser.addOption("Rotate and drive straight", AutonomousSequences.straightLineRotationTest());
  m_chooser.addOption("shoot one pass tarmac", AutonomousSequences.shootPassTarmacLine());

  // vision auto sequences
  m_chooser.addOption("two ball auto RED", AutonomousSequences.twoBallAuto("red", 65, -80));
  m_chooser.addOption("two ball auto BLUE", AutonomousSequences.twoBallAuto("blue", 65, -80));

  m_chooser.addOption("two ball auto RED 2", AutonomousSequences.twoBallAuto("red", -65, 80));
  m_chooser.addOption("two ball auto BLUE 2", AutonomousSequences.twoBallAuto("blue", -65, 80));

  m_chooser.addOption("two ball auto RED 3", AutonomousSequences.twoBallAuto("red", -55, 80));
  m_chooser.addOption("two ball auto BLUE 3", AutonomousSequences.twoBallAuto("blue", -55, 80));
  // FRANK TEST m_chooser.addOption("short two ball auto red", AutonomousSequences.twoBallAutoShort("red", 100, -120));
  // m_chooser.addOption("one ball auto drive like hell to terminal 2 RED", AutonomousSequences.shootDriveLikeHellToTerminal2("red"));
  // m_chooser.addOption("one ball auto drive like hell to terminal 2 BLUE", AutonomousSequences.shootDriveLikeHellToTerminal2("blue"));

  // end of vision auto sequences

  // m_chooser.addOption("limelight test", AutonomousSequences.limelightTest());

  //  m_chooser.addOption("Shoot, Collect Right, Shoot Again ", AutonomousSequences.ShootThenCollectRight_ThenShootAgain());f
  //  m_chooser.addOption("Leave Initiation Line", AutonomousSequences.backAwayFromInitiationLine());
  //  m_chooser.addOption("Shoot from Right, Collect Right, Shoot Again", AutonomousSequences.ShootFromRight_Of_Optimal_Then_Collect());
  //  m_chooser.addOption("Shoot Then Leave Initiation Line", AutonomousSequences.shootThenBackAwayFromInitiationLine());
  //  m_chooser.addOption("Shoot, Collect Left", AutonomousSequences.ShootThenCollectLeft());
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
    subsystemManager.outputToSmartDashboard();
    //drivetrainSubsystem.outputToSmartDashboard();
    colorDetectorSubsystem.outputColor();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.;
   */
  @Override
  public void disabledInit() {
    Robot.drivetrainSubsystem.getFollower().cancel();

    //TODO: add logic in "robotDisable"
    //to set Solenoids to desired state.
    Robot.intakeSubsystem.midState();
    Robot.climberSubsystem.retractClimber();
    System.out.println("disabled init");
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0.0, true);

    subsystemManager.disableKinematicLoop();
    vision.ledOff();
    autoHappened = false;
    
    
  }

  

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    // if(vision.getLed() != 1)
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
    // autonomousCommand = AutonomousSequences.shootCollectRight();
    autonomousCommand = m_chooser.getSelected();
    Robot.climberSubsystem.retractClimber();

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
    Robot.climberSubsystem.retractClimber();

    vision.ledOn();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // if(autoHappened){
      //  reverseZeroCommand.start(); // *** uncomment for reversing bot so shooter is "front"
    // }
    Robot.drivetrainSubsystem.getFollower().cancel();

    // SmartDashboard.putNumber("ShooterMotor1", RobotMap.SHOOTER_MOTOR_HIGH_DEFAULT_SPEED);
    


    subsystemManager.enableKinematicLoop(UPDATE_DT);
    zeroCommand.start(); // *** comment out for reversing so shooter is "front"
   // SmartDashboard.putString("Path", PathSelecter.choosePath());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    Scheduler.getInstance().run();
    subsystemManager.outputToSmartDashboard();
    // Vector2 vec = drivetrainSubsystem.getKinematicPosition();
    // SmartDashboard.putNumber("Current Pose X", vec.x);
    // SmartDashboard.putNumber("Current Pose Y", vec.y);
    colorDetectorSubsystem.get_color();
    // SmartDashboard.putNumber("Distance from sensor", colorDetectorSubsystem.outputDistance()); 
    // SmartDashboard.putNumber("Navx Angle", drivetrainSubsystem.getGyroscope().getAngle().toDegrees());

   

    drivetrainSubsystem.outputToSmartDashboard();
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
