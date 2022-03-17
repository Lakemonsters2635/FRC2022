/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private boolean useCamera = false;
  private double targetDistance = 0;
  private double m_motorSpeed = 0;
  private double m_lowerMotorSpeed = 0;
  private boolean shootHigh = true;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(boolean useCamera) {
    this.useCamera = useCamera;
    m_motorSpeed = RobotMap.SHOOTER_MOTOR_HIGH_DEFAULT_SPEED;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  //Constructor for Autonomous, contains a timeout and motorspeed.
  public ShooterCommand(boolean useCamera, double timeout, double motorSpeed) {
    super(timeout);
    this.useCamera = useCamera;
    m_motorSpeed = motorSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    // System.out.println("shooter command init");
    if (m_motorSpeed == 0) {
      m_motorSpeed = 3000;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftJoystickZ = Robot.oi.leftStick.getZ();
    leftJoystickZ = leftJoystickZ + 1;
    double motor1Speed = m_motorSpeed + leftJoystickZ * 2500;
    Robot.shooterSubsystem.SpinShooter(motor1Speed);
    // System.out.println("shooter command " + motor1Speed);
  }

  public double computeShooterSpeedFromTargetDistance(double targetDistance, boolean isShooterHigh) {
    double adjustedMotorSpeed;
    if(isShooterHigh) {
      adjustedMotorSpeed = 2.0693 * targetDistance + 1418.6;
      //adjustedMotorSpeed = 2.0693 * targetDistance + 1400.6;
    }
    else {
      adjustedMotorSpeed = 4000;
    }
    
    return adjustedMotorSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    Robot.shooterSubsystem.stop();
    Robot.shooterSubsystem.configureMotors();

    
    if (useCamera) {
      Robot.vision.ledOff();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = super.isTimedOut();
    
    if (isFinished) {
      // System.out.println("ShooterCommand finished");
    }
    return false;
    //return super.isTimedOut();
  }
}
