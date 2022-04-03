// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class ShooterIdleCommand extends Command {
  /** Creates a new ShooterIdleCommand. */
  private double m_idleShooterSpeed;
  private boolean is_idle_done;

  public ShooterIdleCommand() {
    this(1000);
  }

  public ShooterIdleCommand(double idleShooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // is_idle_done = false;

    is_idle_done = false;
    m_idleShooterSpeed = idleShooterSpeed;
    requires(Robot.shooterSubsystem);
    setInterruptible(true);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("shooter idle command init");
    // is_idle_done = false;
    // Robot.shooterSubsystem.configureMotors();
    // System.out.println(m_idleShooterSpeed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Robot.shooterSubsystem.SpinShooter(m_idleShooterSpeed);


    // if (m_idleShooterSpeed == 0.0) {
    //   is_idle_done = true;
      // System.out.println(is_idle_done);

    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    // System.out.println("reached end function");
    Robot.shooterSubsystem.stop();
    // Robot.shooterSubsystem.configureMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
