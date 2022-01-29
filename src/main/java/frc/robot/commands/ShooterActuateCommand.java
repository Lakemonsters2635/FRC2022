/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShooterActuateCommand extends Command {
  private boolean m_raiseShooter = false;
  public ShooterActuateCommand(boolean raiseShooter, double timeOut) {
    super(timeOut);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_raiseShooter = raiseShooter;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    System.out.println("Actuate Shooter Initialized.");
    if (m_raiseShooter) {
      Robot.shooterSubsystem.aimHigh();
    } else {
      Robot.shooterSubsystem.aimLow();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean isFinished = super.isTimedOut();
    if (isFinished) {
      System.out.println("ShooterActuateCommand finished");
    }
    //return super.isTimedOut();
    return isFinished; 
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
   
    super.end();
  }


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
