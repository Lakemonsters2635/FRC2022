/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorCommand extends Command {
  private boolean m_reverse = false;
  public ElevatorCommand(boolean reverse) {
    m_reverse = reverse;
  }

  public ElevatorCommand(boolean reverse, double timeout) {
    super(timeout);
    m_reverse = reverse;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //System.out.println("Elevator Down command init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (m_reverse) {
      Robot.elevatorSubsystem.setBeltMotor(0.8);
      Robot.intakeSubsystem.setKickerMotor(-0.8);
    } else {
      Robot.elevatorSubsystem.setBeltMotor(-0.8);
      Robot.intakeSubsystem.setKickerMotor(0.8);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean isFinished = super.isTimedOut();
    if (isFinished) {
      System.out.println("ElevatorCommand finished");
    }
    return isFinished; 
    //return super.isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //System.out.println("Elevator Down command end");
    Robot.elevatorSubsystem.setBeltMotor(0);
    Robot.intakeSubsystem.setKickerMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
