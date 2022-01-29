/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
public class ElevatorIndexCommand extends Command {
  boolean up = true;
  double index = 0;
  public ElevatorIndexCommand(boolean up, double index) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.up = up;
    this.index = index;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevatorSubsystem.setForIndex(up, index);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevatorSubsystem.PIDDrive();
    if(up) {
      Robot.intakeSubsystem.setKickerMotor(0.5);
    } else {
      Robot.intakeSubsystem.setKickerMotor(-0.5);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.elevatorSubsystem.getBeltEncoderValue() > (Robot.elevatorSubsystem.getSetpoint() - 1) && Robot.elevatorSubsystem.getBeltEncoderValue() < (Robot.elevatorSubsystem.getSetpoint() + 1)) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.intakeSubsystem.setKickerMotor(0);
    Robot.elevatorSubsystem.setBeltMotor(0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
