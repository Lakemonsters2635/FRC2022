/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class IntakeActuateCommand extends Command {
  private boolean m_raiseIntake = false;
  private IntakePosition m_intakePosition;
  public IntakeActuateCommand(boolean raiseIntake, double timeOut) {
    super(timeOut);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_raiseIntake = raiseIntake;
  }


  public IntakeActuateCommand(IntakeSubsystem.IntakePosition position, double timeOut) {
    super(timeOut);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_intakePosition = position;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (m_raiseIntake) {
      Robot.intakeSubsystem.extendIntake();
    } else {
      Robot.intakeSubsystem.retractIntake();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return super.isTimedOut();
  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  //  if(m_intakePosition == IntakePosition.Middle) {
  //    Robot.intakeSubsystem.midState();
  //  }
    super.end();
  }


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
