/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeCommand extends Command {
  private boolean m_reverse = false;
  public IntakeCommand(boolean reverse) {
    m_reverse = reverse;
  }

  public IntakeCommand(boolean reverse, double timeout) {
    super(timeout);
    m_reverse = reverse;
  }

  // Called just before this Command runs the first time
  @Override
  
  protected void initialize() {
    boolean m_intakeExtended = Robot.intakeSubsystem.intakeIsExtended();
    System.out.println("!!!!!!!!!!!!!!!!!!!IntakeCommand initialized: " + m_intakeExtended);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean m_intakeExtended = Robot.intakeSubsystem.intakeIsExtended();
    System.out.println("IntakeSubsystem.isExtended: " + m_intakeExtended);
    // if (m_intakeExtended) {
      if (m_reverse) {
        Robot.intakeSubsystem.setIntakeMotor(0.95);
      } else {
        Robot.intakeSubsystem.setIntakeMotor(-0.95);
      }
    // } else {
    //   Robot.intakeSubsystem.setIntakeMotor(0.0);
    // }
    
   
    //Robot.elevatorSubsystem.setBottomKickerMotor(1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return super.isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.intakeSubsystem.setIntakeMotor(0);
  }


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
