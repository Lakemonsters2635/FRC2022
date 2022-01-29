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

public class IntakeDetectCommand extends Command {
  int counter;
  int detectCounter;
  boolean current;
  public IntakeDetectCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    detectCounter = 0;
    counter = 0;
    current = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.intakeSubsystem.setIntakeMotor(0.5);
    if(!current) {
      Robot.intakeSubsystem.setKickerMotor(0.5);
    } 
    //SmartDashboard.putNumber("Kicker Current", Robot.intakeSubsystem.getKickerCurrent());
    counter++;

    if(Robot.intakeSubsystem.getKickerCurrent() > 6 && counter > 10) {
      current = true;
    }
      

    if(current) {
      Robot.elevatorSubsystem.setBeltMotor(-1);
      Robot.intakeSubsystem.setKickerMotor(0.5);
      detectCounter++;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    // if (Robot.elevatorSubsystem.powerCellCount>=4) {
    //   return true;
    // }
    
    if(current && detectCounter > 20) {
      Robot.elevatorSubsystem.powerCellCount++;
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.intakeSubsystem.setIntakeMotor(0);
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
