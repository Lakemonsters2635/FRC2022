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

public class IndexZoneCommand extends Command {
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexZoneCommand() {
    
  }

  //Constructor for Autonomous, contains a timeout and motorspeed.
  public IndexZoneCommand(double timeout) {
    super(timeout);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    Robot.shooterSubsystem.indexZone();
    // SmartDashboard.putNumber("motor1Speed", m_upperMotorSpeed);
    // SmartDashboard.putNumber("motor2Speed", m_lowerMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    }

    // yellow zone: upper motor - 1750, lower motor - 1300 (raised shooter)
    // blue zone: upper motor - 2200, lower motor - 1600 (lowered shooter)
    // red zone: upper motor - 2200, lower motor - 1700 (lowered shooter)


 // }


  // Called once the command ends or is interrupted.
  @Override
  public void end() {
     
    super.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // return super.isTimedOut();
    //return super.isTimedOut();
    return true;
  }
  @Override
  protected void interrupted() {
    end();
  }
}
