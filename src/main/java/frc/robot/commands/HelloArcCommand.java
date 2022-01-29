/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.control.Trajectory;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import org.frcteam2910.common.math.Vector2;

import frc.robot.models.AutonomousTrajectories;

public class HelloArcCommand extends Command {
  Trajectory trajectory;
  public HelloArcCommand() {
    requires(Robot.drivetrainSubsystem);

    AutonomousTrajectories trajectoryLibrary = new AutonomousTrajectories(Robot.drivetrainSubsystem.CONSTRAINTS);
    trajectory = trajectoryLibrary.getHelloTrajectory();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle());
    Vector2 position = new Vector2(0, 0);
    Robot.drivetrainSubsystem.resetKinematics(position, 0);


    Robot.drivetrainSubsystem.getFollower().follow(trajectory);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0.0, true);
    Robot.drivetrainSubsystem.getFollower().cancel();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
