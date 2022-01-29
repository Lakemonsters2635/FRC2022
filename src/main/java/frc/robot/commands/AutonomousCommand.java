/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.control.Trajectory;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutonomousCommand extends CommandGroup {

  public AutonomousCommand(Trajectory trajectory, double endRotation) {
    requires(Robot.drivetrainSubsystem);

    AutonomousTrajectoryCommand trajectoryCommand = new AutonomousTrajectoryCommand(trajectory);
    RobotRotateCommand rotateCommand = new RobotRotateCommand(endRotation);

    addSequential(trajectoryCommand);
    addSequential(rotateCommand);
  }
}
