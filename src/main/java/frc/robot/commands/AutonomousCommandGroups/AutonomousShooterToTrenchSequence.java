/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousCommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutonomousTrajectoryCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.models.AutonomousTrajectories;

public class AutonomousShooterToTrenchSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutonomousShooterToTrenchSequence() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    AutonomousTrajectories autonomousTrajectories = new AutonomousTrajectories(Robot.drivetrainSubsystem.CONSTRAINTS);

    ShooterCommand shooterCommand = new ShooterCommand(false);
    ElevatorCommand elevatorUpCommand = new ElevatorCommand(false);
    AutonomousTrajectoryCommand shootingToTrenchPickupCommand = new AutonomousTrajectoryCommand(autonomousTrajectories.getShootingToTrenchPickupTrajectory());
    
    addParallel(elevatorUpCommand, 5);
    addSequential(shooterCommand,5);
    addSequential(shootingToTrenchPickupCommand);
  }
}
