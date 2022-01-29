/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class FetchCollectPowerCellCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FetchCollectPowerCellCommand() {
    FetchPowerCellCommand fetchPowerCellCommand = new FetchPowerCellCommand();
    AreWeThereYetCommand areWeThereYetCommand = new AreWeThereYetCommand();
    IntakeActuateCommand lowerIntakeCommand = new IntakeActuateCommand(false,1);
    IntakeDetectCommand intakeDetectCommand = new IntakeDetectCommand();
    ElevatorIndexCommand elevatorIndexCommand = new ElevatorIndexCommand(true, 35);

    addParallel(fetchPowerCellCommand);
    addSequential(areWeThereYetCommand);
    addSequential(lowerIntakeCommand);
    addSequential(intakeDetectCommand);
    addSequential(elevatorIndexCommand);
  }

  public FetchCollectPowerCellCommand(int timeout) {
    AreWeThereYetCommand areWeThereYetCommand = new AreWeThereYetCommand();
    IntakeActuateCommand lowerIntakeCommand = new IntakeActuateCommand(false,1);
    IntakeDetectCommand intakeDetectCommand = new IntakeDetectCommand();
    ElevatorIndexCommand elevatorIndexCommand = new ElevatorIndexCommand(true, 35);
    
    addSequential(areWeThereYetCommand, timeout);
    addSequential(lowerIntakeCommand, timeout);
    addSequential(intakeDetectCommand, timeout);
    addSequential(elevatorIndexCommand, timeout);
  }
}