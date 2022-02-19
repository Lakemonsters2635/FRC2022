/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class IntakeExtendCollectCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public IntakeExtendCollectCommand() {
    IntakeActuateCommand intakeExtendCommand = new IntakeActuateCommand(false, 1.5);
    IntakeCommand intakeCommand = new IntakeCommand(true, 3);
    IntakeCommand intakeCommand2 = new IntakeCommand(true, 3);

    IntakeActuateCommand intakeRetractCommand = new IntakeActuateCommand(true, 1.5);
    addParallel(intakeExtendCommand);
    addSequential(intakeCommand);
    addSequential(intakeCommand2);
    addSequential(intakeRetractCommand);
  }

  public IntakeExtendCollectCommand(int timeout) {
    IntakeDetectCommand intakeDetectCommand = new IntakeDetectCommand();
    ElevatorIndexCommand elevatorIndexCommand = new ElevatorIndexCommand(true, 35);

    addSequential(intakeDetectCommand, timeout);
    addSequential(elevatorIndexCommand, timeout);
  }
}
