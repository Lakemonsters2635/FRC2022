/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class IntakeDetectToElevatorIndexCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public IntakeDetectToElevatorIndexCommand() {
    IntakeDetectCommand intakeDetectCommand = new IntakeDetectCommand();
    ElevatorIndexCommand elevatorIndexCommand = new ElevatorIndexCommand(true, 15);

    addSequential(intakeDetectCommand);
    addSequential(elevatorIndexCommand);
  }

  public IntakeDetectToElevatorIndexCommand(int timeout) {
    IntakeDetectCommand intakeDetectCommand = new IntakeDetectCommand();
    ElevatorIndexCommand elevatorIndexCommand = new ElevatorIndexCommand(true, 35);

    addSequential(intakeDetectCommand, timeout);
    addSequential(elevatorIndexCommand, timeout);
  }
}
