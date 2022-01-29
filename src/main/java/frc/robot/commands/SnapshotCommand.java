// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class SnapshotCommand extends Command {
  /** Creates a new SnapshotCommand. */
  public SnapshotCommand() {
    SmartDashboard.putString("Snapshot File Name","");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String fileName = SmartDashboard.getString("Snapshot File Name", "");
    try {
      Robot.objectTrackerSubsystem.saveVisionSnapshot(fileName);
    } catch (IOException e) {
      System.out.println("hey you failed");
    }   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
