/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;

import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends Command {
  private Vector2 translation;
  private double rotation;
  private boolean fieldOriented;

  public DriveCommand(Vector2 translation, double rotation, boolean fieldOriented) {
      this.translation = translation;
      this.rotation = rotation;
      this.fieldOriented = fieldOriented;

      requires(Robot.drivetrainSubsystem);
  }

  @Override
  protected void initialize() {
      Robot.drivetrainSubsystem.holonomicDrive(translation, rotation, fieldOriented);
  }

  @Override
  protected void end() {
      Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO,  0.0, true);
  }

  @Override
  protected boolean isFinished() {
      return false;
  }
}
