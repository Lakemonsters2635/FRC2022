/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class RobotRotateCommand extends Command {
  PIDController angleController;
  double targetAngle;
  double currentAngle;

  //NOTE: This command rotates to an absolute angle based on the orientation the robot started in. Will work in Auto, must be adapted for Teleop
  public RobotRotateCommand(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(3);
    requires(Robot.drivetrainSubsystem);
    angleController = new PIDController(0.005, 0.002, 0.0);
    angleController.enableContinuousInput(-180, 180);
    this.targetAngle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Rotation2.fromDegrees(offset + drivetrain.getGyroscope().getUnadjustedAngle().toDegrees()));
    Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle());

    System.out.println("Rotation initialized.");
    //Vector2 position = new Vector2(0, 0);
    //Robot.drivetrainSubsystem.resetKinematics(position, 0);
    angleController.setSetpoint(targetAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentAngle = Robot.drivetrainSubsystem.getGyroscope().getAngle().toDegrees();
    System.out.println("current angle: " + currentAngle);
    //boolean fieldOriented = false;
    //Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, angleController.calculate(currentAngle), fieldOriented);
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, angleController.calculate(currentAngle));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
     if (super.isTimedOut())
     {
      System.out.println("Rotation timed out");
       return true;
     }
     double angleDelta = Math.abs(currentAngle - targetAngle);
    //if(currentAngle > targetAngle - 2 && currentAngle < targetAngle + 2 )

    double omega = Robot.drivetrainSubsystem.getGyroscope().getRate();
    System.out.println("Omega : " + omega);
    // if ((angleDelta < 2 || (angleDelta > 358 && angleDelta < 362)) && Math.abs(omega) < 0.003)

    if ((angleDelta < 2 || (angleDelta > 358 && angleDelta < 362)))
    {
      System.out.println("Rotation finished");
      return true;
    } 
    else 
    {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
