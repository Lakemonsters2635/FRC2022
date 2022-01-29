/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.ControlMode;

public class HolonomicDriveCommand extends Command {
  ControlMode mode;

  public HolonomicDriveCommand(ControlMode mode) {
    requires(Robot.drivetrainSubsystem);
    this.mode = mode;
    
}

@Override
protected void execute() {
  double forward = 0;
  double strafe = 0;
  double rotation = 0;      

  //boolean ignoreScalars = Robot.oi.primaryController.getforwardBumperButton().get();

  //Comment this to deactivate teleop drive
  switch(mode) {
    case DualStick: 
       forward = Robot.oi.leftStick.getRawAxis(1);
       strafe = Robot.oi.leftStick.getRawAxis(0);
       rotation = Robot.oi.rightStick.getRawAxis(0);
      break;
    case SingleStick: 
       forward = Robot.oi.leftStick.getRawAxis(1);
       strafe = Robot.oi.leftStick.getRawAxis(0);
       rotation = Robot.oi.leftStick.getRawAxis(2);
       break; 
    case Controller:
      forward = Robot.oi.leftStick.getRawAxis(1);
      strafe = Robot.oi.leftStick.getRawAxis(0);
      rotation = Robot.oi.leftStick.getRawAxis(4);  
      break;    
      
  }


  double deadzone = 0.05;
  
  forward = deadZoneAdjust(forward, deadzone);
  strafe = deadZoneAdjust(strafe, deadzone);
  rotation = deadZoneAdjust(rotation, deadzone);

  
  boolean robotOriented = false;
  boolean reverseRobotOriented = false;

  Vector2 translation = new Vector2(forward, strafe);

  // if (reverseRobotOriented) {
  //     robotOriented = true;
  // }
  //     translation = translation.rotateBy(Rotation2.fromDegrees(180.0));
  Robot.drivetrainSubsystem.holonomicDrive(translation, rotation,  !robotOriented);
  //System.out.println("HoloDriveCommand.execute" + translation + " " + rotation);
  
}

public double deadZoneAdjust(double input, double deadzone) {
  if(input > deadzone) {
    input = ((input - deadzone)/(1.0 - deadzone));
  } else if(input < -deadzone) {
    input = ((input + deadzone)/(1.0 - deadzone));
  } else {
    input = 0;
  }    
  return input;
}

@Override
protected boolean isFinished() {
    return false;
}

}
