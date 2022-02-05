/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.models.VisionObject;

// import com.kauailabs.navx.frc.AHRS;  
// import edu.wpi.first.wpilibj.SPI;

public class FetchPowerCellCommand extends Command {
  PIDController angleController;
  PIDController strafeController;
  PIDController forwardController; 
  double gyroAngle;
  double angle;
  double desiredAngle;
  double setPointAngle=6;
  boolean isClose;
  double v;

  public double totalRotation = 0;
  public FetchPowerCellCommand() {
    requires(Robot.drivetrainSubsystem);
    //initPID();    
  }

  public FetchPowerCellCommand(double timeout) {
    super(timeout);
    requires(Robot.drivetrainSubsystem);
    //initPID();
    
  }

  protected void initPID(){
    angleController = new PIDController(0.25, 0.0, 0.0);
    strafeController = new PIDController(0.009, 0.0, 0.0); // TODO update constants
    forwardController = new PIDController(0.05, 0.01, 0.0); // TODO update constants
   
  }

  @Override
  protected void initialize() {
    initPID();
    System.out.println("FPC start");
    
    // SmartDashboard.putNumber("Vision angle", angle);
    // SmartDashboard.putNumber("Desired angle", desiredAngle);
    // SmartDashboard.putNumber("initial angle", gyroAngle);
    // SmartDashboard.putNumber("SetPoint angle", setPointAngle);
    
    Vector2 position = new Vector2(0, 0);
    Robot.drivetrainSubsystem.resetKinematics(position, 0);
    System.out.println("Initialized FPC");

    isClose = false;
  }

  @Override
  protected void execute() {
    
    Robot.objectTrackerSubsystem.data();
    double forward = 0;
    double strafe = 0;
    double rotation = 0;
    VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
     if (closestObject == null) {
       SmartDashboard.putNumber("driveRotation", 99);
       Robot.drivetrainSubsystem.holonomicDrive(new Vector2(0,0), 0, false);
       return; // no object found
     }
     System.out.println("Closest z: " + closestObject.z);
     closestObject.motionCompensate(Robot.drivetrainSubsystem, true);
  
    double angle =  Math.atan2(closestObject.x, closestObject.z);
    
    angleController.setSetpoint(angle);
    rotation = angleController.calculate(0) * 0;
    
    if(rotation > 1){
      rotation = 1;
    }else if(rotation < -1){
      rotation = -1;
    }

    totalRotation += rotation;
    SmartDashboard.putNumber("driveRotation", rotation);
    
    // strafe
    strafeController.setSetpoint(closestObject.x);
    strafe = strafeController.calculate(0);

    if(strafe > 1){
      strafe = 1;
    }else if (strafe < -1){
      strafe = -1;
    }

    SmartDashboard.putNumber("driveStrafe", strafe);

    // forward
    //forwardController.setSetpoint(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE); // TODO figure out how to implement code that begins intake process 
    forward = forwardController.calculate(0);

    if(forward > 1){
      forward = 1;
    }else if (forward < -1){
      forward = -1;
    }

    SmartDashboard.putNumber("driveForward", forward);
    
    final boolean robotOriented = false;

    //final Vector2 translation = new Vector2(-forward, -strafe*0);
  
    v = -0.5;  
    
    //  if (closestObject.z < 60) {
    //    isClose = true;
    // //   v = -0.05;
     
    //  }
    final Vector2 translation = new Vector2(v, strafe);
    System.out.println("translation: " + translation);
    Robot.drivetrainSubsystem.holonomicDrive(translation, rotation, robotOriented);
  }


@Override
protected boolean isFinished() {
  double tolerance = 4;
  VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
  if(closestObject == null) {
    return false;
  }//TODO could lose sight for small amount of time causing command to finish early
  
  //boolean done = Math.abs(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE) <= tolerance;
  isClose = Math.abs(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE) <= tolerance;
  // if (done) {
  //   System.out.println("done FPC");
  // }
  if (isClose) {
    System.out.println("FPC done");
  }
  return isClose;
  // return false;
  // boolean isFinished = super.isTimedOut();
  // if (isFinished) {
  //   SmartDashboard.putNumber("totalRotation", totalRotation);
  // }
  //  return isFinished;
     // TODO: add the actual completion test code
}

@Override
  protected void end() {
    Robot.vision.ledOff();
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
