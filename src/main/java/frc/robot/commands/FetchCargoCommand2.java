// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.models.VisionObject;
import frc.robot.subsystems.Vision;

public class FetchCargoCommand2 extends Command {
  /** Creates a new FetchCargoCommand2. 
   *  Grabs either the closest object of specified color or the closest object in a given range
   *  Dead-reckons to cargo
   *  
   *  Not tested on robot yet as of 3/17
  */
  // ***ALL DISTANCE UNITS SHOULD BE IN INCHES***

  private String cargoColor; 
  private Vector2 targetPosition; 
  private double targetRotation; 
  private VisionObject targetCargo; 
  private double[] distanceInRange = {-1, -1}; // [lower bound, upper bound]
  // {-1, -1} means get the closest object - special case placeholder

  private boolean noCargoFound = false; 
  private double threshold = 50.0; // how close the bot needs to get to cargo before terminating command

  public FetchCargoCommand2(String cargoColor, double timeOut) {
    super(timeOut);
    this.cargoColor = cargoColor; 
    checkUpdateCargoColor(this.cargoColor);
    requires(Robot.drivetrainSubsystem);
    setInterruptible(true);  

  }

  public FetchCargoCommand2(String cargoColor, double lowerDistanceBound, double upperDistanceBound, double timeOut) {
    super(timeOut); 
    this.cargoColor = cargoColor; 
    checkUpdateCargoColor(this.cargoColor);
    distanceInRange[0] = lowerDistanceBound; 
    distanceInRange[1] = upperDistanceBound;
    requires(Robot.drivetrainSubsystem);
    setInterruptible(true);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // puts data onto network tables and applies rotation/translation matrix
    Robot.objectTrackerSubsystem.data(); 

    // gets closest object in specified range
    // else get absolute closest object if the range is the special -1 to 1 default case
    if (distanceInRange[0] != -1 && distanceInRange[1] != -1) {
      VisionObject[] cargoList = Robot.objectTrackerSubsystem.getObjectsOfType(this.cargoColor);
      for (int i = 0; i < cargoList.length; i++) {
        if (cargoList[i].z > distanceInRange[0] && cargoList[i].z < distanceInRange[1]) {
          this.targetCargo = cargoList[i]; 
          break; 
        }
      }
    } else { 
      this.targetCargo = Robot.objectTrackerSubsystem.getClosestObject(this.cargoColor); 
    }
    
    if (targetCargo == null) {
      System.out.println("No cargo found"); 
      noCargoFound = true; // ends command
    }

    // zero drivetrain: is this necessary? also may be better to put at top of initialize() 
    Robot.drivetrainSubsystem.resetKinematics(Vector2.ZERO, 0);
  
    // find angle bot rotates to line up with cargo
    this.targetRotation = Math.toDegrees(Math.atan2(targetCargo.x, targetCargo.z)); // in camera coordinates

    // TODO may need to negate either y or z - don't remember which direction is negative on bot
    this.targetPosition = new Vector2(-targetCargo.z, targetCargo.x); // in camera coordinates
  
    // dead reckoning drive to target
    // TODO may need to add an offset to stop before reaching cargo
    SmartDashboard.putString("FetchCargoCommand2 target position Vector2", this.targetPosition.toString());
    SmartDashboard.putNumber("FetchCargoCommand2 rootation angle", this.targetRotation);
    
    Robot.drivetrainSubsystem.holonomicDrive(this.targetPosition, 0.0, false); // camera returns robot-centric coordinates
    // Robot.drivetrainSubsystem.holonomicDrive(new Vector2(0.0, 40.0), 0.0, false); // camera returns robot-centric coordinates
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("FCC2 z (corrected w/ matrix)", targetCargo.z);
    SmartDashboard.putNumber("FCC2 y (corrected w/ matrix)", targetCargo.y);
    SmartDashboard.putNumber("FCC2 x (corrected w/ matrix)", targetCargo.x);
  }

  // Called once the command ends.
  @Override
  public void end() {
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0, true);
    System.out.println("FetchCargoCommand2 done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (noCargoFound) {
      return true;
    } else if (Robot.drivetrainSubsystem.autonomousDriveFinished(targetPosition, this.threshold)) {
      return true;
    } else {
      super.isTimedOut(); 
    }
    return false; 
  }

  private void checkUpdateCargoColor(String color) {
    // checks cargoColor given to frconstructor, normalizes case
    if (color.equalsIgnoreCase("blue")) {
      this.cargoColor = "blue";
    } else if (color.equalsIgnoreCase("red")) {
      this.cargoColor = "red"; 
    } else {
      // TODO some time of catch all end case that deals with a non red/blue input?
      // ^ low priority
    }
  }
}
