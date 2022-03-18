// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
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

  public FetchCargoCommand2(String cargoColor) {
    this.cargoColor = cargoColor; 
    checkUpdateCargoColor(this.cargoColor);
    requires(Robot.drivetrainSubsystem);
  }

  public FetchCargoCommand2(String cargoColor, double lowerDistanceBound, double upperDistanceBound) {
    this.cargoColor = cargoColor; 
    checkUpdateCargoColor(this.cargoColor);
    distanceInRange[0] = lowerDistanceBound; 
    distanceInRange[1] = upperDistanceBound;
    requires(Robot.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // puts data onto network tables and applies rotation/translation matrix
    Robot.objectTrackerSubsystem.data(); 

    if (distanceInRange[0] != -1 && distanceInRange[1] != -1) {
      VisionObject[] cargoList = Robot.objectTrackerSubsystem.getObjectsOfType(this.cargoColor);
      for (int i = 0; i < cargoList.length; i++) {
        if (cargoList[i].z > distanceInRange[0] && cargoList[i].z < distanceInRange[1]) {
          this.targetCargo = cargoList[i]; 
          break; 
        }
      }
    } else { // get closest object if the range is the special -1 to 1 default case
      this.targetCargo = Robot.objectTrackerSubsystem.getClosestObject(this.cargoColor); 
    }
    
    if (targetCargo == null) {
      System.out.println("No cargo found"); 
      return; 
    }

    // zero drivetrain: is this necessary?
    Robot.drivetrainSubsystem.resetKinematics(Vector2.ZERO, 0);
  
    // find angle bot rotates to line up with cargo
    this.targetRotation = Math.atan2(targetCargo.y, targetCargo.z);

    // TODO may need to negate either y or z - don't remember which direction is negative on bot
    this.targetPosition = new Vector2(targetCargo.y, targetCargo.z); 
  
    // dead reckoning drive to target
    // TODO may need to add an offset to stop before reaching cargo
    Robot.drivetrainSubsystem.holonomicDrive(this.targetPosition, this.targetRotation, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends.
  @Override
  public void end() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void checkUpdateCargoColor(String color) {
    // checks cargoColor given to constructor, normalizes case
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
