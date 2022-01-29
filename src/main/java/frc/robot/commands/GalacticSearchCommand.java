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
import frc.robot.models.AutonomousSequences;
import frc.robot.models.PathSelecter;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GalacticSearchCommand extends Command {

  private CommandGroup commandSequence;
  String pathName;
  //NOTE: This command rotates to an absolute angle based on the orientation the robot started in. Will work in Auto, must be adapted for Teleop
  public GalacticSearchCommand(double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(timeout);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    pathName = PathSelecter.choosePath();
    if (pathName == null){
      System.out.println("path selector returns no sequence");
      end();
      return;
    }
    
     if (pathName.equals("PathRedB")) {
       commandSequence = AutonomousSequences.GalacticSearchRedPathB();
     } else if (pathName.equals("PathRedA")){
      commandSequence = AutonomousSequences.GalacticSearchRedPathA();
     } else if (pathName.equals("PathBlueB")){
      commandSequence = AutonomousSequences.GalacticSearchBluePathB();
     } else if (pathName.equals("PathBlueA")){
      commandSequence = AutonomousSequences.GalacticSearchBluePathA();

     }

     // commandSequence.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if (commandSequence == null){
        end();
      }
      if (!commandSequence.isRunning() && !(commandSequence.isCompleted() || commandSequence.isCanceled())){
        System.out.println("**************************************");
        System.out.println("******* SEQUENCE STARTED    **********");
        System.out.println("**************************************");
        commandSequence.start();
      }
    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean finished = false;

    //return commandSequence.isFinished();
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
