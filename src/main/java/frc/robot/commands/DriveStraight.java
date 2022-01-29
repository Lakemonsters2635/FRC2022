package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import java.util.ArrayList;

import org.frcteam2910.common.math.Vector2;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class DriveStraight extends Command {
  Vector2 translation;
  double velocity;
  
    public DriveStraight(Vector2 translation, double velocity, double timeout) {
        super(timeout);
        requires(Robot.drivetrainSubsystem);
        this.translation = translation;
        this.velocity = velocity;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }
	
    // Called just before this Command runs the first time
    protected void initialize() {
        //Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle());
        Vector2 position = new Vector2(0, 0);
        Robot.drivetrainSubsystem.resetKinematics(position, 0);


        

    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        //Robot.drivetrainSubsystem.setTargetVelocity(velocity);
        Robot.drivetrainSubsystem.holonomicDrive(translation, 0.0, true);
    }

    // Called once after timeout
    protected void end() {
        Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0.0, true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0.0, true);
    }
    
    @Override protected boolean isFinished() {
        

        boolean isFinished =  Robot.drivetrainSubsystem.autonomousDriveFinished(translation);      
        
    	if(isFinished) {

//    		double currentHeading = Robot.drive.getNavxHeading();
//    		double navxHeading = Math.abs(intialHeading-currentHeading);
//    		System.out.println("Navx straight heading:" + navxHeading);
//    		System.out.println("Navx straight-drive angle: " + Robot.drive.getNavxAngle());
    		System.out.println("Drive Straight Finished.");
            System.out.println("-----------");
           
    	}
        
        if(isTimedOut()){
            System.out.println("Timed Out");
            isFinished = true;
        }

    	return isFinished;
    }
}