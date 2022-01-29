package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import java.util.ArrayList;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;



/**
 *
 */
public class AutonomousPathFollowCommand extends Command {
    ArrayList<HolonomicDriveSignal> driveSequence;
    int index;
 
  
    public AutonomousPathFollowCommand(ArrayList<HolonomicDriveSignal> driveSequence, double timeout) {
        super(timeout);
        requires(Robot.drivetrainSubsystem);
        this.driveSequence = driveSequence;

        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }
	
    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle());
        Vector2 position = new Vector2(0, 0);
        Robot.drivetrainSubsystem.resetKinematics(position, 0);
        index = 0;


        

    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        //Robot.drivetrainSubsystem.setTargetVelocity(velocity);
        if (index < driveSequence.size()) {
            Vector2 translation = driveSequence.get(index).getTranslation();
            Double rotation = driveSequence.get(index).getRotation();
            Robot.drivetrainSubsystem.holonomicDrive(translation, rotation, true);
            index++;
        } else {
            Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0, true);
        }
        

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


        
        boolean isFinished = (index >= driveSequence.size());
        if(isFinished) {
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