package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.models.AutonomousTrajectories;

import java.util.ArrayList;

import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.Trajectory.Segment;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.Side;



/**
 *
 */
public class AutonomousTrajectoryCommand extends Command {

    Trajectory autonomousTrajectory;
    PIDController angleController;
    private double intialRobotOrientation = 0;
    
    public AutonomousTrajectoryCommand(Trajectory trajectory) {
        super(trajectory.getDuration());
        requires(Robot.drivetrainSubsystem);
       
        autonomousTrajectory = trajectory;
        angleController = new PIDController(0.01, 0.01, 0);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    public AutonomousTrajectoryCommand(Trajectory trajectory, double orientation) {
        super(trajectory.getDuration());
        requires(Robot.drivetrainSubsystem);
        this.intialRobotOrientation = orientation;
        autonomousTrajectory = trajectory;
        angleController = new PIDController(0.01, 0.01, 0);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }
	
    // Called just before this Command runs the first time
    protected void initialize() {
        //FHE: Enable the following line to for drive/rotate/drive test
         Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle());
     
        //Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Rotation2.fromDegrees(intialRobotOrientation + Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle().toDegrees()));

        //autonomousTrajectory.calculateSegments(5/1000);


        System.out.println("Autonomous Trajectory Command Initialized.");
        Vector2 position = new Vector2(0, 0);
        Robot.drivetrainSubsystem.resetKinematics(position, 0);


        Robot.drivetrainSubsystem.getFollower().follow(autonomousTrajectory);

        System.out.println(autonomousTrajectory.getDuration());
        

    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

       // double pathDuration = autonomousTrajectory.getDuration();
        //double lastTimeStamp =  Robot.drivetrainSubsystem.lastTimestamp;

        //double rotationAngle = 90;

        //double currentAngle = rotationAngle * (lastTimeStamp/pathDuration);

        // System.out.println("currentAngle: " + currentAngle);

        //Robot.drivetrainSubsystem.setRotation(currentAngle);
        //angleController.setSetpoint(currentAngle);
        
        // double rotation = angleController.calculate(0);
        // if(rotation > 1){
        //     rotation = 1;
        //   }else if(rotation < -1){
        //     rotation = -1;
        //   }
          
         //HolonomicMotionProfiledTrajectoryFollower follower = (HolonomicMotionProfiledTrajectoryFollower)Robot.drivetrainSubsystem.getFollower();
   
        
        // Segment lastSegment = follower.getLastSegment();
        // if (lastSegment != null) {
        //     System.out.println("heading: " + lastSegment.heading.toDegrees()
        //     + "\t rotation: " + lastSegment.rotation.toDegrees()
        //     + "\t translation: " + lastSegment.translation.toString());
        // }


        //Robot.drivetrainSubsystem.setTargetVelocity(velocity);
        //Robot.drivetrainSubsystem.updateKinematics(timestamp); 

    }

    // Called once after timeout
    protected void end() {
        Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0.0, true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
    
    @Override protected boolean isFinished() {       


        
        boolean isFinished = false;

        if(isTimedOut()){
            System.out.println("Timed Out");
            isFinished = true;
        }

    	return isFinished;
    }
}