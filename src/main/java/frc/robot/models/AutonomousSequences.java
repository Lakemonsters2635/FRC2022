package frc.robot.models;


// import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
// import org.frcteam2910.common.control.PathArcSegment;
// import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.CommandGroup;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AutonomousTrajectoryCommand;

// import frc.robot.commands.GalacticSearchCommand;
import frc.robot.commands.IntakeActuateCommand;
import frc.robot.commands.RobotRotateCommand;
import frc.robot.commands.ShooterCommand;

public class AutonomousSequences {
        public static double sampleDistance = 12.0;
        public static double startingVelocity = 1.0;
        public static double endingVelocity = 1.0;

        public static CommandGroup shootCollectRight() {
            CommandGroup output = new CommandGroup();

            IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);
            //Shoot the ball.
            //Turn around 180 degrees.
            // RobotRotateCommand rotateCommand1 = new RobotRotateCommand(-90);
            // RobotRotateCommand rotateCommand2 = new RobotRotateCommand(-180-26.57);
            RobotRotateCommand rotateCommand2 = new RobotRotateCommand(180+26.57);
            // RobotRotateCommand rotateCommand3 = new RobotRotateCommand(90);
            RobotRotateCommand rotateCommand4 = new RobotRotateCommand(180-15.32);

            

            //Drive backward 42 in.
            SimplePathBuilder driveBack = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            driveBack.lineTo(new Vector2(-66.733, 0.0));
            
            Path driveBackPath = driveBack.build();

            Trajectory driveBackTrajectory = new Trajectory(driveBackPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveBackCommand = new AutonomousTrajectoryCommand(driveBackTrajectory);
            //run intake
            
            //Turn around 180 degrees.

            //Shoot the ball.
            ShooterCommand shooterCommand = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
            ShooterCommand shooterCommand2 = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );

            output.addSequential(lowerIntake);
            output.addSequential(shooterCommand);
            // output.addSequential(rotateCommand1, 4);
            output.addSequential(rotateCommand2, 4);
            output.addSequential(driveBackCommand);
            // output.addSequential(rotateCommand3, 4);
            output.addSequential(rotateCommand4, 4);
            output.addSequential(shooterCommand2);

            
            return output;

        }

        public static CommandGroup shootCollectLeft() {
            CommandGroup output = new CommandGroup();

            IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);
            //Shoot the ball.
            //Turn around 180 degrees.
            // RobotRotateCommand rotateCommand1 = new RobotRotateCommand(-90);
            // RobotRotateCommand rotateCommand2 = new RobotRotateCommand(-180-26.57);
            RobotRotateCommand rotateCommand2 = new RobotRotateCommand(180-26.57);
            // RobotRotateCommand rotateCommand3 = new RobotRotateCommand(90);
            RobotRotateCommand rotateCommand4 = new RobotRotateCommand(180-15.32);

            

            //Drive backward 42 in.
            SimplePathBuilder alignWheels = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
            .lineTo(new Vector2(-1.0, 0.0));
            Path alignwheelsPath = alignWheels.build();

            Trajectory alignwheelsTrajectory = new Trajectory(alignwheelsPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand alignwheelsCommand = new AutonomousTrajectoryCommand(alignwheelsTrajectory);
            SimplePathBuilder driveBack = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            driveBack.lineTo(new Vector2(-66.733, 0.0));
            
            Path driveBackPath = driveBack.build();

            Trajectory driveBackTrajectory = new Trajectory(driveBackPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveBackCommand = new AutonomousTrajectoryCommand(driveBackTrajectory);

            //run intake
            
            //Turn around 180 degrees.

            //Shoot the ball.
            ShooterCommand shooterCommand = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
            ShooterCommand shooterCommand2 = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );

            output.addSequential(lowerIntake);
            output.addSequential(shooterCommand);
            // output.addSequential(rotateCommand1, 4);
            output.addSequential(rotateCommand2, 4);
            output.addSequential(alignwheelsCommand);
            output.addSequential(driveBackCommand);
            // // output.addSequential(rotateCommand3, 4);
            // output.addSequential(rotateCommand4, 4);
            // output.addSequential(shooterCommand2);

            
            return output;

        }

        public static CommandGroup shootCollectShootTwoCargo() {
            CommandGroup output = new CommandGroup();

            IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);


            RobotRotateCommand rotateCommand1 = new RobotRotateCommand(180+26.57);
            RobotRotateCommand rotateCommand2 = new RobotRotateCommand(180-15.32);
            RobotRotateCommand rotateCommand3 = new RobotRotateCommand(60.0); // this value and the one below it should be 67.5 and -67.5
            RobotRotateCommand rotateCommand4 = new RobotRotateCommand(-85.0);
            
            SimplePathBuilder alignWheels = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
            .lineTo(new Vector2(-1.0, 0.0))
            .lineTo(new Vector2(0.0, 0.0));
            Path alignwheelsPath = alignWheels.build();

            Trajectory alignwheelsTrajectory = new Trajectory(alignwheelsPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand alignwheelsCommand = new AutonomousTrajectoryCommand(alignwheelsTrajectory);


            SimplePathBuilder driveBack = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            driveBack.lineTo(new Vector2(-66.733, 0.0));
            Path driveBackPath = driveBack.build();

            Trajectory driveBackTrajectory = new Trajectory(driveBackPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveBackCommand = new AutonomousTrajectoryCommand(driveBackTrajectory);
            

            SimplePathBuilder driveToNextBall = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            driveToNextBall.lineTo(new Vector2(-117.101, 0.0));
            Path driveToNextBallPath = driveToNextBall.build();

            Trajectory driveToNextBallTrajectory = new Trajectory(driveToNextBallPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveToNextBallCommand = new AutonomousTrajectoryCommand(driveToNextBallTrajectory);


            ShooterCommand shooterCommand = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
            ShooterCommand shooterCommand2 = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
            ShooterCommand shooterCommand3 = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );


            output.addSequential(lowerIntake);
            output.addSequential(shooterCommand);
            output.addSequential(rotateCommand1, 4);

            output.addSequential(alignwheelsCommand);
            output.addSequential(driveBackCommand);

            output.addSequential(rotateCommand2, 4);
            output.addSequential(shooterCommand2);

            output.addSequential(rotateCommand3, 4);
            output.addSequential(driveToNextBallCommand);

            output.addSequential(rotateCommand4, 4);
            output.addSequential(shooterCommand3);


            return output;
        }

        public static CommandGroup passTarmacLine() {
            CommandGroup output = new CommandGroup();
            ShooterCommand shooterCommand = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );

            SimplePathBuilder driveOver = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
            .lineTo(new Vector2(-10.0, 0.0));

            Path driveOverPath = driveOver.build();
            
            Trajectory driveOverTrajectory = new Trajectory(driveOverPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveOverCommand = new AutonomousTrajectoryCommand(driveOverTrajectory);

            output.addSequential(shooterCommand);
            output.addSequential(driveOverCommand);
            return output;
        }
        public static CommandGroup rotate360() {
            CommandGroup output = new CommandGroup();

            RobotRotateCommand rotateCommand1 = new RobotRotateCommand(90);
            RobotRotateCommand rotateCommand2 = new RobotRotateCommand(90);
            // RobotRotateCommand rotateCommand3 = new RobotRotateCommand(90);
            // RobotRotateCommand rotateCommand4 = new RobotRotateCommand(90);

            output.addSequential(rotateCommand1);
            output.addSequential(rotateCommand2);
            // output.addSequential(rotateCommand3, 4);
    
            // output.addSequential(rotateCommand4, 4);


            return output;
        }

        public static CommandGroup shootCollectLeft1() {
            CommandGroup output = new CommandGroup();
            return output;
        }

        public static CommandGroup shootCollectLeft2() {
            CommandGroup output = new CommandGroup();
            return output;
        }
        public static CommandGroup dance2022Command() {
            CommandGroup output = new CommandGroup();
            SimplePathBuilder pathBuilder = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            pathBuilder.lineTo(new Vector2(-96.0, 0.0));
            pathBuilder.lineTo(new Vector2(-96.0, 96.0));
            pathBuilder.lineTo(new Vector2(0, 96.0));
            pathBuilder.lineTo(new Vector2(0, 0));
            Path path = pathBuilder.build();
            Trajectory driveTrajectory = new Trajectory(path, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveCommand1 = new AutonomousTrajectoryCommand(driveTrajectory);
            output.addSequential(driveCommand1);
            RobotRotateCommand rotateCommand1 = new RobotRotateCommand(-360-90);
            output.addSequential(rotateCommand1);
            return output;
        }    

        public static CommandGroup driveStraightThenBack() {
            CommandGroup output = new CommandGroup();
            SimplePathBuilder pathBuilder = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            pathBuilder.lineTo(new Vector2(-96.0, 0.0));
            
            SimplePathBuilder goBackwards = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            goBackwards.lineTo(new Vector2(96.0,0.0));
            
            Path path = pathBuilder.build();
            Path path2 = goBackwards.build();
         
            Trajectory driveTrajectory = new Trajectory(path, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveCommand1 = new AutonomousTrajectoryCommand(driveTrajectory);
            Trajectory driveTrajectory2 = new Trajectory(path2, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveCommand2 = new AutonomousTrajectoryCommand(driveTrajectory2);
                // System.out.println("command almost finished");
            output.addSequential(driveCommand1);
            output.addSequential(driveCommand2);

            return output;
        }

        public static CommandGroup arcTest() {
            CommandGroup output = new CommandGroup();

            SimplePathBuilder pathBuilder = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            pathBuilder.arcTo(new Vector2(-120.0, 0.0), new Vector2(-60.0, 0), Rotation2.fromDegrees(-5));

            Path path = pathBuilder.build();
                     
            Trajectory driveTrajectory = new Trajectory(path, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveCommand1 = new AutonomousTrajectoryCommand(driveTrajectory);

            output.addSequential(driveCommand1);

            return output;
        }


        public static CommandGroup straightLineRotationTest() {
            CommandGroup output = new CommandGroup();
            Trajectory driveTrajectory = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO)
                .lineTo(new Vector2(0.0, -66.733), Rotation2.fromDegrees(90.0)).build(),
                Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, 
                sampleDistance, startingVelocity, endingVelocity
            );
        
            
            // Path path = driveStraight.build();

            // Trajectory driveTrajectory = new Trajectory(path, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveCommand1 = new AutonomousTrajectoryCommand(driveTrajectory);
            output.addSequential(driveCommand1);
            return output;
        }

        public static CommandGroup shootCollectRightNoRotation() {
            CommandGroup output = new CommandGroup();
            
            // intake and shoot commands - not necessarily in order until output.add...
            IntakeActuateCommand aintake = new IntakeActuateCommand(false, 4); // raise intake
            ShooterCommand shooterCommand = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
            ShooterCommand shooterCommand2 = new ShooterCommand(false, 2, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );


            // drive straight portion
            SimplePathBuilder pathBuilder = new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO);
            pathBuilder.lineTo(new Vector2(54.243, -60.925)); 
            Path path = pathBuilder.build();

            Trajectory driveTrajectory = new Trajectory(path, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveCommand1 = new AutonomousTrajectoryCommand(driveTrajectory); 

            // arc portion
            SimplePathBuilder pathBuilder2 = new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO);
            pathBuilder.arcTo(new Vector2(18.902, 36.591), new Vector2(18.902/2, 36.591/2));

            Path path2 = pathBuilder.build();

            Trajectory driveTrajectory2 = new Trajectory(path2, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveCommand2 = new AutonomousTrajectoryCommand(driveTrajectory2); 

            output.addSequential(shooterCommand);
            output.addSequential(driveCommand1);
            output.addSequential(driveCommand2);
            output.addSequential(aintake);
            output.addSequential(shooterCommand2);
            return output;
        }
  


        public static String getMethodName() {
		    String methodName = Thread.currentThread().getStackTrace()[2].getMethodName();
	    	return methodName;
	    }
}