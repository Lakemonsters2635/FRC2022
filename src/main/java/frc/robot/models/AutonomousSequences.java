package frc.robot.models;

import java.sql.DriverManager;

// import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
// import org.frcteam2910.common.control.PathArcSegment;
// import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AutonomousTrajectoryCommand;
import frc.robot.commands.DoNothingCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FetchCollectPowerCellCommand;
import frc.robot.commands.FetchPowerCellCommand;
import frc.robot.commands.FetchPowerCellPidTestCommand;
// import frc.robot.commands.GalacticSearchCommand;
import frc.robot.commands.IntakeActuateCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDetectToElevatorIndexCommand;
import frc.robot.commands.RobotRotateCommand;
import frc.robot.commands.ShooterActuateCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.VisionRotationDriveCommand;

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
            output.addParallel(new IntakeDetectToElevatorIndexCommand(6));
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
            output.addParallel(new IntakeDetectToElevatorIndexCommand(6));
            output.addSequential(driveBackCommand);
            // // output.addSequential(rotateCommand3, 4);
            // output.addSequential(rotateCommand4, 4);
            // output.addSequential(shooterCommand2);

            
            return output;

        }

        public static CommandGroup twoBallAutoMidStart() {
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
            output.addParallel(new IntakeDetectToElevatorIndexCommand(6));
            output.addSequential(driveBackCommand);

            output.addSequential(rotateCommand2, 4);
            output.addSequential(shooterCommand2);

            output.addSequential(rotateCommand3, 4);
            output.addParallel(new IntakeDetectToElevatorIndexCommand(6));
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

        public static CommandGroup new2022Command() {
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





}
        // public static CommandGroup FetchPowerCellPidTest() {
        //         CommandGroup output = new CommandGroup(); 
        //         FetchPowerCellPidTestCommand tunePIDCommand = new FetchPowerCellPidTestCommand(); 
        //         output.addSequential(tunePIDCommand); 
        //         return output; 
        // }

//         public static CommandGroup FetchCollectPowerCell() {
//                 CommandGroup output = new CommandGroup(); 
//                 FetchCollectPowerCellCommand command = new FetchCollectPowerCellCommand(); 
//                 output.addSequential(command); 
//                 return output; 
//         }

//         public static CommandGroup GoFetchTest() {
//                 CommandGroup output = new CommandGroup();
//                 FetchPowerCellCommand fpcCommand = new FetchPowerCellCommand();
//                 FetchPowerCellCommand fpcCommand1 = new FetchPowerCellCommand();
//                 FetchPowerCellCommand fpcCommand2 = new FetchPowerCellCommand();
//                 DoNothingCommand doNothing = new DoNothingCommand(5);

//                 RobotRotateCommand rotateCommand = new RobotRotateCommand(-60);
//                 RobotRotateCommand rotateCommand1 = new RobotRotateCommand(100);


//                 Path rotatePath = new Path(Rotation2.ZERO);
//                 rotatePath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0), 
//                                 new Vector2(-3.0, 0)
//                                 ),
//                         Rotation2.fromDegrees(-72)
//                 );
//                 rotatePath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0), 
//                                 new Vector2(-3.0, 0)
//                                 ),
//                         Rotation2.fromDegrees(-72)
//                 );
//                 // rotatePath.addSegment(
//                 //         new PathLineSegment(
//                 //                 new Vector2(0.0, 0.0), 
//                 //                 new Vector2(-1.0, 0.0)
//                 //         )
//                 // );
//                 rotatePath.subdivide(8);
//                 Trajectory rotatePathTrajectory = new Trajectory(rotatePath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand rotatePathCommand = new AutonomousTrajectoryCommand(rotatePathTrajectory);

//                 Path rotatePath2 = new Path(Rotation2.ZERO);
//                 rotatePath2.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0), 
//                                 new Vector2(-3.0, 0)
//                                 ),
//                         Rotation2.fromDegrees(120)
//                 );
//                 rotatePath2.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0), 
//                                 new Vector2(-3.0, 0)
//                                 ),
//                         Rotation2.fromDegrees(120)
//                 );
//                 // rotatePath2.addSegment(
//                 //         new PathLineSegment(
//                 //                 new Vector2(0.0, 0.0), 
//                 //                 new Vector2(-1.0, 0.0)
//                 //                 )
//                 // );

//                 rotatePath2.subdivide(8);
//                 Trajectory rotatePath2Trajectory = new Trajectory(rotatePath2, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand rotatePath2Command = new AutonomousTrajectoryCommand(rotatePath2Trajectory);


//                 Path goToEnd = new Path(Rotation2.ZERO);
//                 goToEnd.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0, 0), 
//                                 new Vector2(-100, -80)
//                                 )
//                 );

//                 Trajectory goToEndTrajectory = new Trajectory(goToEnd, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand goToEndCommand = new AutonomousTrajectoryCommand(goToEndTrajectory);

                
//                 output.addParallel(fpcCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(8));
//                 // output.addSequential(rotateCommand);
//                 output.addSequential(rotatePathCommand);
//                 output.addParallel(fpcCommand1);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(20));
//                 output.addSequential(rotatePath2Command);
//                 // // output.addSequential(doNothing);
//                 output.addParallel(fpcCommand2);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(32));
//                 output.addSequential(goToEndCommand);
                

                
//                 return output;
//         }

//         public static CommandGroup PathArcTest() {
//                 double radius = 50;
//                 CommandGroup output = new CommandGroup();
//                 Path arcPath = new Path(Rotation2.ZERO);
//                 arcPath.addSegment(
//                         new PathArcSegment(
//                                 new Vector2(0,0), 
//                                 new Vector2(radius+radius*Math.sqrt(3)/2, radius/2), 
//                                 new Vector2(radius,0)
//                                 )
//                 );
//                 arcPath.addSegment(
//                         new PathArcSegment(
//                                 new Vector2(radius+radius*Math.sqrt(3)/2, radius/2), 
//                                 new Vector2(radius, -radius), 
//                                 new Vector2(radius,0)
//                                 )
//                 );
//                 arcPath.addSegment(
//                         new PathArcSegment(
//                                 new Vector2(radius, -radius), 
//                                 new Vector2(0,0), 
//                                 new Vector2(radius,0)
//                                 )
//                 );
//                 Trajectory arcTrajectory = new Trajectory(arcPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand arcCommand = new AutonomousTrajectoryCommand(arcTrajectory);

//                 output.addSequential(arcCommand);
//                 return output; 
//         }
//         public static CommandGroup PathLineTest() {
                
//                 CommandGroup output = new CommandGroup();
//                 Path arcPath = new Path(Rotation2.ZERO);
//                 arcPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0,0), 
//                                 new Vector2(0, 25)
//                                 )
//                 );
//                 arcPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0, 25), 
//                                 new Vector2(0, -50) 
//                                 )
//                 );
//                 arcPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0, -50), 
//                                 new Vector2(0, 25) 
//                                 )
//                 );
//                 Trajectory arcTrajectory = new Trajectory(arcPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand arcCommand = new AutonomousTrajectoryCommand(arcTrajectory);

//                 output.addSequential(arcCommand);
//                 return output; 
//         }
        
//         public static CommandGroup DriveLeftThenRight() {
//                 CommandGroup output = new CommandGroup();
//                 Path driveLeft = new Path(Rotation2.ZERO);
//                 driveLeft.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2( 0.0, -90)
//                         )
//                 );
                
//                 Trajectory driveLeftTrajectory = new Trajectory(driveLeft, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveLeftCommand = new AutonomousTrajectoryCommand(driveLeftTrajectory);

//                 Path driveRight = new Path(Rotation2.ZERO);
//                 driveRight.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2( 0.0, 90)
//                         )
//                 );
                
//                 Trajectory driveRightTrajectory = new Trajectory(driveRight, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveRightCommand = new AutonomousTrajectoryCommand(driveRightTrajectory);


//                 output.addSequential(driveLeftCommand);
//                 output.addSequential(driveRightCommand);
//                 return output;
//         }

//         public static CommandGroup DriveStraightForwardAndBack() {


//               // Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Rotation2.fromDegrees(90 + Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle().toDegrees()));

//                 CommandGroup output = new CommandGroup();

//                 Path driveForward = new Path(Rotation2.ZERO);
//                 driveForward.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-180, 0.0)
//                         )
//                 );
                
//                 Trajectory driveforwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveforwardTrajectory);

//                 Path driveBackward = new Path(Rotation2.ZERO);
//                 driveBackward.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(180, 0.0)
//                         )
//                 );
                
//                 Trajectory drivebackwardTrajectory = new Trajectory(driveBackward, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveBackwardCommand = new AutonomousTrajectoryCommand(drivebackwardTrajectory);

//                 output.addSequential(driveForwardCommand);
//                 output.addSequential(driveBackwardCommand);

//                 return output;
//         }


//         public static CommandGroup RotateTest(){
//                 CommandGroup output = new CommandGroup();

//                 RobotRotateCommand rotateCommand = new RobotRotateCommand(90);
//                 RobotRotateCommand rotateCommand2 = new RobotRotateCommand(90);
//                 RobotRotateCommand rotateCommand3 = new RobotRotateCommand(90);
//                 RobotRotateCommand rotateCommand4 = new RobotRotateCommand(90);

//                 Path driveForward1 = new Path(Rotation2.ZERO);
//                 driveForward1.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-3, 0.0)
//                         ),
//                         Rotation2.fromDegrees(-170)
//                 );

//                 driveForward1.subdivide(8);                
//                 Trajectory driveForwardTrajectory1 = new Trajectory(driveForward1, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand1 = new AutonomousTrajectoryCommand(driveForwardTrajectory1);

//                 Path driveForward2 = new Path(Rotation2.ZERO);
//                 driveForward2.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-30, 0.0)
//                         )
//                 );
                
//                 Trajectory driveForwardTrajectory2 = new Trajectory(driveForward2, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand2 = new AutonomousTrajectoryCommand(driveForwardTrajectory2);

//                 Path driveForward3 = new Path(Rotation2.ZERO);
//                 driveForward3.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-30, 0.0)
//                         )
//                 );
                
//                 Trajectory driveForwardTrajectory3 = new Trajectory(driveForward3, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand3 = new AutonomousTrajectoryCommand(driveForwardTrajectory3);

//                 Path driveForward4 = new Path(Rotation2.ZERO);
//                 driveForward4.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-0.1, 0.0)
//                         )
//                 );
            
//                 Trajectory driveForwardTrajectory4 = new Trajectory(driveForward4, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand4 = new AutonomousTrajectoryCommand(driveForwardTrajectory4);



//                 // output.addSequential(rotateCommand, 2);
//                 output.addSequential(driveForwardCommand1);
//                 // output.addSequential(rotateCommand2, 2);
//                 //output.addSequential(driveForwardCommand4);
//                 // output.addSequential(rotateCommand3, 2);
//                 //output.addSequential(driveForwardCommand4);
//                 // output.addSequential(rotateCommand4, 2);
//                 //output.addSequential(driveForwardCommand4);
//                 return output;
//                 //-----------------------------------------------------------
                
//                 // RobotRotateCommand rotateCommand = new RobotRotateCommand(90);
//                 // output.addSequential(rotateCommand);
//                 // Path driveToTrenchPath = new Path(Rotation2.ZERO);
//                 // driveToTrenchPath.addSegment(
//                 //         new PathLineSegment(
//                 //                 new Vector2(0.0, 0.0),
//                 //                 new Vector2(0.0, 0.0)
//                 //         ),
//                 //         Rotation2.fromDegrees(-90)
//                 //         // negative angle = clockwise when viewed from Frank's safe command station
//                 //         // positive angle = counterclockwise when viewed from Frank's safe command station       
//                 // );

                
//                 // PathArcSegment fooFi = PathArcSegment.fromPoints(new Vector2(0.0, 0.0), new Vector2(70.0, -20.0), new Vector2(0, -60.0));
//                 // Path arcFooFi = new Path(Rotation2.ZERO);
//                 // arcFooFi.addSegment(fooFi);

//                 // // Trajectory driveToTrenchTrajectory = new Trajectory(driveToTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 // // AutonomousTrajectoryCommand driveToTrenchCommand = new AutonomousTrajectoryCommand(driveToTrenchTrajectory);

//                 // Trajectory fooFiTrajectory = new Trajectory(arcFooFi, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 // AutonomousTrajectoryCommand fooFiCommand = new AutonomousTrajectoryCommand(fooFiTrajectory);

                
//                 // output.addSequential(fooFiCommand);
        
//                 //return output;
//         }

//         public static CommandGroup DriveTwoFeetTwice() {
//                 CommandGroup output = new CommandGroup();
//                 RobotRotateCommand rotateCommand = new RobotRotateCommand(-90);
//                 Path driveForward = new Path(Rotation2.ZERO);
//                 driveForward.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-48, 0.0)
//                         )
//                 );
                
//                 Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand1 = new AutonomousTrajectoryCommand(driveForwardTrajectory);

//                 Path driveForward2 = new Path(Rotation2.ZERO);
//                 driveForward2.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-48, 0.0)
//                         )
//                 );
                
//                 Trajectory driveForwardTrajectory2 = new Trajectory(driveForward2, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand2 = new AutonomousTrajectoryCommand(driveForwardTrajectory2);

//                 output.addSequential(driveForwardCommand1);
//                 output.addSequential(rotateCommand, 2);
//                 output.addSequential(driveForwardCommand2);

//                 return output;
//         }


//         public static CommandGroup GalacticSearch(){
//                 CommandGroup output = new CommandGroup();
//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);
//                 output.addSequential(lowerIntake);

//                 GalacticSearchCommand searchCommand = new GalacticSearchCommand(20);
//                 output.addSequential(searchCommand);
//                 return output;
//         }
        
//         public static CommandGroup GalacticSearchRedPathA() {
//                 CommandGroup output = new CommandGroup();
//                 RobotRotateCommand rotateCommand = new RobotRotateCommand(26.8); // before 23.56, which is too far over
//                 RobotRotateCommand rotateCommand2 = new RobotRotateCommand(-78.988-10); // before -85.12 when rotateCommand angle was 23.56
//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);

//                 Path driveForward = new Path(Rotation2.ZERO);
//                 driveForward.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-29.0, 0.0)
//                         )
//                 );
                
//                 Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);

                
//                 Path driveToD5 = new Path(Rotation2.ZERO);
//                 driveToD5.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-78, 0.0)
//                         )
//                 );
                
//                 Trajectory driveToD5Trajectory = new Trajectory(driveToD5, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToD5Command = new AutonomousTrajectoryCommand(driveToD5Trajectory);


//                 Path driveToA6 = new Path(Rotation2.ZERO);
//                 driveToA6.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-105, 0.0)
//                         )
//                 );
                
//                 Trajectory driveToA6Trajectory = new Trajectory(driveToA6, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToA6Command = new AutonomousTrajectoryCommand(driveToA6Trajectory);

//                 Path driveToEnd = new Path(Rotation2.ZERO);
//                 driveToEnd.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-60, 140)
//                         )
//                 );
                
//                 Trajectory driveToEndTrajectory = new Trajectory(driveToEnd, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToEndCommand = new AutonomousTrajectoryCommand(driveToEndTrajectory);

//                 output.addSequential(new TimedCommand(1));
//                 output.addParallel(driveForwardCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(4));
//                 output.addSequential(rotateCommand, 1);
//                 output.addParallel(driveToD5Command);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
//                 output.addSequential(rotateCommand2, 1);
//                 output.addParallel(driveToA6Command);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(8));
//                 output.addSequential(driveToEndCommand);

//                 return output; 
//         }

//         public static CommandGroup GalacticSearchBluePathA() {
//                 CommandGroup output = new CommandGroup();
//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
//                 RobotRotateCommand rotateCommand0 = new RobotRotateCommand(-71.56+12); 
//                 RobotRotateCommand rotateCommand1 = new RobotRotateCommand(97.12-12); 
                
//                 Path driveForward = new Path(Rotation2.ZERO);
//                 driveForward.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-123.0, 0.0)
//                         )
//                 );
//                 Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);


//                 Path driveToB7 = new Path(Rotation2.ZERO);
//                 driveToB7.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-94.86, 0.0)
//                         )
//                 );
                
//                 Trajectory driveToB7Trajectory = new Trajectory(driveToB7, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToB7Command = new AutonomousTrajectoryCommand(driveToB7Trajectory);

//                 Path driveToC9 = new Path(Rotation2.ZERO);
//                 driveToC9.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-120.0,0.0)
//                         )
//                 );
                
//                 Trajectory driveToC9Trajectory = new Trajectory(driveToC9, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToC9Command = new AutonomousTrajectoryCommand(driveToC9Trajectory);

//                 // Path driveToEndzone = new Path(Rotation2.ZERO);
//                 // driveToEndzone.addSegment(
//                 //         new PathLineSegment(
//                 //                 new Vector2(0.0,0.0), 
//                 //                 new Vector2(-67.08,0.0)
//                 //         )
//                 // );
                
//                 // Trajectory driveToEndzoneTrajectory = new Trajectory(driveToEndzone, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
//                 // AutonomousTrajectoryCommand driveToEndzoneCommand = new AutonomousTrajectoryCommand(driveToEndzoneTrajectory);

//                 output.addSequential(new TimedCommand(0.5));
//                 output.addParallel(driveForwardCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5)); //is 3 seconds enough?
//                 output.addSequential(rotateCommand0);
//                 output.addParallel(driveToB7Command);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5)); //is 3 seconds enough?
//                 output.addSequential(rotateCommand1);
//                 output.addParallel(driveToC9Command);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
//                 return output; 

//                 // Why dont we make a function that takes the params for movement and such then returns the AutonomousTrajectoryCommand to make the code smaller, just an idea
//         }

//         public static CommandGroup GalacticSearchRedPathB() {

//                 //https://firstfrc.blob.core.windows.net/frc2021/Manual/TeamUpdates/2021TeamUpdate02.pdf
//                 CommandGroup output = new CommandGroup();
//                 RobotRotateCommand rotateCommand = new RobotRotateCommand(45.0); // before 23.56, which is too far over
//                 RobotRotateCommand rotateCommand2 = new RobotRotateCommand(-45.0); // before -85.12 when rotateCommand angle was 23.
//                 IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true, 1);
//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);

//                 Path driveForward = new Path(Rotation2.ZERO);
//                 driveForward.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-32.0, 0.0)
//                         )
//                 );
                
//                 Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);

//                 Path driveArc = new Path(Rotation2.ZERO);
//                 driveArc.addSegment(
//                         new PathArcSegment(
//                                 new Vector2(0.0, 0.0), 
//                                 new Vector2(-55, 200), 
//                                 new Vector2(-55, 10)
//                         )
//                 );
                

//                 Trajectory driveArcTrajectory = new Trajectory(driveArc, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveArcCommand = new AutonomousTrajectoryCommand(driveArcTrajectory);
                
//                 Path driveArc2 = new Path(Rotation2.ZERO);
//                 driveArc2.addSegment(
//                         new PathArcSegment(
//                                 new Vector2(0.0, 0.0), 
//                                 new Vector2(-70.0, -65.0), 
//                                 new Vector2(-70.0, 5)
//                         )
//                 );
//                 Trajectory driveArc2Trajectory = new Trajectory(driveArc2, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveArc2Command = new AutonomousTrajectoryCommand(driveArc2Trajectory);

//                 Path driveToEnd = new Path(Rotation2.ZERO);
//                 driveToEnd.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-140.0, 0.0)
//                         )
//                 );
                
//                 Trajectory driveToEndTrajectory = new Trajectory(driveToEnd, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToEndCommand = new AutonomousTrajectoryCommand(driveToEndTrajectory);

                

//                 output.addSequential(new TimedCommand(0.5));
//                 output.addParallel(driveForwardCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
//                 output.addParallel(driveArcCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
//                 output.addParallel(driveArc2Command);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(4));
//                 output.addParallel(driveToEndCommand);
//                 output.addSequential(raiseIntake);

//                 return output; 
//         }

//         public static CommandGroup GalacticSearchBluePathB() {
//                 CommandGroup output = new CommandGroup();
//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
//                 IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true, 1);

                
//                 Path driveForward = new Path(Rotation2.ZERO);
//                 driveForward.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-123.0, 0.0)
//                         )
//                 );
//                 Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);

//                 Path driveArc = new Path(Rotation2.ZERO);
//                 driveArc.addSegment(
//                         new PathArcSegment(
//                                 new Vector2(0.0, 0.0), 
//                                 new Vector2(-70.0, -65.0), 
//                                 new Vector2(-70.0, 5)
//                         )
//                 );
                

//                 Trajectory driveArcTrajectory = new Trajectory(driveArc, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveArcCommand = new AutonomousTrajectoryCommand(driveArcTrajectory);
                
//                 Path driveArc2 = new Path(Rotation2.ZERO);
//                 driveArc2.addSegment(
//                         new PathArcSegment(
//                                 new Vector2(0.0, 0.0), 
//                                 new Vector2(-55, 200), 
//                                 new Vector2(-55, 10)
//                         )
//                 );
//                 driveArc2.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(-55,200),
//                                 new Vector2(-65,200)
//                         )
//                 );
                
//                 Trajectory driveArc2Trajectory = new Trajectory(driveArc2, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveArc2Command = new AutonomousTrajectoryCommand(driveArc2Trajectory);

//                 Path driveToEnd = new Path(Rotation2.ZERO);
//                 driveToEnd.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-35.0, 0.0)
//                         )
//                 );
                
//                 Trajectory driveToEndTrajectory = new Trajectory(driveToEnd, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToEndCommand = new AutonomousTrajectoryCommand(driveToEndTrajectory);

                
//                 output.addParallel(driveForwardCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5)); //is 3 seconds enough?
//                 output.addParallel(driveArcCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
//                 output.addParallel(driveArc2Command);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(6));
//                 output.addParallel(driveToEndCommand);
//                 output.addSequential(raiseIntake);
//                 return output; 

//                 // Why dont we make a function that takes the params for movement and such then returns the AutonomousTrajectoryCommand to make the code smaller, just an idea
//         }

//         public static CommandGroup GalacticSearchRedPathARotate() {
//                 CommandGroup output = new CommandGroup();
//                 double firstRotate = 20.8;
//                 double secondRotate = -78.988;
//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);
//                 Rotation2 firstRotation = new Rotation2(Math.cos(firstRotate*Math.PI/180), Math.sin(firstRotate*Math.PI/180), true);
//                 Rotation2 secondRotation = new  Rotation2(Math.cos(secondRotate*Math.PI/180), Math.sin(secondRotate*Math.PI/180), true);

//                 Path driveForward = new Path(Rotation2.ZERO);
//                 driveForward.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-29.0, 0.0)
//                         )
//                 );
                
//                 Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);

                
//                 Path driveToD5 = new Path(Rotation2.ZERO);
//                 driveToD5.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-78, 0.0)
//                         ), firstRotation
//                 ); // can add another segment to same path
                
//                 Trajectory driveToD5Trajectory = new Trajectory(driveToD5, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToD5Command = new AutonomousTrajectoryCommand(driveToD5Trajectory);


//                 Path driveToA6 = new Path(Rotation2.ZERO);
//                 driveToA6.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0,0.0), 
//                                 new Vector2(-105, 0.0)
//                         ), secondRotation
//                 );
                
//                 Trajectory driveToA6Trajectory = new Trajectory(driveToA6, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand driveToA6Command = new AutonomousTrajectoryCommand(driveToA6Trajectory);

//                 output.addParallel(lowerIntake);
//                 output.addParallel(driveForwardCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(4));
//                 // output.addSequential(rotateCommand, 2);
//                 output.addParallel(driveToD5Command);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
//                 // output.addSequential(rotateCommand2, 2);
//                 output.addParallel(driveToA6Command);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand(8));

//                 return output; 
//         }

        
//         public CommandGroup buildDriveAndCollect(Vector2 startPose, Vector2 endPose)
//         {
//                 CommandGroup driveAndCollect = new CommandGroup();


//                 return driveAndCollect;
//         }


//         public static CommandGroup IntakeTest(){
//                 CommandGroup output = new CommandGroup();
//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
//                 IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,1);
//                 Path driveForwardPath = new Path(Rotation2.ZERO);
//                 driveForwardPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-106, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveForwardTrajectory = new Trajectory(driveForwardPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);

           
//                 output.addSequential(lowerIntake);
//                 output.addParallel(driveForwardCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(raiseIntake);

//                 return output;
//         }

// 	public static CommandGroup ShootThenCollectRight(){
//                 CommandGroup output = new CommandGroup();
//                 ShooterActuateCommand shooterAcuateCommand = new ShooterActuateCommand(true, 1);
//                 ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
//                 ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
//                 Path driveToTrenchPath = new Path(Rotation2.ZERO);
//                 driveToTrenchPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-44.63, -67.905) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveToTrenchTrajectory = new Trajectory(driveToTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveToTrenchCommand = new AutonomousTrajectoryCommand(driveToTrenchTrajectory);

//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
//                 IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,1);

                
//                 output.addSequential(shooterAcuateCommand);
//                 output.addParallel(elevatorCommand);
//                 output.addSequential(shooterCommand);
//                 output.addParallel(lowerIntake);
//                 output.addSequential(driveToTrenchCommand);
                
                
//                 //We've reached the trench. Now collect power cell. 
//                 Path driveThroughTrenchPath = new Path(Rotation2.ZERO);
//                 driveThroughTrenchPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-106, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveThroughTrenchTrajectory = new Trajectory(driveThroughTrenchPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveThroughTrenchCommand = new AutonomousTrajectoryCommand(driveThroughTrenchTrajectory);

           
        
//                 output.addParallel(driveThroughTrenchCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(raiseIntake);
//                 return output;
//         }

// 	public static CommandGroup ShootThenCollectLeft(){
//                 CommandGroup output = new CommandGroup();
//                 ShooterActuateCommand shooterAcuateCommand = new ShooterActuateCommand(true, 1);
//                 ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
//                 ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
//                 Path driveToLeftTrenchPath = new Path(Rotation2.ZERO);
//                 driveToLeftTrenchPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-75, 191.8) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveToLeftTrenchTrajectory = new Trajectory(driveToLeftTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveToTrenchCommand = new AutonomousTrajectoryCommand(driveToLeftTrenchTrajectory);

//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
//                 IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,1);

                
//                 output.addSequential(shooterAcuateCommand);
//                 output.addParallel(elevatorCommand);
//                 output.addSequential(shooterCommand);
//                 output.addParallel(lowerIntake);
//                 output.addSequential(driveToTrenchCommand);
                
//                 Path driveToBallPath = new Path(Rotation2.ZERO);
//                 driveToBallPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-20, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveToBallTrajectory = new Trajectory(driveToBallPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveToBallCommand = new AutonomousTrajectoryCommand(driveToBallTrajectory);

                
//                 output.addParallel(driveToBallCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
                
//                 Path driveToNextBallPath = new Path(Rotation2.ZERO);
//                 driveToNextBallPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(20, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );
//                 driveToNextBallPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(0.0, 18.3) //FHE:TODO Confirm positive/negative
//                         )
//                 );
//                 driveToNextBallPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-20, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveToNextBallTrajectory = new Trajectory(driveToNextBallPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveToNextBallCommand = new AutonomousTrajectoryCommand(driveToNextBallTrajectory);

//                 output.addParallel(driveToNextBallCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());

//                 output.addSequential(raiseIntake);
//                 return output;
//         }


//         public static CommandGroup ShootThenCollectRight_ThenShootAgain(){
//                 CommandGroup output =  ShootThenCollectRight();
//                 Path driveBackToShoot = new Path(Rotation2.ZERO);


//                 driveBackToShoot.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(106, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveBackToShootTrajectory = new Trajectory(driveBackToShoot, Robot.drivetrainSubsystem.CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveBackToShootCommand= new AutonomousTrajectoryCommand(driveBackToShootTrajectory);
//                 output.addSequential(driveBackToShootCommand,3);
//                 VisionRotationDriveCommand rotateCommand = new VisionRotationDriveCommand(2);
             

//                 ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
//                 ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
//                 output.addSequential(rotateCommand);
//                 output.addParallel(elevatorCommand);
//                 output.addSequential(shooterCommand);;
//                 return output;
//         }

//         public static CommandGroup ShootFromRight_Of_Optimal_Then_Collect(){
//                 CommandGroup output =  new CommandGroup();
//                 IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,2);
//                 VisionRotationDriveCommand visionRotateCommand = new VisionRotationDriveCommand(2);
//                 RobotRotateCommand rotateCommand = new RobotRotateCommand(0);
//                 ShooterActuateCommand shooterAcuateCommand = new ShooterActuateCommand(true, 1);
//                 ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
//                 IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 2);
//                 ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );

//                 output.addParallel(raiseIntake);
//                 output.addSequential(visionRotateCommand);
                
     
//                 output.addSequential(shooterAcuateCommand);
//                 output.addParallel(elevatorCommand);
//                 output.addSequential(shooterCommand);

//                 //output.addSequential(rotateCommand);
               
//                 Path driveToTrenchPath = new Path(Rotation2.ZERO);
//                 driveToTrenchPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-44.63, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveToTrenchTrajectory = new Trajectory(driveToTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveToTrenchCommand= new AutonomousTrajectoryCommand(driveToTrenchTrajectory);
//                 output.addSequential(driveToTrenchCommand);


//                 output.addParallel(lowerIntake);

//                 Path driveThroughTrenchPath = new Path(Rotation2.ZERO);
//                 driveThroughTrenchPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-106, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveThroughTrenchTrajectory = new Trajectory(driveThroughTrenchPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveThroughTrenchCommand = new AutonomousTrajectoryCommand(driveThroughTrenchTrajectory);

           
        
//                 output.addParallel(driveThroughTrenchCommand);
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(new IntakeDetectToElevatorIndexCommand());
//                 output.addSequential(new IntakeActuateCommand(true,2));

//                 return output;

//         }

//         public static CommandGroup shootThenBackAwayFromInitiationLine(){
//                 CommandGroup output =  new CommandGroup();
//                 IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,2);
//                 VisionRotationDriveCommand visionRotateCommand = new VisionRotationDriveCommand(2);
//                 RobotRotateCommand rotateCommand = new RobotRotateCommand(0);
//                 ShooterActuateCommand shooterAcuateCommand = new ShooterActuateCommand(true, 1);
//                 ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
//                 ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );

//                 output.addParallel(raiseIntake);
//                 output.addSequential(visionRotateCommand);
                
     
//                 output.addSequential(shooterAcuateCommand);
//                 output.addParallel(elevatorCommand);
//                 output.addSequential(shooterCommand);

//                 //output.addSequential(rotateCommand);
               
//                 Path driveAwayPath = new Path(Rotation2.ZERO);
//                 driveAwayPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-20, 0.0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory driveAwayTrajectory = new Trajectory(driveAwayPath, Robot.drivetrainSubsystem.CONSTRAINTS);

//                 AutonomousTrajectoryCommand driveAwayCommand= new AutonomousTrajectoryCommand(driveAwayTrajectory);
//                 output.addSequential(driveAwayCommand);

//                 return output;
//         }


// 	public static CommandGroup backAwayFromInitiationLine(){
//                 CommandGroup output = new CommandGroup();
//                 Path backAwayPath = new Path(Rotation2.ZERO);
//                 backAwayPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0.0, 0.0),
//                                 new Vector2(-48, 0) //FHE:TODO Confirm positive/negative
//                         )
//                 );


//                 Trajectory backawayTrajectory = new Trajectory(backAwayPath, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand backAwayCommand = new AutonomousTrajectoryCommand(backawayTrajectory);
//                 output.addSequential(backAwayCommand, 2);

//                 return output;

//         }

//         public static CommandGroup barrelRacing() {
//                 CommandGroup output = new CommandGroup();
//                 Path barrelPath = new Path(Rotation2.ZERO);
//                 // TODO UPDATE 3/22 use following addSegment to test AutoNavMath functions
//                 PathLineSegment segment = AutoNavMath.circlePointTangent(new Vector2(-80, -28), new Vector2(0,0), true, false, 28);
//                 System.out.println("Start: " + segment.getStart() + " End: " + segment.getEnd());
//                 barrelPath.addSegment( // start zone to B5 
//                         segment
//                 );

//                 // barrelPath.addSegment( // loop around D5
//                 //         new PathArcSegment(
//                 //                 new Vector2(0, 0), // start point
//                 //                 new Vector2(0, 0), // end point
//                 //                 new Vector2(0, 0) // center point
//                 //         )
//                 // );
//                 // barrelPath.addSegment( // straight line to B8
//                 //         AutoNavMath.-("D5", "B8", true) 
//                 // );
//                 // barrelPath.addSegment( // loop around B8
//                 //         new PathArcSegment(
//                 //                 new Vector2(0, 0), // start point
//                 //                 new Vector2(0, 0), // end point
//                 //                 new Vector2(0, 0) // center point
//                 //         )
//                 // );
//                 // barrelPath.addSegment( // straight line to D10
//                 //         AutoNavMath.circleCircleExternalTangent("B8", "D10", false)
//                 // );
//                 // barrelPath.addSegment( // loop around D10
//                 //         new PathArcSegment(
//                 //                 new Vector2(0, 0), // start point
//                 //                 new Vector2(0, 0), // end point
//                 //                 new Vector2(0, 0) // center point
//                 //         )
//                 // );
//                 // barrelPath.addSegment( // return to start zone
//                 //         AutoNavMath.circlePointTangent("D10", "C1", true, true)
//                 // );

//                 Trajectory barrelTrajectory = new Trajectory(barrelPath, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand barrelCommand = new AutonomousTrajectoryCommand(barrelTrajectory);
//                 output.addSequential(barrelCommand); 

//                 return output; 
//         }

//         public static CommandGroup barrelRacing2() {
//                 CommandGroup output = new CommandGroup();
//                 Path barrelPath = new Path(Rotation2.ZERO);
//                 // barrelPath.addSegment( // start zone to B5
//                 //          new PathLineSegment(
//                 //                  new Vector2(0,0),
//                 //                  new Vector2(-90,0)
//                 //          )
//                 //  );
//                 barrelPath.addSegment( // start zone to B5
//                         new PathLineSegment(
//                                 new Vector2(0,0),
//                                 new Vector2(-138.448,0)
//                         )
//                 );
//                 barrelPath.addSegment( // loop around D5
//                         new PathArcSegment(
//                                 new Vector2(-138.448, 0), // start point
//                                 new Vector2(-140, 58.665), // end point
//                                 new Vector2(-114, 30) // center point
//                         )
//                 );
//                 barrelPath.addSegment( // loop around D5
//                         new PathArcSegment(
//                                 new Vector2(-140, 58.665), // start point
//                                 new Vector2(-90, 60.359), // end point
//                                 new Vector2(-114, 30) // center point
//                         )
//                 );
//                 barrelPath.addSegment( // loop around D5
//                         new PathArcSegment(
//                                 new Vector2(-90, 60.359), // start point
//                                 new Vector2(-89.552, 0), // end point
//                                 new Vector2(-114, 30) // center point
//                         )
//                 );
//                 barrelPath.addSegment( 
//                         new PathLineSegment(
//                                 new Vector2(-89.552,0),
//                                 new Vector2(-228.448,0)
//                         )
//                 );
//                 barrelPath.addSegment( 
//                         new PathArcSegment(
//                                 new Vector2(-228.448,0), // start point
//                                 new Vector2(-204, -68.7), // end point
//                                 new Vector2(-204, -30) // center point
//                         )
//                 );
//                 barrelPath.addSegment( 
//                         new PathArcSegment(
//                                 new Vector2(-204, -68.7), // start point
//                                 new Vector2(-179.552, 0), // end point
//                                 new Vector2(-204, -30) // center point
//                         )
//                 );
//                 barrelPath.addSegment( 
//                         new PathLineSegment(
//                                 new Vector2(-179.552, 0),
//                                 new Vector2(-245,63.664)
//                         )
//                 );
//                 barrelPath.addSegment( 
//                         new PathArcSegment(
//                                 new Vector2(-245,63.664), // start point
//                                 new Vector2(-313,52.329), // end point
//                                 new Vector2(-276, 40) // center point
//                         )
//                 );
//                 barrelPath.addSegment( 
//                         new PathArcSegment(
//                                 new Vector2(-313,52.329), // start point
//                                 new Vector2(-300.92, 10), // end point
//                                 new Vector2(-276, 40) // center point
//                         )
//                 );

//                 // Path barrelPath2 = new Path(Rotation2.ZERO);
//                 barrelPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(-300.92, 10), 
//                                 new Vector2(-20, 0)
//                 ));

//                 // barrelPath.addSegment( 
//                 //         new PathLineSegment(
//                 //                 new Vector2(-300.92, 10),
//                 //                 new Vector2(0,10)
//                 //         )
//                 // );

//                 // *** dummy segment to make the bot stop ***
//                 barrelPath.addSegment( 
//                         new PathLineSegment(
//                                 new Vector2(-20, 0),
//                                 new Vector2(-0, 5)
//                         )
//                 );

//                 Trajectory barrelTrajectory = new Trajectory(barrelPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS); // make new constraints for autonav
//                 AutonomousTrajectoryCommand barrelCommand = new AutonomousTrajectoryCommand(barrelTrajectory);
//                 output.addSequential(barrelCommand); 

//                 // Trajectory barrel2Trajectory = new Trajectory(barrelPath2, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS); // make new constraints for autonav
//                 // AutonomousTrajectoryCommand barrel2Command = new AutonomousTrajectoryCommand(barrel2Trajectory);
//                 // output.addSequential(barrel2Command); 


//                 return output; 
//         }

//         public static CommandGroup slalom() {
//                 CommandGroup output = new CommandGroup();
//                 Path slalomPath = new Path(Rotation2.ZERO);
//                 // TODO: update values in Vector2's using tangent calculators
//                 /*
//                 ***HOW ROBOT COORDS ARE DETERMINED***
//                 X: sutract 36 from field coord and negate it 
//                 Y: subtract 90 from field coord and negate it 
//                 */
//                 double yValue = -(60 - 30); // in robot coords
                
//                 Vector2 start = new Vector2(0, 0);
//                 Vector2 D2 = new Vector2(-(60 - 36), yValue);
//                 Vector2 D4 = new Vector2(-(120 - 36), yValue);
//                 Vector2 D8 = new Vector2(-(240 - 36), yValue);
//                 Vector2 D10 = new Vector2(-(300 - 36), yValue);
                
//                 PathLineSegment startToD2 = AutoNavMath.circlePointTangent(D2, start, true, false, 35);
//                 slalomPath.addSegment(startToD2);

//                 PathLineSegment D2toD4 = AutoNavMath.circleCircleInternalTangent(D2, D4, true);

//                 // curve around D2
//                 slalomPath.addSegment(
//                         new PathArcSegment(
//                                 startToD2.getEnd(),
//                                 D2toD4.getStart(),
//                                 D2
//                         )
//                 );
                

//                 // straight line diagonal to D4
//                 slalomPath.addSegment(D2toD4);


//                 // curve around D4 
//                 PathLineSegment D4toD8 = AutoNavMath.circleCircleExternalTangent(D4, D8, false); // need deltaYLarger = false

//                 slalomPath.addSegment(
//                         new PathArcSegment(
//                                 D2toD4.getEnd(),
//                                 D4toD8.getStart(),
//                                 D4
//                         )
//                 );


//                 slalomPath.addSegment(D4toD8);

//                 // ***THIS BLOCK IS THROWING A MAX VELOCITY EXCEPTION********************
                
//                 Vector2 dummyEnd = new Vector2(D4toD8.getStart().x - 20, D4toD8.getStart().y - 20);
//                 PathLineSegment dummyClosingPath = new PathLineSegment(D8, dummyEnd);
//                 slalomPath.addSegment(dummyClosingPath);
//                 System.out.println("D8 start for dummy closing segment = " + D8.toString());
//                 System.out.println("Ending vector for dummy closing segment = " + dummyEnd.toString());
//                 System.out.println("Overall segment path = " + dummyClosingPath);
  
//                 // **********************************************************************

//                 /*
//                 PathLineSegment D8toD10 = AutoNavMath.circleCircleInternalTangent(D8, D10, false); // maybe should be true?
                
//                 // curve around D8
//                 slalomPath.addSegment(
//                         new PathArcSegment(
//                                 D4toD8.getEnd(),
//                                 D8toD10.getStart(),
//                                 D8
//                         )
//                 )
                
//                 // go to D10
//                 slalomPath.addSegment(D8toD10);

//                 PathLineSegment D10toD8 = AutoNavMath.circleCircleInternalTangent(D10, D8, true); // check largerDeltaY again

//                 // loop around D10
//                 slalomPath.addSegment(
//                         new PathArcSegment(
//                                 D8toD10.getEnd(),
//                                 D10toD8.getStart(),
//                                 D10
//                         )
//                 )

//                 // go to D8 again
//                 slalomPath.addSegment(D10toD8);

//                 // curve around D8
//                 PathLineSegment D8toD4 = AutoNavMath.circleCircleExternalTangent(D8, D4, false); // check largerDeltaY

//                 slalomPath.addSegment(
//                         new PathArcSegment(
//                                 D10toD8.getEnd(),
//                                 D8toD4.getStart(),
//                                 D8
//                         )
//                 )

//                 // go to D4
//                 slalomPath.addSegment(D8toD4);

//                 // curve around D4
//                 Vector2 end = new Vector2(-(90 - 36), -(40 - 30));
//                 PathLineSegment D4toEnd = AutoNavMath.circlePointToTangent(D4, end, false, true, 28);
//                 slalomPath.addSegment(
//                         new PathArcSegment(
//                                 D8toD4.getEnd(),
//                                 D4toEnd.getStart(),
//                                 D4
//                         )
//                 )

//                 // go to end
//                 slalomPath.addSegment(D4toEnd); 

//                 slalomPath.addSegment(D4toEnd.getEnd().x + 1, D4toEnd.getEnd.y + 1); 
//                 */
                

//                 Trajectory slalomTrajectory = new Trajectory(slalomPath, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand slalomCommand = new AutonomousTrajectoryCommand(slalomTrajectory);
//                 output.addSequential(slalomCommand); 

//                 return output; 
//         }

        
//         // public static CommandGroup slalom2() {

//         //         CommandGroup output = new CommandGroup();
//         //         Path slalomPath = new Path(Rotation2.ZERO);
//         //         IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true, 1);


//         //         slalomPath.addSegment(
//         //                 new PathLineSegment(
//         //                         new Vector2(0,0),
//         //                         new Vector2(-24, 0)
//         //                 )
//         //         ); // drive forward

//         //         slalomPath.addSegment( // arc around D2
//         //                 new PathArcSegment(
//         //                         new Vector2(-24, 0),
//         //                         new Vector2(-52, -28), 
//         //                         new Vector2(-24, -28) 
//         //                 )
//         //         );
                
//         //         slalomPath.addSegment( // arc around D4
//         //                 new PathArcSegment(
//         //                         new Vector2(-52, -28),
//         //                         new Vector2(-63.581, -60), 
//         //                         new Vector2(-102, -28) 
//         //                 )
//         //         );

//         //         slalomPath.addSegment( 
//         //                 new PathLineSegment(
//         //                         new Vector2(-63.581, -60),
//         //                         new Vector2(-240, -60)
//         //                 )
//         //         ); 

//         //         slalomPath.addSegment(
//         //                 new PathLineSegment(
//         //                         new Vector2(-240, -60),
//         //                         new Vector2(-240, 5)
//         //                 )
//         //         ); 
                
//         //         slalomPath.addSegment( // drive to end of field arc
//         //                 new PathLineSegment(
//         //                         new Vector2(-240, 5),
//         //                         new Vector2(-252, 5) // 4/3 change
//         //                 )
//         //         ); 

//         //         slalomPath.addSegment( // arc around D4
//         //                 new PathArcSegment(
//         //                         new Vector2(-252, 5),
//         //                         new Vector2(-300, -48), 
//         //                         new Vector2(-272, -30) 
//         //                 )
//         //         );

//         //         slalomPath.addSegment( // arc around D4
//         //                 new PathArcSegment(
//         //                         new Vector2(-300, -48),
//         //                         new Vector2(-228, -35), 
//         //                         new Vector2(-272, -35) 
//         //                 )
//         //         );

                
//         //         slalomPath.addSegment(
//         //                 new PathLineSegment(
//         //                         new Vector2(-228, -35),
//         //                         new Vector2(-234, 0)
//         //                 )
//         //         );
//         //         // slalomPath.addSegment( // end of arc and drives straight
//         //         //         new PathArcSegment(
//         //         //                 new Vector2(-235, -28),
//         //         //                 new Vector2(-210, 0),
//         //         //                 new Vector2(-216, -30)
//         //         //         )
//         //         // ); 

//         //         // Path fastDriveBackPath = new Path(Rotation2.ZERO);
//         //         // fastDriveBackPath.addSegment(
//         //         //         new PathLineSegment(
//         //         //                 new Vector2(0,0),
//         //         //                 new Vector2(193,0)
//         //         //         )
//         //         // );

//         //         slalomPath.addSegment(
//         //                 new PathLineSegment(
//         //                         new Vector2(-234, 0),
//         //                         new Vector2(-55, 0)
//         //                 )
//         //         ); 

//         //         //Path slalomPath2 = new Path(Rotation2.ZERO);

//         //         slalomPath.addSegment(
//         //                 new PathLineSegment(
//         //                         new Vector2(-55, 0),
//         //                         new Vector2(-55, -70)
//         //                 )
//         //         ); 
                
//         //         slalomPath.addSegment(
//         //                 new PathLineSegment(
//         //                         new Vector2(-55, -70),
//         //                         new Vector2(0, -70)
//         //                 )
//         //         ); 

//         //         // dummy segment
//         //         slalomPath.addSegment(
//         //                 new PathLineSegment(
//         //                         new Vector2(0, -65),
//         //                         new Vector2(10, -65)
//         //                 )
//         //         ); 


                
//         //         // Trajectory fastDriveBackTrajectory = new Trajectory(fastDriveBackPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//         //         // AutonomousTrajectoryCommand fastDriveBackCommand = new AutonomousTrajectoryCommand(fastDriveBackTrajectory);

//         //         // Trajectory slalomPath2Trajectory = new Trajectory(slalomPath2, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//         //         // AutonomousTrajectoryCommand slalomPath2Command = new AutonomousTrajectoryCommand(slalomPath2Trajectory);

//         //         Trajectory slalomTrajectory = new Trajectory(slalomPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//         //         AutonomousTrajectoryCommand slalomCommand = new AutonomousTrajectoryCommand(slalomTrajectory);
//         //         output.addSequential(raiseIntake);
//         //         output.addSequential(slalomCommand); 
//         //         // output.addSequential(fastDriveBackCommand); 
//         //         // output.addSequential(slalomPath2Command); 



//         //         return output;
//         // }

                        
//         public static CommandGroup slalom2() {

//                 CommandGroup output = new CommandGroup();
//                 Path slalomPath = new Path(Rotation2.ZERO);
//                 IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true, 1);


//                 slalomPath.addSegment( // drive forward
//                         new PathLineSegment(
//                                 new Vector2(0,0),
//                                 new Vector2(-24, 0)
//                         )
//                 ); 

//                 slalomPath.addSegment( // arc around D2
//                         new PathArcSegment(
//                                 new Vector2(-24, 0),
//                                 new Vector2(-52, -28), 
//                                 new Vector2(-24, -28) 
//                         )
//                 );
                
//                 slalomPath.addSegment( // arc around D4
//                         new PathArcSegment(
//                                 new Vector2(-52, -28),
//                                 new Vector2(-63.581, -60), 
//                                 new Vector2(-102, -28) 
//                         )
//                 ); 

//                 // same as slalom3 until here
//                 slalomPath.addSegment( // drive to D8 tangent
//                         new PathLineSegment(
//                                 new Vector2(-63.581, -60),
//                                 new Vector2(-240, -60)
//                         )
//                 ); 

//                 slalomPath.addSegment( // drive across gap between D8 and D10
//                         new PathLineSegment(
//                                 new Vector2(-240, -60),
//                                 new Vector2(-240, 5)
//                         )
//                 ); 
                
//                 slalomPath.addSegment( // drive forward a little 
//                         new PathLineSegment(
//                                 new Vector2(-240, 5),
//                                 new Vector2(-243, 5)
//                         )
//                 ); 

//                 slalomPath.addSegment( // arc 1 around D10
//                         new PathArcSegment(
//                                 new Vector2(-243, 5),
//                                 new Vector2(-310, -25), 
//                                 new Vector2(-270, -25) 
//                         ) // changed points to fit the circle
//                 );

//                 slalomPath.addSegment( // arc 2 around D10
//                         new PathArcSegment(
//                                 new Vector2(-310, -25),
//                                 new Vector2(-252, -60), 
//                                 new Vector2(-270, -25) 
//                         ) // changed points to fit the circle
//                 );

//                 slalomPath.addSegment( // end of arc and drives across gap
//                         new PathLineSegment(
//                                 new Vector2(-252, -60),
//                                 new Vector2(-252, 0)
//                         )
//                 );

//                 slalomPath.addSegment( // long drive back
//                         new PathLineSegment(
//                                 new Vector2(-252, 0),
//                                 new Vector2(-75, -2)
//                         )
//                 ); 

//                 slalomPath.addSegment( // drive to the left
//                         new PathLineSegment(
//                                 new Vector2(-75, -2),
//                                 new Vector2(-75, -72)
//                         )
//                 ); 
                
//                 slalomPath.addSegment( // drive into end zone
//                         new PathLineSegment(
//                                 new Vector2(-75, -72),
//                                 new Vector2(0, -72)
//                         )
//                 ); 

                
//                 slalomPath.addSegment( // dummy segment
//                         new PathLineSegment(
//                                 new Vector2(0, -72),
//                                 new Vector2(10, -72)
//                         )
//                 ); 

//                 Trajectory slalomTrajectory = new Trajectory(slalomPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS);
//                 AutonomousTrajectoryCommand slalomCommand = new AutonomousTrajectoryCommand(slalomTrajectory);
//                 output.addSequential(raiseIntake);
//                 output.addSequential(slalomCommand); 
//                 return output;
//         }

//         //This is the slalom code which worked.
//         //We have a video of this for submission.
//         //In slalom2, we made changes, didn't test, switched to another activity,
//         public static CommandGroup slalom3() {

//                 CommandGroup output = new CommandGroup();
//                 Path slalomPath = new Path(Rotation2.ZERO);


//                 slalomPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0,0),
//                                 new Vector2(-24, 0)
//                         )
//                 ); // drive forward

//                 slalomPath.addSegment( // arc around D2
//                         new PathArcSegment(
//                                 new Vector2(-24, 0),
//                                 new Vector2(-52, -28), 
//                                 new Vector2(-24, -28) 
//                         )
//                 );

//                 slalomPath.addSegment( // arc around D4
//                         new PathArcSegment(
//                                 new Vector2(-52, -28),
//                                 new Vector2(-63.581, -60), 
//                                 new Vector2(-102, -28) 
//                         )
//                 );

//                 slalomPath.addSegment( // drive to D8 tangent
//                         new PathLineSegment(
//                                 new Vector2(-63.581, -60),
//                                 new Vector2(-234, -60)
//                         )
//                 ); 

//                 slalomPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(-234, -60),
//                                 new Vector2(-234, 0)
//                         )
//                 ); 

//                 slalomPath.addSegment( // drive to end of field arc
//                         new PathLineSegment(
//                                 new Vector2(-234, 0),
//                                 new Vector2(-252, 0)
//                         )
//                 ); 

//                 slalomPath.addSegment( // arc around D4
//                         new PathArcSegment(
//                                 new Vector2(-252, 0),
//                                 new Vector2(-300, -48), 
//                                 new Vector2(-270, -30) 
//                         )
//                 );

//                 slalomPath.addSegment( // arc around D4
//                         new PathArcSegment(
//                                 new Vector2(-300, -48),
//                                 new Vector2(-235, -28), 
//                                 new Vector2(-270, -30) 
//                         )
//                 );

//                 slalomPath.addSegment( // end of arc and drives straight
//                         new PathLineSegment(
//                                 new Vector2(-235, -28),
//                                 new Vector2(-235, 5)
//                         )
//                 ); 

//                 slalomPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(-235, 5),
//                                 new Vector2(-42, 5)
//                         )
//                 ); 

//                 slalomPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(-42, 5),
//                                 new Vector2(-42, -60)
//                         )
//                 ); 

//                 slalomPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(-42, -60),
//                                 new Vector2(0, -60)
//                         )
//                 ); 

//                 slalomPath.addSegment(
//                         new PathLineSegment(
//                                 new Vector2(0, -60),
//                                 new Vector2(5, -60)
//                         )
//                 ); 



//                 Trajectory slalomTrajectory = new Trajectory(slalomPath, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand slalomCommand = new AutonomousTrajectoryCommand(slalomTrajectory);
//                 output.addSequential(slalomCommand); 

//                 return output;
//         }

//         public static CommandGroup bounce() {
//                 CommandGroup output = new CommandGroup();
//                 Path barrelPath = new Path(Rotation2.ZERO);
//                 // TODO: update values in Vector2's using tangent calculators
//                 /*
//                 Field to robot coordinates:
//                 https://www.desmos.com/calculator/reunlnhdlc
//                 */
//                 //Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Rotation2.fromDegrees(90 + drivetrain.getGyroscope().getUnadjustedAngle().toDegrees()));

//                 double robotYOffset = -4; //-5.625; // so that center of the bot is lined up with front of start zone

//                 Vector2 start = new Vector2(0, 0); // robot start position with front bumper lined up at the edge of start zone 
//                 Vector2 B2 = new Vector2((-24 + robotYOffset), -30);
//                 PathLineSegment startToB2Line = AutoNavMath.circlePointTangent(B2, start, true, false, 28);
//                 // start to B2 
//                 barrelPath.addSegment( 
//                         startToB2Line
//                 );
                
//                 // avoid B2 start zone cone
//                 Vector2 A3 = new Vector2((-(90-36) + robotYOffset), -(150-90)); 
//                 // determine the PathLineSegment that goes after the arc so we can use its start Vector2 field as the end point of the arc
//                 PathLineSegment B2toA3Line = AutoNavMath.circlePointTangent(B2, A3, true, true, 28);
//                 barrelPath.addSegment( 
//                         new PathArcSegment(
//                                 startToB2Line.getEnd(), // start point
//                                 B2toA3Line.getStart(), // end point
//                                 B2 // center point
//                         ) //, new Rotation2(0, 0, true) // spin the robot as it loops around
//                 );
                
                
//                 barrelPath.addSegment( // straight line to A3
//                        B2toA3Line
//                 );

//                 // straight line to B4
//                 // Vector2 B4 = new Vector2(-(120 - 36) + robotYOffset, -(120 - 90));
//                 // PathLineSegment A3toB4Line = AutoNavMath.circlePointTangent(B4, A3, false, false, 28);
//                 // barrelPath.addSegment(
//                 //         A3toB4Line
//                 // ); 

//                 // curve around B4
//                 Vector2 D5 = new Vector2(-(150-36) + robotYOffset, -(60-90));
//                 // PathLineSegment B4toD5Line = AutoNavMath.circleCircleExternalTangent(B4, D5, false);
//                 // barrelPath.addSegment( 
//                 //         new PathArcSegment(
//                 //                 A3toB4Line.getEnd(), // start point
//                 //                 B4toD5Line.getStart(), // end point
//                 //                 B4 // center point
//                 //         ) // , new Rotation2(0, 0, true) // spin the robot as it loops around
//                 // );

//                 Vector2 B4prime = new Vector2(-(105 - 36) + robotYOffset, -(50 - 90));

//                 // straight line to D5
//                 // ***FROM A3 TO D5
//                 // PathLineSegment A3toD5Line = AutoNavMath.circlePointTangent(D5, A3, false, false, 40); 
//                 // barrelPath.addSegment( 
//                 //        A3toD5Line
//                 // );
//                 // ***FROM A3 TO D5 IN TWO STRAIGHT LINE SEGMENTS
//                 PathLineSegment A3toB4primeLine = new PathLineSegment(A3, B4prime);
//                 PathLineSegment B4primetoD5Line = AutoNavMath.circlePointTangent(D5, B4prime, true, false, 28);
//                 System.out.println("Tan line from B4'-D5 = " + B4primetoD5Line); 
                                 
//                 // Tan line from B4'-D5 = {start: (-78.000, 40.000), end: (-94.568, 14.672)}

//                 barrelPath.addSegment(A3toB4primeLine);
//                 barrelPath.addSegment(B4primetoD5Line); 

//                 // loop around D5
//                 Vector2 A6 = new Vector2(-(180-36) + robotYOffset, -(150-90));
//                 PathLineSegment D5toA6Line = AutoNavMath.circlePointTangent(D5, A6, true, true, 28);
//                 PathArcSegment loopAroundD5 = new PathArcSegment(
//                                 B4primetoD5Line.getEnd(), // start point
//                                 D5toA6Line.getStart(), // end point
//                                 D5 // center point
//                         );
//                 barrelPath.addSegment( 
//                         loopAroundD5
//                 );
//                 System.out.println("Start and end for the arc set by the tan lines B4'-D5 and D5-A = " + loopAroundD5);
//                 System.out.println("Tangent line from circle centered at D5 to point A6 = " + D5toA6Line.toString());
                
//                 barrelPath.addSegment( 
//                         D5toA6Line
//                 );
                
//                 // straight line to D7
//                 Vector2 D7 = new Vector2(-(210-36) + robotYOffset, -(60-90));
//                 PathLineSegment A6toD7Line = AutoNavMath.circlePointTangent(D7, A6, false, false, 28);
//                 barrelPath.addSegment( 
//                         A6toD7Line
//                 );          
                
                
//                 Vector2 D8 = new Vector2(-(240-36) + robotYOffset, -(60-90));
//                 Vector2 A9 = new Vector2(-(270-36) + robotYOffset, -(150-90));
//                 // haven't added B10 or C11 coordinates 

//                 PathLineSegment D7toD8Line = AutoNavMath.circleCircleExternalTangent(D7, D8, false);

//                 // loop around D7
//                 barrelPath.addSegment( 
//                         new PathArcSegment(
//                                 A6toD7Line.getEnd(), // start point
//                                 D7toD8Line.getStart(), // end point
//                                 D7 // center point
//                         ) // , new Rotation2(0, 0, true) // spin the robot as it loops around
//                 );

//                 // straight line to D8
//                 barrelPath.addSegment( 
//                         AutoNavMath.circleCircleExternalTangent(D7, D8, false)
//                 );
                
//                 // loop around D8
//                 PathLineSegment D8toA9Line = AutoNavMath.circlePointTangent(D8, A9, false, true, 28);
//                 barrelPath.addSegment( 
//                         new PathArcSegment(
//                                 D7toD8Line.getEnd(), // start point
//                                 D8toA9Line.getStart(), // end point
//                                 D8 // center point
//                         ) // , new Rotation2(0, 0, true) // spin the robot as it loops around
//                 );

//                 // drive straight to A9
//                 barrelPath.addSegment(
//                         D8toA9Line
//                 );

//                 // drive to circle centered at B10
//                 Vector2 B10 = new Vector2(-(300 - 36) + robotYOffset, -(120 - 90));
//                 Vector2 end = new Vector2(-(330 - 35) + robotYOffset, -(90 - 90));
                
//                 barrelPath.addSegment(
//                         new PathLineSegment(A9, B10)
//                 );

//                 barrelPath.addSegment(
//                         new PathLineSegment(B10, end)
//                 );

//                 // barrelPath.addSegment( // loop around B10
//                 //         new PathArcSegment(
//                 //                 new Vector2(0, 0), // start point
//                 //                 new Vector2(0, 0), // end point
//                 //                 new Vector2(0, 0) // center point
//                 //         ), new Rotation2(0, 0, true) // spin the robot as it loops around
//                 // ); 
//                 // // TODO see if it's possible to drive straight into finish zone and break the plane without looping around B10 (similar to slalom ending)
//                 // barrelPath.addSegment(
//                 //         AutoNavMath.circlePointTangent(B10, C11, false, true, 28)
//                 // );

//                 // arbitrary line segment that caps the sequence
//                 barrelPath.addSegment( // straight line to A3
//                        new PathLineSegment(
//                                B2toA3Line.getEnd(),
//                                new Vector2(B2toA3Line.getEnd().x - 1, B2toA3Line.getEnd().y - 1)
//                        )
//                 ); 

//                 Trajectory barrelTrajectory = new Trajectory(barrelPath, Robot.drivetrainSubsystem.CONSTRAINTS);
//                 AutonomousTrajectoryCommand barrelCommand = new AutonomousTrajectoryCommand(barrelTrajectory);
//                 output.addSequential(barrelCommand); 

//                 return output; 
//         }
        
//         public static CommandGroup bounce2() {
//                 CommandGroup output = new CommandGroup();
//                 Path barrelPath = new Path(Rotation2.ZERO);
        
//                         barrelPath.addSegment(
//                                 new PathLineSegment(
//                                         new Vector2(0,0),
//                                         new Vector2(-36,0)
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathArcSegment(
//                                         new Vector2(-36, 0), // start point
//                                         new Vector2(-44, -30), // end point
//                                         new Vector2(0, -30) // center point
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-44, -30), // start point
//                                         new Vector2(-54, -50) // end point
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-54, -50), // start point
//                                         new Vector2(-54, 0) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-54, 0), // start point
//                                         new Vector2(-84, 0) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-84, 0), // start point
//                                         new Vector2(-84, 62) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-84, 62), // start point
//                                         new Vector2(-148, 62) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-148, 62), // start point
//                                         new Vector2(-148, -55) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-148, -55), // start point
//                                         new Vector2(-148, 65) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-148, 65), // start point
//                                         new Vector2(-246, 65) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-246, 65), // start point
//                                         new Vector2(-246, -50) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-246, -50), // start point
//                                         new Vector2(-246, 0) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-246, 0), // start point
//                                         new Vector2(-270, 0) // end point   
//                                 )
//                         );
//                         barrelPath.addSegment( // loop around D5
//                                 new PathLineSegment(
//                                         new Vector2(-270, 0), // start point
//                                         new Vector2(-260, 0) // end point   
//                                 )
//                         );

                        
//                         // barrelPath.addSegment( 
//                         //         new PathArcSegment(
//                         //                 new Vector2(-89, 30),
//                         //                 new Vector2(-114,55),
//                         //                 new Vector2(-114,30)
//                         //         )
//                         // );
//                         // barrelPath.addSegment( 
//                         //         new PathArcSegment(
//                         //                 new Vector2(-114,55), // start point
//                         //                 new Vector2(-139,30), // end point
//                         //                 new Vector2(-114,30) // center point
//                         //         )
//                         // );
//                         // barrelPath.addSegment( 
//                         //         new PathLineSegment(
//                         //                 new Vector2(-139, 30), // start point
//                         //                 new Vector2(-144, -60) // end point
//                         //         )       
//                         // );
//                         // barrelPath.addSegment( 
//                         //         new PathLineSegment(
//                         //                 new Vector2(-144, -60),
//                         //                 new Vector2(-160,43.775)
//                         //         )
//                         // );
//                         // barrelPath.addSegment( 
//                         //         new PathArcSegment(
//                         //                 new Vector2(-160,43.775), // start point
//                         //                 new Vector2(-229,30), // end point
//                         //                 new Vector2(-189,30) // center point
//                         //         )
//                         // );
                    
//                         // barrelPath.addSegment( 
//                         //         new PathLineSegment(
//                         //                 new Vector2(-229,30),
//                         //                 new Vector2(-234,-60)
//                         //         )
//                         // );
//                         // barrelPath.addSegment( 
//                         //         new PathLineSegment(
//                         //                 new Vector2(-234,-60),
//                         //                 new Vector2(-240, -70)
//                         //         )
//                         // );
        
//                         Trajectory barrelTrajectory = new Trajectory(barrelPath, Robot.drivetrainSubsystem.CONSTRAINTS); // make new constraints for autonav
//                         AutonomousTrajectoryCommand barrelCommand = new AutonomousTrajectoryCommand(barrelTrajectory);
//                         output.addSequential(barrelCommand); 
        
//                         return output; 	
//         }
        
       
//     //Lifts intake
//     //Drives forward 5 inches
//     //Spins intake
//     //Lowers intake and calls elevator state machine
//     //Drives backward 5 inches.
//     public static CommandGroup CollectPowerCell(){
//         Path fiveInchesPath = new Path(Rotation2.ZERO);
//         fiveInchesPath.addSegment(
//                 new PathLineSegment(
//                         new Vector2(0.0, 0.0),
//                         new Vector2(5, 0)
//                 )
//         );


//         Trajectory fiveInchesForward = new Trajectory(fiveInchesPath, Robot.drivetrainSubsystem.CONSTRAINTS);


//         CommandGroup output = new CommandGroup();
        
//         AutonomousTrajectoryCommand trajectoryCommand = new AutonomousTrajectoryCommand(fiveInchesForward);
//         IntakeCommand intakeCommand = new IntakeCommand(false);
//         output.addParallel(intakeCommand);
//         output.addParallel(trajectoryCommand);
// 	return output;
//     }


//     public static String getMethodName()
// 	{
// 		String methodName = Thread.currentThread().getStackTrace()[2].getMethodName();
// 		return methodName;
// 	}
// }