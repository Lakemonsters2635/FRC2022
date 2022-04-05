package frc.robot.models;


import com.fasterxml.jackson.databind.ser.std.InetSocketAddressSerializer;

// import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
// import org.frcteam2910.common.control.PathArcSegment;
// import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;

import edu.wpi.first.wpilibj.command.CommandGroup;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AreWeThereYetCommand;
import frc.robot.commands.AutonomousTrajectoryCommand;
import frc.robot.commands.DoNothingCommand;
import frc.robot.commands.FetchCargoCommand;
import frc.robot.commands.FetchCargoCommand2;
// import frc.robot.commands.GalacticSearchCommand;
import frc.robot.commands.IntakeActuateCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.IntakeRetractCommand2;
import frc.robot.commands.RobotRotateCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterIdleCommand;
import frc.robot.commands.VisionRotationDriveCommand;

public class AutonomousSequences {
        public static double sampleDistance = 12.0;
        public static double startingVelocity = 1.0;
        public static double endingVelocity = 1.0;

        public static CommandGroup shootCollectRight() {
            CommandGroup output = new CommandGroup();

            IntakeActuateCommand retractCommand = new IntakeActuateCommand(false, 1);
            IntakeActuateCommand retractCommand2 = new IntakeActuateCommand(false, 1);

            IntakeActuateCommand extendCommand = new IntakeActuateCommand(true, 1);
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
            ShooterIdleCommand shooterCommand = new ShooterIdleCommand(1000.0);            
            ShooterIdleCommand shooterCommand2 = new ShooterIdleCommand(1000.0);
            ShooterIdleCommand revUpCommand = new ShooterIdleCommand(1000.0);
           
            ShooterIdleCommand revUpCommand2 = new ShooterIdleCommand(1000.0);
            IntakeCommand runIntake = new IntakeCommand(false);
            IntakeCommand runIntake2 = new IntakeCommand(false);
            IntakeCommand driveRunIntake = new IntakeCommand(false);

            output.addSequential(revUpCommand, 2);

            output.addParallel(retractCommand);
            output.addParallel(runIntake);
            output.addSequential(shooterCommand, 2);
            // output.addSequential(rotateCommand1, 4);
            output.addSequential(rotateCommand2, 4);
            output.addSequential(extendCommand);
            output.addParallel(driveRunIntake);
            output.addSequential(driveBackCommand);
            // output.addSequential(rotateCommand3, 4);

            output.addSequential(rotateCommand4, 4);

            output.addSequential(revUpCommand2, 2);
            output.addParallel(retractCommand2);
            output.addParallel(runIntake2);
            output.addSequential(shooterCommand2, 2);

            
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
            .lineTo(new Vector2(-60.0, 0.0));

            Path driveOverPath = driveOver.build();
            
            Trajectory driveOverTrajectory = new Trajectory(driveOverPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveOverCommand = new AutonomousTrajectoryCommand(driveOverTrajectory);

            // output.addSequential(shooterCommand);
            output.addSequential(driveOverCommand);
            return output;
        }

        public static CommandGroup shootPassTarmacLine() {
            CommandGroup output = new CommandGroup();

            SimplePathBuilder driveStart = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
            .lineTo(new Vector2(55.0, 0.0));

            Path driveStartPath = driveStart.build();
            
            Trajectory driveStartTrajectory = new Trajectory(driveStartPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveStartCommand = new AutonomousTrajectoryCommand(driveStartTrajectory);

            SimplePathBuilder driveOver = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
            .lineTo(new Vector2(55.0, 0.0));

            Path driveOverPath = driveOver.build();

            IntakeActuateCommand extendIntakeCommand = new IntakeActuateCommand(true, 2);
            IntakeActuateCommand retractIntakeCommand = new IntakeActuateCommand(false, 2);

            ShooterCommand shooterCommand = new ShooterCommand(true);
            ShooterIdleCommand revUpCommand = new ShooterIdleCommand(2800.0);
            IntakeCommand runIntake = new IntakeCommand(false);


            
            Trajectory driveOverTrajectory = new Trajectory(driveOverPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveOverCommand = new AutonomousTrajectoryCommand(driveOverTrajectory);
            VisionRotationDriveCommand visionRotateCommand = new VisionRotationDriveCommand(2);
            // output.addSequential(extendIntakeCommand);
            
            // output.addParallel(retractIntakeCommand);
            Robot.vision.ledOn();

            output.addSequential(driveOverCommand);

            output.addParallel(revUpCommand, 2);
            output.addSequential(visionRotateCommand);
            
            output.addParallel(runIntake);
            output.addParallel(retractIntakeCommand);
            Robot.vision.ledOn();

            output.addSequential(shooterCommand, 2);

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
  
        public static CommandGroup testFetchCargoCommand2RED() {
            CommandGroup output = new CommandGroup();
            FetchCargoCommand2 fcc = new FetchCargoCommand2("red", 100);             
            output.addSequential(fcc);
        
            return output;
        }

        public static CommandGroup testFetchCargoCommand2BLUE() {
            CommandGroup output = new CommandGroup();
            FetchCargoCommand2 fcc = new FetchCargoCommand2("blue", 100);             
            output.addSequential(fcc);
        
            return output;
        }

        public static CommandGroup testfccOLD() {
            CommandGroup output = new CommandGroup();
            FetchCargoCommand fcc = new FetchCargoCommand("red"); 
            output.addSequential(fcc);
            
            return output;
        }

        public static CommandGroup testVisionDriveToPickUpCargo() {
            CommandGroup output = new CommandGroup(); 
            FetchCargoCommand2 fcc = new FetchCargoCommand2("red", 100); 
            IntakeActuateCommand extendIntake = new IntakeActuateCommand(true, 2); 
            IntakeCommand spinIntake = new IntakeCommand(false, 3); // need to find the right timeout so that it stops spinning when intake retracts
            FetchCargoCommand fccshort = new FetchCargoCommand("red", 3);
            IntakeRetractCommand2 retractIntake = new IntakeRetractCommand2(2); 

            output.addSequential(fcc);
            output.addParallel(extendIntake);
            output.addParallel(spinIntake);
            output.addSequential(fccshort);

            // output.addSequential(retractIntake);

            return output; 
        }

        public static CommandGroup twoFccOld() {
            CommandGroup output = new CommandGroup();
            FetchCargoCommand fcc1 = new FetchCargoCommand("red");
            RobotRotateCommand rr = new RobotRotateCommand(90); 
            FetchCargoCommand fcc2 = new FetchCargoCommand("red");

            output.addSequential(fcc1, 5);

            output.addSequential(rr);
            output.addSequential(fcc2, 5);
            return output; 
        }

        public static CommandGroup twoBallAuto(String cargoColor, double estimatedRotationToCargo, double estimatedRotationToHubAfter2ndCargo) { // previously "threeBallAuto()"
            CommandGroup output = new CommandGroup();

            // collectball goes 20 in forward after the cargo goes out of sight due to intake blocking it
            SimplePathBuilder collectBall = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(-20.0, 0.0));
            collectBall.lineTo(new Vector2(-20.0, 0.0));

            Path collectBallPath = collectBall.build();
            
            Trajectory collectBallTrajectory = new Trajectory(collectBallPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand collectBallCommand = new AutonomousTrajectoryCommand(collectBallTrajectory);
            // --------------------------

            FetchCargoCommand fcc1 = new FetchCargoCommand(cargoColor);

            // rr and rr2 to be parameters
            RobotRotateCommand rotate2ndCargoInFrame = new RobotRotateCommand(estimatedRotationToCargo); //60 
            RobotRotateCommand rotateToHubAfter2ndCargoIn = new RobotRotateCommand(estimatedRotationToHubAfter2ndCargo); // -80
            // RobotRotateCommand rr3 = new RobotRotateCommand(90); 
            // RobotRotateCommand rr4 = new RobotRotateCommand(-90);
            // FetchCargoCommand fcc2 = new FetchCargoCommand(cargoColor);

            // drive back at very beginning to shoot 1st cargo
            SimplePathBuilder driveStart = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
            .lineTo(new Vector2(60.0, 0.0));

            Path driveStartPath = driveStart.build();
            
            Trajectory driveStartTrajectory = new Trajectory(driveStartPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveStartCommand = new AutonomousTrajectoryCommand(driveStartTrajectory);
            // --------------------------

            IntakeActuateCommand extendIntakeCommand = new IntakeActuateCommand(true, 2);
            IntakeActuateCommand retractIntakeCommand = new IntakeActuateCommand(false, 2);
            // IntakeActuateCommand extendIntakeCommand2 = new IntakeActuateCommand(true, 2);
            IntakeActuateCommand retractIntakeCommand2 = new IntakeActuateCommand(false, 2);
            // IntakeActuateCommand extendIntakeCommand3 = new IntakeActuateCommand(true, 2);
            // IntakeActuateCommand retractIntakeCommand3 = new IntakeActuateCommand(false, 2);

            ShooterCommand shooterCommand = new ShooterCommand(true);
            ShooterCommand shooterCommand2 = new ShooterCommand(true);
            // ShooterCommand shooterCommand3 = new ShooterCommand(true);

            ShooterIdleCommand revUpCommand = new ShooterIdleCommand(2500.0);
            ShooterIdleCommand revUpCommand2 = new ShooterIdleCommand(2700.0);
            ShooterIdleCommand revUpCommand3 = new ShooterIdleCommand(0.0);
            ShooterIdleCommand revUpCommand4 = new ShooterIdleCommand(2800.0);


            IntakeCommand runIntake = new IntakeCommand(false);
            IntakeCommand runIntake2 = new IntakeCommand(false);
            IntakeCommand runIntake3 = new IntakeCommand(false);
            IntakeCommand runIntake4 = new IntakeCommand(false);


            VisionRotationDriveCommand visionRotateCommand = new VisionRotationDriveCommand(2);
            VisionRotationDriveCommand visionRotateCommand2 = new VisionRotationDriveCommand(2);
            DoNothingCommand doNothing = new DoNothingCommand(2);
            // output.addSequential(extendIntakeCommand);
            
            // taxis out of cargo + revs up shooter
            // output.addParallel(retractIntakeCommand);
            output.addParallel(revUpCommand, 2);
            output.addSequential(driveStartCommand);
            
            Robot.vision.ledOn();
            output.addParallel(revUpCommand2);
            output.addSequential(visionRotateCommand, 1.5);
            
            // retracts intake, shoots
            output.addParallel(runIntake);
            output.addParallel(retractIntakeCommand);
            Robot.vision.ledOn();

            output.addSequential(shooterCommand, 1);

            // extends intake, drives to red cargo
            // uncomment everything below this line 
            output.addParallel(extendIntakeCommand);

            // rotation to get cargo in frame
            output.addSequential(rotate2ndCargoInFrame, 2);

            // collects intake
            // output.addSequential(driveOverCommand);
            output.addParallel(runIntake2);
            output.addParallel(revUpCommand3);

            output.addSequential(fcc1, 3);

            output.addParallel(runIntake3);
            output.addSequential(collectBallCommand, 1); 

            output.addSequential(doNothing, .1);// ends the second leg of grabbing red cargo

            // Rotation to get in ballpark of hub
            output.addSequential(rotateToHubAfter2ndCargoIn, 1.5);


            // shoots cargo
            Robot.vision.ledOn();
            output.addParallel(revUpCommand4);
            output.addSequential(visionRotateCommand2, 1.5);

            
            output.addParallel(runIntake4);
            output.addParallel(retractIntakeCommand2);
            Robot.vision.ledOn();

            output.addSequential(shooterCommand2, 1);

            // output.addSequential(fcc2, 5);


            return output; 
        }

      
    

        public static CommandGroup limelightTest() {
            CommandGroup output = new CommandGroup(); 

            // taxi out of tarmac
            SimplePathBuilder driveStart = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
            .lineTo(new Vector2(50.0, 0.0));

            Path driveStartPath = driveStart.build();
            
            Trajectory driveStartTrajectory = new Trajectory(driveStartPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveStartCommand = new AutonomousTrajectoryCommand(driveStartTrajectory);

            // shooter + intake retract
            ShooterCommand revUpCommand = new ShooterCommand(true);
            ShooterCommand shooterCommand = new ShooterCommand(true); 
            VisionRotationDriveCommand rotate = new VisionRotationDriveCommand(2);
            IntakeActuateCommand retract = new IntakeActuateCommand(false, 2);


            Robot.vision.ledOn();
            output.addSequential(rotate);

            output.addSequential(revUpCommand, 2);
            output.addParallel(retract);

            output.addSequential(shooterCommand, 2); 

            return output; 
        }

        

        public static CommandGroup twoBallAutoWithArc(boolean withVision) { // previously "threeBallAuto()"
            CommandGroup output = new CommandGroup();

            SimplePathBuilder collectBall = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(-20.0, 0.0));
            collectBall.lineTo(new Vector2(-20.0, 0.0));
 
            Path collectBallPath = collectBall.build();
            
            Trajectory collectBallTrajectory = new Trajectory(collectBallPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand collectBallCommand = new AutonomousTrajectoryCommand(collectBallTrajectory);
            
            FetchCargoCommand fcc1 = new FetchCargoCommand("red", 5);

            // // rr and rr2 to be parameters
            // RobotRotateCommand rr = new RobotRotateCommand(estimatedRotationToCargo); //60 
            // RobotRotateCommand rr2 = new RobotRotateCommand(estimatedRotationToHubAfter2ndCargo); // -80
            // RobotRotateCommand rr3 = new RobotRotateCommand(90); 
            // RobotRotateCommand rr4 = new RobotRotateCommand(-90);
            // FetchCargoCommand fcc2 = new FetchCargoCommand(cargoColor);

            SimplePathBuilder pathBuilder = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
            pathBuilder.arcTo(new Vector2(113.444, -11.855), new Vector2(115.87/2, 0), Rotation2.ZERO);

            Path path = pathBuilder.build();
                     
            Trajectory driveTrajectory = new Trajectory(path, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveCommand1 = new AutonomousTrajectoryCommand(driveTrajectory);

            SimplePathBuilder driveOver = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
            .lineTo(new Vector2(-60.0, 0.0));

            Path driveOverPath = driveOver.build();

            Trajectory driveOverPathTrajectory = new Trajectory(driveOverPath, Robot.drivetrainSubsystem.FETCH_CARGO_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            AutonomousTrajectoryCommand driveOverPathCommand = new AutonomousTrajectoryCommand(driveOverPathTrajectory);

            IntakeActuateCommand extendIntakeCommand = new IntakeActuateCommand(true, 2);
            IntakeActuateCommand retractIntakeCommand = new IntakeActuateCommand(false, 2);
            // IntakeActuateCommand extendIntakeCommand2 = new IntakeActuateCommand(true, 2);
            IntakeActuateCommand retractIntakeCommand2 = new IntakeActuateCommand(false, 2);
            // IntakeActuateCommand extendIntakeCommand3 = new IntakeActuateCommand(true, 2);
            // IntakeActuateCommand retractIntakeCommand3 = new IntakeActuateCommand(false, 2);

            // ShooterCommand shooterCommand = new ShooterCommand(true);
            ShooterIdleCommand shooterCommand = new ShooterIdleCommand(3700.0);
            // ShooterCommand shooterCommand2 = new ShooterCommand(true);
            ShooterIdleCommand shooterCommand2 = new ShooterIdleCommand(2875.0);

            // ShooterCommand shooterCommand3 = new ShooterCommand(true);

            ShooterIdleCommand revUpCommand = new ShooterIdleCommand(3500.0);
            // ShooterCommand revUpCommand2 = new ShooterCommand(true);
            ShooterIdleCommand revUpCommand2 = new ShooterIdleCommand(3700.0);
            ShooterIdleCommand revUpCommand3 = new ShooterIdleCommand(0.0);
            // ShooterCommand revUpCommand4 = new ShooterCommand(true);
            ShooterIdleCommand revUpCommand4 = new ShooterIdleCommand(2875.0);


            IntakeCommand runIntake = new IntakeCommand(false);
            IntakeCommand runIntake2 = new IntakeCommand(false, true);
            IntakeCommand runIntake3 = new IntakeCommand(false);
            IntakeCommand runIntake4 = new IntakeCommand(false);


            // Trajectory driveOverTrajectory = new Trajectory(driveOverPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
            // AutonomousTrajectoryCommand driveOverCommand = new AutonomousTrajectoryCommand(driveOverTrajectory);
            VisionRotationDriveCommand visionRotateCommand = new VisionRotationDriveCommand(2);
            VisionRotationDriveCommand visionRotateCommand2 = new VisionRotationDriveCommand(2);
            DoNothingCommand doNothing = new DoNothingCommand(2);
            // output.addSequential(extendIntakeCommand);
            
            // taxis out of cargo + revs up shooter
            // output.addParallel(retractIntakeCommand);
            // output.addParallel(revUpCommand);
            output.addSequential(driveCommand1);
            
            Robot.vision.ledOn();
            output.addParallel(revUpCommand2);
            output.addSequential(visionRotateCommand, 1.5);
            
            // retracts intake, shoots
            output.addParallel(runIntake);
            output.addParallel(retractIntakeCommand);
            Robot.vision.ledOn();

            output.addSequential(shooterCommand, 1.5);

            output.addParallel(revUpCommand3);
            output.addParallel(extendIntakeCommand);
            output.addParallel(runIntake2);

            if (withVision) {
                output.addSequential(fcc1);
            } else {
                output.addSequential(driveOverPathCommand);
            }

            output.addParallel(revUpCommand4);
            output.addSequential(visionRotateCommand2, 2);

            // output.addParallel(runIntake4);
            output.addParallel(retractIntakeCommand2);
            Robot.vision.ledOn();

            output.addSequential(shooterCommand2, 1.5);
            // output.addSequential(fcc1, 3);

            // extends intake, drives to red cargo
            // uncomment everything below this line 
            // output.addParallel(extendIntakeCommand);

            // // rotation to get cargo in frame
            // output.addSequential(rr);

            // // collects intake
            // // output.addSequential(driveOverCommand);
            // output.addParallel(runIntake2);
            // output.addParallel(revUpCommand3);


            // output.addParallel(runIntake3);
            // output.addSequential(collectBallCommand, 2); 

            // output.addSequential(doNothing, .1);// ends the second leg of grabbing red cargo

            // // Rotation to get in ballpark of hub
            // output.addSequential(rr2, 2);


            // // shoots cargo
            // Robot.vision.ledOn();
            // output.addParallel(revUpCommand4);
            // output.addSequential(visionRotateCommand2, 1.5);

            
            // output.addParallel(runIntake4);
            // output.addParallel(retractIntakeCommand2);
            // Robot.vision.ledOn();

            // output.addSequential(shooterCommand2, 1);

            // output.addSequential(fcc2, 5);


            return output; 
        }
        
        public static CommandGroup twoBallAutoWithArcVISION() { // previously "threeBallAuto()"
        // NOT IN USE - SEE IF CONDITION ON twoBallAutoWithArc()
        CommandGroup output = new CommandGroup();

        SimplePathBuilder collectBall = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(-20.0, 0.0));
        collectBall.lineTo(new Vector2(-20.0, 0.0));

        Path collectBallPath = collectBall.build();
        
        Trajectory collectBallTrajectory = new Trajectory(collectBallPath, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
        AutonomousTrajectoryCommand collectBallCommand = new AutonomousTrajectoryCommand(collectBallTrajectory);
        
        FetchCargoCommand fcc1 = new FetchCargoCommand("red", 5);

        SimplePathBuilder pathBuilder = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
        pathBuilder.arcTo(new Vector2(113.444, -11.855), new Vector2(115.87/2, 0), Rotation2.ZERO);

        Path path = pathBuilder.build();
                 
        Trajectory driveTrajectory = new Trajectory(path, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
        AutonomousTrajectoryCommand driveCommand1 = new AutonomousTrajectoryCommand(driveTrajectory);

        SimplePathBuilder driveOver = new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
        .lineTo(new Vector2(-60.0, 0.0));

        Path driveOverPath = driveOver.build();

        Trajectory driveOverPathTrajectory = new Trajectory(driveOverPath, Robot.drivetrainSubsystem.FETCH_CARGO_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
        AutonomousTrajectoryCommand driveOverPathCommand = new AutonomousTrajectoryCommand(driveOverPathTrajectory);

        IntakeActuateCommand extendIntakeCommand = new IntakeActuateCommand(true, 2);
        IntakeActuateCommand retractIntakeCommand = new IntakeActuateCommand(false, 2);
        IntakeActuateCommand retractIntakeCommand2 = new IntakeActuateCommand(false, 2);

        ShooterIdleCommand shooterCommand = new ShooterIdleCommand(3700.0);
        ShooterIdleCommand shooterCommand2 = new ShooterIdleCommand(2875.0);

        ShooterIdleCommand revUpCommand = new ShooterIdleCommand(3500.0);
        ShooterIdleCommand revUpCommand2 = new ShooterIdleCommand(3700.0);
        ShooterIdleCommand revUpCommand3 = new ShooterIdleCommand(0.0);
        ShooterIdleCommand revUpCommand4 = new ShooterIdleCommand(2875.0);


        IntakeCommand runIntake = new IntakeCommand(false);
        IntakeCommand runIntake2 = new IntakeCommand(false, true);
        IntakeCommand runIntake3 = new IntakeCommand(false);
        IntakeCommand runIntake4 = new IntakeCommand(false);


        VisionRotationDriveCommand visionRotateCommand = new VisionRotationDriveCommand(2);
        VisionRotationDriveCommand visionRotateCommand2 = new VisionRotationDriveCommand(2);
        DoNothingCommand doNothing = new DoNothingCommand(2);
        
        // taxis out of cargo + revs up shooter
        output.addSequential(driveCommand1);
        
        Robot.vision.ledOn();
        output.addParallel(revUpCommand2);
        output.addSequential(visionRotateCommand, 1.5);
        
        // retracts intake, shoots
        output.addParallel(runIntake);
        output.addParallel(retractIntakeCommand);
        Robot.vision.ledOn();

        output.addSequential(shooterCommand, 1.5);

        output.addParallel(revUpCommand3);
        output.addParallel(extendIntakeCommand);
        output.addParallel(runIntake2);
        // output.addSequential(driveOverPathCommand); // drives forward to second cargo
        output.addSequential(fcc1);

        output.addParallel(revUpCommand4);
        output.addSequential(visionRotateCommand2, 2);

        // output.addParallel(runIntake4);
        output.addParallel(retractIntakeCommand2);
        Robot.vision.ledOn();

        output.addSequential(shooterCommand2, 1.5);
        // output.addSequential(fcc1, 3);

        return output; 
    }

        public static String getMethodName() {
		    String methodName = Thread.currentThread().getStackTrace()[2].getMethodName();
	    	return methodName;
	    }
        // public static CommandGroup twoBallAutoWithArc() {
        //     CommandGroup output = new CommandGroup();

        //     SimplePathBuilder pathBuilder = new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO);
        //     pathBuilder.arcTo(new Vector2(115.87, 0.0), new Vector2(115.87/2, 0), Rotation2.ZERO);

        //     Path path = pathBuilder.build();
                     
        //     Trajectory driveTrajectory = new Trajectory(path, Robot.drivetrainSubsystem.AUTONOMOUS_CONSTRAINTS, sampleDistance, startingVelocity, endingVelocity);
        //     AutonomousTrajectoryCommand driveCommand1 = new AutonomousTrajectoryCommand(driveTrajectory);

        //     output.addSequential(driveCommand1);

        //     return output;
        // }

}
