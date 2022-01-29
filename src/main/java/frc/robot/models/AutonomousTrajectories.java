package frc.robot.models;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.Side;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class AutonomousTrajectories {
    private static final int SUBDIVIDE_ITERATIONS = 8;
    private final Trajectory turnTrajectory;

    private final Trajectory helloTrajectory;
    private final Trajectory helloArcTrajectory;
    private final Trajectory helloArcReverseTrajectory;

    private final Trajectory loadingBayToShootingTrajectory;

    private final Trajectory sixthTrajectory;

    private final Trajectory shootingToTrenchPickupTrajectory;

    public AutonomousTrajectories(ITrajectoryConstraint... constraints) {
        Path turnPath = new Path(Rotation2.ZERO);
        
        turnPath.addSegment(
                new PathLineSegment(
                        new Vector2(0,0),
                        new Vector2(100,0)
                ),
                Rotation2.fromDegrees(90)
        );
        turnPath.subdivide(SUBDIVIDE_ITERATIONS);
        turnTrajectory = new Trajectory(turnPath, constraints);

        Path helloPath = new Path(Rotation2.ZERO);
        helloPath.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(-200, 0)
                )
        );
        helloPath.subdivide(SUBDIVIDE_ITERATIONS);
        helloTrajectory = new Trajectory(helloPath, constraints);


        Path helloArcPath = new Path(Rotation2.ZERO);
        helloArcPath.addSegment(
                new PathArcSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(50, 50), 
                        new Vector2(50, 0)
                )
        );
        helloArcTrajectory = new Trajectory(helloArcPath, constraints);

        Path helloArcReversePath = new Path(Rotation2.ZERO);
        helloArcReversePath.addSegment(
                new PathArcSegment(
                        new Vector2(50, 50),
                        new Vector2(0, 0),
                        new Vector2(50,0)
                )
        );
        helloArcReverseTrajectory = new Trajectory(helloArcPath, constraints);

        Path sixthPath = new Path(Rotation2.ZERO);
        sixthPath.addSegment(
                new PathArcSegment(
                        new Vector2(0,0),
                        new Vector2(43.4,25),
                        new Vector2(50,0)
                )
        );
        sixthTrajectory = new Trajectory(sixthPath, constraints);

        Path loadingBayToShootingPath = new Path(Rotation2.ZERO);
        loadingBayToShootingPath.addSegment(
                new PathArcSegment(
                        new Vector2(0, 0),//0,0
                        new Vector2(70, 70),//-50, -86.6
                        new Vector2(70, 0)//50, -100+13.4
                ),
                Rotation2.fromDegrees(0)
        );
        loadingBayToShootingTrajectory = new Trajectory(loadingBayToShootingPath, constraints);

        Path shootingToTrenchPickupPath = new Path(Rotation2.ZERO);
        shootingToTrenchPickupPath.addSegment(
                new PathLineSegment(
                        new Vector2(0,0),
                        new Vector2(-87, 200)
                )
        );
        shootingToTrenchPickupTrajectory = new Trajectory(shootingToTrenchPickupPath, constraints);
    }
       

    public Trajectory getTurnTrajectory() {
        return turnTrajectory;
    }

    public Trajectory getHelloTrajectory() {
        return helloTrajectory;
    }

    public Trajectory getHelloArcTrajectory() {
        return helloArcTrajectory;
    }

    public Trajectory getHelloArcReverseTrajectory() {
            return helloArcReverseTrajectory;
    }

    public Trajectory getLoadingBayToShootingTrajectory() {
        return loadingBayToShootingTrajectory;
    }

    public Trajectory getSixthTrajectory() {
            return sixthTrajectory;
    }

    public Trajectory getShootingToTrenchPickupTrajectory() {
            return shootingToTrenchPickupTrajectory;
    }
}