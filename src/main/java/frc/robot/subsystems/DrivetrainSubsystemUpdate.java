package frc.robot.subsystems;

// import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import frc.robot.RobotMap;


public class DrivetrainSubsystemUpdate extends SubsystemBase implements UpdateManager.Updatable {
    private static final double TRACKWIDTH = 21.0;
    private static final double WHEELBASE = 25.0;

    private static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-29.9); //95.7
    private static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-145.8+5); //-148 + 180   -326.8
    //private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-173.1); //14
    //private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-131.5); //14
    //private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-12.5); //14
    private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(48.5 + 45); //14
    private static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-24.5); //-336+180    -151.6

    private final SwerveModule frontLeftModule =
            new Mk2SwerveModuleBuilder(new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleEncoder(
                            new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER),
                            FRONT_LEFT_ANGLE_OFFSET_COMPETITION)
                    .angleMotor(
                            new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless))
                    .driveMotor(
                            new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                            Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule frontRightModule =
            new Mk2SwerveModuleBuilder(new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                .angleEncoder(
                    new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER),
                    FRONT_RIGHT_ANGLE_OFFSET_COMPETITION)
                .angleMotor(
                        new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless))
                .driveMotor(
                        new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                        Mk2SwerveModuleBuilder.MotorType.NEO)
                .build();
    private final SwerveModule backLeftModule =
            new Mk2SwerveModuleBuilder(new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                .angleEncoder(
                    new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER),
                    BACK_LEFT_ANGLE_OFFSET_COMPETITION)
                .angleMotor(
                        new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless))
                .driveMotor(
                        new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                        Mk2SwerveModuleBuilder.MotorType.NEO)
                .build();
    private final SwerveModule backRightModule =
            new Mk2SwerveModuleBuilder(new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                .angleEncoder(
                    new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER),
                    BACK_RIGHT_ANGLE_OFFSET_COMPETITION)
                .angleMotor(
                        new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless))
                .driveMotor(
                        new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                        Mk2SwerveModuleBuilder.MotorType.NEO)
                .build();
    private final SwerveModule[] modules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

    private final SwerveKinematics kinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front Left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // Front Right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Back Left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // Back Right
    );
    private final SwerveOdometry odometry = new SwerveOdometry(kinematics, RigidTransform2.ZERO);

    private final Object sensorLock = new Object();
    // @GuardedBy("sensorLock")
    private final NavX navX = new NavX(SPI.Port.kMXP);

    private final Object kinematicsLock = new Object();
    // @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;

    private final Object stateLock = new Object();
    // @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    // Logging stuff
    private NetworkTableEntry poseXEntry;
    private NetworkTableEntry poseYEntry;
    private NetworkTableEntry poseAngleEntry;

    private NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

    public DrivetrainSubsystemUpdate() {
        synchronized (sensorLock) {
            navX.setInverted(true);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        poseXEntry = tab.add("Pose X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        poseYEntry = tab.add("Pose Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        poseAngleEntry = tab.add("Pose Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        ShuffleboardLayout frontLeftModuleContainer = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withPosition(1, 0)
                .withSize(2, 3);
        moduleAngleEntries[0] = frontLeftModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout frontRightModuleContainer = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withPosition(3, 0)
                .withSize(2, 3);
        moduleAngleEntries[1] = frontRightModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout backLeftModuleContainer = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withPosition(5, 0)
                .withSize(2, 3);
        moduleAngleEntries[2] = backLeftModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout backRightModuleContainer = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withPosition(7, 0)
                .withSize(2, 3);
        moduleAngleEntries[3] = backRightModuleContainer.add("Angle", 0.0).getEntry();
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, fieldOriented);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            navX.setAdjustmentAngle(
                    navX.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }

    @Override
    public void update(double timestamp, double dt) {
        updateOdometry(dt);

        HolonomicDriveSignal driveSignal;
        synchronized (stateLock) {
            driveSignal = this.driveSignal;
        }

        updateModules(driveSignal, dt);
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        synchronized (sensorLock) {
            angle = navX.getAngle();
        }

        RigidTransform2 pose = odometry.update(angle, dt, moduleVelocities);

        synchronized (kinematicsLock) {
            this.pose = pose;
        }
    }

    private void updateModules(HolonomicDriveSignal signal, double dt) {
        ChassisVelocity velocity;
        if (signal == null) {
            velocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (signal.isFieldOriented()) {
            velocity = new ChassisVelocity(
                    signal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    signal.getRotation()
            );
        } else {
            velocity = new ChassisVelocity(signal.getTranslation(), signal.getRotation());
        }

        Vector2[] moduleOutputs = kinematics.toModuleVelocities(velocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0);

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i]);
            module.updateState(dt);
        }
    }

    @Override
    public void periodic() {
        var pose = getPose();
        poseXEntry.setDouble(pose.translation.x);
        poseYEntry.setDouble(pose.translation.y);
        poseAngleEntry.setDouble(pose.rotation.toDegrees());

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            moduleAngleEntries[i].setDouble(Math.toDegrees(module.getCurrentAngle()));
        }
    }
}
