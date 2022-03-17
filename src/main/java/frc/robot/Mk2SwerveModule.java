/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;


import java.util.concurrent.atomic.AtomicLong;

public class Mk2SwerveModule extends SwerveModule {
    private static final PidConstants ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.000);//0.5, 0.0, 0.0001
    // private static final double DRIVE_TICKS_PER_INCH = (1.0 / (4.0625 * Math.PI / 60.0 * 15.0 / 20.0 * 24.0 / 38.0 * 18.0) / 0.827828) * (200/213.875);
    private static final double DRIVE_TICKS_PER_REVOLUTION_OF_WHEEL =  (1.0 * // ticks per revolution of motor
                                                                        (42.0 / 14.0) * // gear ratio of first stage
                                                                        (18.0 / 26.0) * // gear ratio of second stage
                                                                        (60.0 / 15.0) // gear ratio of third stage
                                                                        );  
    /*
    DRIVE_TICKS_PER_REVOLUTION_OF_WHEEL is calculated as follows
    (ticks per revolution of motor * (gear ratio of 1st stage) * (2nd stage gear ratio) * (3rd stage gear ratio))

    the ticks for revolution of the motor = 42

    ***VERY IMPORTANT MUST READ***
    * DRIVE_TICKS_PER_INCH is actually MOTOR REVOLUTIONS per inch

    * that's why DRIVE_TICKS_PER_REVOLUTION_OF_WHEEL has 1.0 as one of the numerators instead of 42, which is the external encoder ticks per revolution of the motor

    depending on the module there may be more or fewer stages, you want to multiply by overall gear ratio
    */
    private static final double WHEEL_DIAMETER = 4.0625; // inches 
    private static final double CARPET_FACTOR_LHS_FWING = 96.0 / 91.5; // fraction to account for wheel/carpet interation. Tell the robot to drive x amount then measure true value.
    
    private static final double DRIVE_TICKS_PER_INCH = (DRIVE_TICKS_PER_REVOLUTION_OF_WHEEL / (WHEEL_DIAMETER * Math.PI)) * CARPET_FACTOR_LHS_FWING;

    // changed from 60.0 * 15.0 / 20.0 * 24.0 / 38.0 * 18.0 to 60.0 * 15.0 / 28.0 * 26.0 / 42.0 * 14.0
    private static final double CAN_UPDATE_RATE = 50.0;

    private final double angleOffset;

    private CANSparkMax angleMotor;
    private AnalogInput angleEncoder;
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    private final Object canLock = new Object();
    private double driveEncoderTicks = 0.0;
    private double drivePercentOutput = 0.0;
    private double driveVelocityRpm = 0.0;
    private double driveCurrent = 0.0;

    private Notifier canUpdateNotifier = new Notifier(() -> {
        double driveEncoderTicks = driveEncoder.getPosition();
        synchronized (canLock) {
            Mk2SwerveModule.this.driveEncoderTicks = driveEncoderTicks;
        }

        double driveVelocityRpm = driveEncoder.getVelocity();
        synchronized (canLock) {
            Mk2SwerveModule.this.driveVelocityRpm = driveVelocityRpm;
        }

        double localDriveCurrent = driveMotor.getOutputCurrent();
        synchronized (canLock) {
            driveCurrent = localDriveCurrent;
        }

        double drivePercentOutput;
        synchronized (canLock) {
            drivePercentOutput = Mk2SwerveModule.this.drivePercentOutput;
        }
        driveMotor.set(drivePercentOutput);
        //if(getName() == "Front Left") System.out.println(getName() + " drivePercentOutput: " + drivePercentOutput);
    });

    private PidController angleController = new PidController(ANGLE_CONSTANTS);

    public Mk2SwerveModule(Vector2 modulePosition, double angleOffset,
                           CANSparkMax angleMotor, CANSparkMax driveMotor, AnalogInput angleEncoder) {
        super(modulePosition);  
        this.angleOffset = angleOffset;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.driveMotor = driveMotor;
        this.driveEncoder = driveMotor.getEncoder();
        //this.driveCurrent = CanSparkMax.
        driveMotor.setIdleMode(IdleMode.kBrake); // this UNCOMMENTED makes robot stop abruptly - use to save Frank's walls
        //driveMotor.setIdleMode(IdleMode.kCoast); // this COMMENTED makes robot stop abruptly - use to save Frank's walls
        driveMotor.setSmartCurrentLimit(60);

        angleController.setInputRange(0.0, 2.0 * Math.PI);
        angleController.setContinuous(true);
        angleController.setOutputRange(-0.5, 0.5);

        canUpdateNotifier.startPeriodic(1.0 / CAN_UPDATE_RATE);

        // System.out.println("DRIVE_TICKS_PER_INCH = " + DRIVE_TICKS_PER_INCH);
    }

    @Override
    protected double readAngle() {
        double angle = (1.0 - angleEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + angleOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        // SmartDashboard.putNumber(getName(), Math.toDegrees((1.0 - angleEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI) + angleOffset);


        return angle;
    }

    @Override
    protected double readDistance() {
        double driveEncoderTicks;
        synchronized (canLock) {
            driveEncoderTicks = this.driveEncoderTicks;
        }

        return driveEncoderTicks / DRIVE_TICKS_PER_INCH;
    }

    protected double readVelocity() {
        double driveVelocityRpm;
        synchronized (canLock) {
            driveVelocityRpm = this.driveVelocityRpm;
        }

        return driveVelocityRpm * (1.0 / 60.0) / DRIVE_TICKS_PER_INCH;
    }

    protected double readDriveCurrent() {
        double localDriveCurrent;
        synchronized (canLock) {
            localDriveCurrent = driveCurrent;
        }

        return localDriveCurrent;
    }

    @Override
    public double getCurrentVelocity() {
        return readVelocity();
    }

    @Override
    public double getDriveCurrent() {
        return readDriveCurrent();
    }

    @Override
    protected void setTargetAngle(double angle) {
        angleController.setSetpoint(angle);
    }

    @Override
    protected void setDriveOutput(double output) {
        synchronized (canLock) {
            this.drivePercentOutput = output;
        }
    }

    @Override
    public void updateState(double dt) {
        super.updateState(dt);

        angleMotor.set(angleController.calculate(getCurrentAngle(), dt));
    }
}
