/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ShooterSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonFX motor1;

  // DoubleSolenoid rightSolenoid;
  public ShooterSubsystem(){
    motor1 = new TalonFX(RobotMap.SHOOTER_BOTTOM_CAN);
    
    configureMotors();  
  }


  public void configureMotors(){
      motor1.setSensorPhase(true);

      motor1.configNominalOutputForward(0, RobotMap.kTimeoutMs);
      motor1.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
      motor1.configPeakOutputForward(1, RobotMap.kTimeoutMs);
      motor1.configPeakOutputReverse(-1, RobotMap.kTimeoutMs);

      motor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                              RobotMap.kPIDLoopIdx, 
                                              RobotMap.kTimeoutMs);

      motor1.config_kF(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kF, RobotMap.kTimeoutMs);
      motor1.config_kP(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kP, RobotMap.kTimeoutMs);
      motor1.config_kI(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kI, RobotMap.kTimeoutMs);
      motor1.config_kD(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kD, RobotMap.kTimeoutMs);
  }

  public void update() {
    // FireLog.log("colorwheelpos", drive.getEncoder().getPosition());
  }

  public void stop() {
    // 2/21 not sure if setting the PID values back to zero in this function will reduce the oscillations present in deceleration of shooter wheelss

    // motor1.config_kF(RobotMap.kPIDLoopIdx, 0, RobotMap.kTimeoutMs);
    // motor1.config_kP(RobotMap.kPIDLoopIdx, 0, RobotMap.kTimeoutMs);
    // motor1.config_kI(RobotMap.kPIDLoopIdx, 0, RobotMap.kTimeoutMs);
    // motor1.config_kD(RobotMap.kPIDLoopIdx, 0, RobotMap.kTimeoutMs);
    

    motor1.setNeutralMode(NeutralMode.Coast); 
    motor1.set(ControlMode.Velocity, 0);
  
  }

  public void Init() {

  }

  public void SpinShooter(double motorSpeed) {
    //DON'T GO OVER 8,000;
    motorSpeed = Math.min(10000, motorSpeed); // was Math.abs(motorSpeed) 3/10
    
   // double lowerMotorSpeed = upperMotorSpeed * 3;

    SmartDashboard.putNumber("shooter speed", motorSpeed);
    // SmartDashboard.putNumber("lower", lowerMotorSpeed);
    motor1.set(ControlMode.Velocity, motorSpeed*2048/600);

  }
  
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
