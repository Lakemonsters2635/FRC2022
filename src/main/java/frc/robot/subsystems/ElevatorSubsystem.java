/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  CANSparkMax beltMotor;
 
  CANEncoder beltEncoder;
  CANPIDController beltController;
  
  DigitalInput bottomSensor;
  public int powerCellCount = 0;
  double setpoint;
  public ElevatorSubsystem() {
    beltMotor = new CANSparkMax(RobotMap.ELEVATOR_MOTOR_CHANNEL, MotorType.kBrushless);
    beltMotor.setIdleMode(IdleMode.kBrake);

    beltEncoder = beltMotor.getEncoder();
    
    beltController = beltMotor.getPIDController();

    beltController.setP(0.1);
    beltController.setI(0);
    beltController.setD(0);
  }

  public void setBeltMotor(double input) {
    beltMotor.set(input);
  }
  
  public double getBeltEncoderValue() {
    return beltEncoder.getPosition();
  }

  public void setForIndex(boolean up, double index) {
    if(up) {
    setpoint = beltEncoder.getPosition() - index;
    } else {
      setpoint = beltEncoder.getPosition() + index;
    }
  }

  public void PIDDrive() {
    beltController.setReference(setpoint, ControlType.kPosition);
  }

  public double getSetpoint() {
    return setpoint;
  }
  
  public void shooterLoad() {
    beltMotor.set(1);
  
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
