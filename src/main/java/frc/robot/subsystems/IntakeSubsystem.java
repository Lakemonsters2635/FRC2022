/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeCommand;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  CANSparkMax intakeSweeperMotor;
  CANSparkMax intakeKickerMotor;
  DoubleSolenoid extender;


  public IntakeSubsystem() {
    intakeSweeperMotor = new CANSparkMax(RobotMap.INTAKE_SWEEPER_MOTOR, MotorType.kBrushless);
    intakeKickerMotor = new CANSparkMax(RobotMap.INTAKE_KICKER_MOTOR, MotorType.kBrushless);
    intakeKickerMotor.setIdleMode(IdleMode.kBrake);
    intakeSweeperMotor.setIdleMode(IdleMode.kBrake);


    extender = new DoubleSolenoid(6,7);
    //extender = new DoubleSolenoid(4,5);

  }


  public void raiseIntake() {
    extender.set(Value.kReverse);
  }

  public void lowerIntake() {
    extender.set(Value.kForward);
  }

  public double getKickerCurrent() {
    return Math.abs(intakeKickerMotor.getOutputCurrent());
  }
  
  public void setIntakeMotor(double input) {
    intakeSweeperMotor.set(input);
  }

  public void setKickerMotor(double input){
    intakeKickerMotor.set(-input);
  }
 


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
