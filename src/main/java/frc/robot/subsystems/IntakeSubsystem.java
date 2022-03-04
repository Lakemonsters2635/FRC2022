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
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeCommand;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final int PH_CAN_ID = 15;
  PneumaticHub m_ph;
  CANSparkMax intakeSweeperMotor;
  CANSparkMax intakeKickerMotor;
  DoubleSolenoid frontSolenoid;
  DoubleSolenoid backSolenoid;

public enum IntakePosition{
  Raised, // retracted
  Middle,
  Lowered // extended
}
  public IntakeSubsystem() {

    m_ph = new PneumaticHub(PH_CAN_ID);

    intakeSweeperMotor = new CANSparkMax(RobotMap.INTAKE_SWEEPER_MOTOR, MotorType.kBrushless);
    // intakeKickerMotor = new CANSparkMax(RobotMap.INTAKE_KICKER_MOTOR, MotorType.kBrushless);
    // intakeKickerMotor.setIdleMode(IdleMode.kBrake);
    intakeSweeperMotor.setIdleMode(IdleMode.kBrake);

    frontSolenoid = m_ph.makeDoubleSolenoid(RobotMap.FRONT_PISTON_BLOCKED, RobotMap.FRONT_PISTON_UNBLOCKED);
    backSolenoid = m_ph.makeDoubleSolenoid(RobotMap.BACK_PISTON_BLOCKED, RobotMap.BACK_PISTON_UNBLOCKED);

    

    // frontSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.FRONT_PISTON_BLOCKED, RobotMap.FRONT_PISTON_UNBLOCKED);
    // backSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.BACK_PISTON_BLOCKED, RobotMap.BACK_PISTON_UNBLOCKED);
    // raiseIntake();
    // extender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,6,7);
    // extender = new DoubleSolenoid(4,5);

  }

  // public void midState() {
  //   solenoid1.set(false);
  //   solenoid2.set(false);
  //   }
  public void extendIntake() {
    System.out.println("extend intake");
    //System.out.println("forward channel front: " + frontSolenoid.getFwdChannel());
    //System.out.println("forward channel back: " + backSolenoid.getFwdChannel());

    frontSolenoid.set(Value.kForward); // "reverse" is 7 closed
    backSolenoid.set(Value.kReverse);  // "forward" is 15 open
  }

  public void retractIntake() {
    System.out.println("retract intake");

    frontSolenoid.set(Value.kReverse);
    backSolenoid.set(Value.kForward);

  }

  public void neutralIntake() {

  }
    // extender.get();
  // }
// 
  // public void lowerIntake() {
  //   solenoid1.set(false);
  //   solenoid2.set(true);  
  // }

  public double getKickerCurrent() {
    return Math.abs(intakeKickerMotor.getOutputCurrent());
  }
  
  public void setIntakeMotor(double input) { // spin intake
    intakeSweeperMotor.set(input);
  }

  public void setKickerMotor(double input){
    intakeKickerMotor.set(-input);
  }
 
  public boolean intakeIsExtended() {
    // Value extenderValue = extender.get();
    // if ( extenderValue == Value.kForward) {
    //   return true;
    // }
    return false;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
