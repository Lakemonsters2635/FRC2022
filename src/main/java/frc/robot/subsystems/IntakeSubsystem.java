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
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

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

  private final static I2C.Port i2cPort = I2C.Port.kOnboard;
  private final static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  public enum IntakePosition{
    Raised, // retracted
    Middle,
    Lowered // extended
  }

  public IntakeSubsystem() {

    m_ph = new PneumaticHub(PH_CAN_ID);
    intakeSweeperMotor = new CANSparkMax(RobotMap.INTAKE_SWEEPER_MOTOR, MotorType.kBrushless);
    // intakeKickerMotor = new CANSparkMax(RobotMap.INTAKE_KICKER_MOTOR, MotorType.kBrushless); // from 2021 code, not used in 2022
    // intakeKickerMotor.setIdleMode(IdleMode.kBrake);
    intakeSweeperMotor.setIdleMode(IdleMode.kBrake);

    frontSolenoid = m_ph.makeDoubleSolenoid(RobotMap.FRONT_PISTON_BLOCKED, RobotMap.FRONT_PISTON_UNBLOCKED);
    backSolenoid = m_ph.makeDoubleSolenoid(RobotMap.BACK_PISTON_BLOCKED, RobotMap.BACK_PISTON_UNBLOCKED);
    // frontSolenoid.
    

    // frontSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.FRONT_PISTON_BLOCKED, RobotMap.FRONT_PISTON_UNBLOCKED);
    // backSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.BACK_PISTON_BLOCKED, RobotMap.BACK_PISTON_UNBLOCKED);
    // raiseIntake();
    // extender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,6,7);
    // extender = new DoubleSolenoid(4,5);

  }

  public void midState() {
    // System.out.println("mid state");
    backSolenoid.set(Value.kForward);
    frontSolenoid.set(Value.kForward);
  }


  public void extendIntake() {
    // System.out.println("extend intake");
    //System.out.println("forward channel front: " + frontSolenoid.getFwdChannel());
    //System.out.println("forward channel back: " + backSolenoid.getFwdChannel());

    backSolenoid.set(Value.kReverse); // "forward" is 15 open
    frontSolenoid.set(Value.kForward); // "reverse" is 7 closed
    
  }

  public void retractIntake() {
    // System.out.println("retract intake");

    backSolenoid.set(Value.kForward);
    frontSolenoid.set(Value.kReverse);

  }


  public void robotDisable() {
    
  }
  
  public void neutralIntake() {

  }

  public static boolean isCargoIn() {
    // based off whether sensor is over the yellow plate 
    int distanceThreshold = 200; // this is in native color sensor units with a max of 2047 (?) and LARGER units = CLOSER to sensor
    int currentDistance = m_colorSensor.getProximity();
   // System.out.println(currentDistance);
    if (currentDistance >= distanceThreshold) { // TODO revisit < 1000 condition
      // System.out.println("target found in isCargoIn()");
      return true;
    }
    return false;
  }

    // extender.get();
  // }
// 
  // public void lowerIntake() {
  //   solenoid1.set(false);
  //   solenoid2.set(true);  
  // }

  // public double getKickerCurrent() {
  //   return Math.abs(intakeKickerMotor.getOutputCurrent());
  // }
  
  public void setIntakeMotor(double input) { 

    // System.out.println("intake motor spinning: " + input);// spin intake
    intakeSweeperMotor.set(input);
  }

  // public void setKickerMotor(double input){
  //   intakeKickerMotor.set(-input);
  // }
 
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
