/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public Joystick rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK_CHANNEL);
  public Joystick leftStick = new Joystick(RobotMap.LEFT_JOYSTICK_CHANNEL);

  public Button referenceResetButton = new JoystickButton(leftStick, RobotMap.L_REFERENCE_RESET_BUTTON);

  Button shooterVisionButton = new JoystickButton(leftStick, RobotMap.L_SHOOTER_WITHVISION_BUTTON);
  Button shooterNoVisionButton = new JoystickButton(rightStick, RobotMap.R_SHOOTER_NOVISION_BUTTON);
  Button shooterReverseButton = new JoystickButton(rightStick, RobotMap.R_SHOOTER_REVERSE_BUTTON);
 
  Button visionDriveButton = new JoystickButton(leftStick, RobotMap.L_VISION_BUTTON);
  
  // Button intakeDetectButton = new JoystickButton(leftStick, RobotMap.L_INTAKE_BUTTON);
  Button intakeInButton = new JoystickButton(leftStick, RobotMap.L_INTAKE_BUTTON);
  Button intakeOutButton = new JoystickButton(leftStick, RobotMap.L_INTAKE_REVERSE_BUTTON);
  // Button intakeElevationButton = new JoystickButton(leftStick, RobotMap.L_INTAKE_ELEVATION_BUTTON);

  // Button intakeActuateMiddleButton = new JoystickButton(leftStick, RobotMap.L_INTAKE_ACTUATE_MIDDLE_BUTTON);
  Button intakeActuateUpButton = new JoystickButton(rightStick, RobotMap.R_INTAKE_EXTEND_BUTTON);
  Button intakeActuateDownButton = new JoystickButton(rightStick, RobotMap.R_INTAKE_RETRACT_BUTTON);
  Button intakeMidStateButton = new JoystickButton(leftStick, RobotMap.L_INTAKE_MID_STATE_BUTTON);

  // Button indexZoneButton = new JoystickButton(rightStick, RobotMap.R_INDEX_ZONE_BUTTON);
  // Button shooterIdleButton = new JoystickButton(rightStick, RobotMap.R_SHOOTER_IDLE_BUTTON); 
  // Button shooterZERORPMButton = new JoystickButton(rightStick, RobotMap.R_SHOOTER_ZERO_RPM_BUTTON);
  // climber extend is left 6
  // climber retract is left 7
  Button extendClimberButton = new JoystickButton(rightStick, RobotMap.R_EXTEND_CLIMBER_BUTTON);
  Button retractClimberButton = new JoystickButton(rightStick, RobotMap.R_RETRACT_CLIMBER_BUTTON);

}
