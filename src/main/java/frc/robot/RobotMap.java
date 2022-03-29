/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.models.Gains;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static final boolean IS_2022 = true;
  public static final int LEFT_JOYSTICK_CHANNEL = 0;
  public static final int RIGHT_JOYSTICK_CHANNEL = 1;

  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 8; 
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = IS_2022 ? 0 : 3; 
  public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 7; 

  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 4; 
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = IS_2022 ? 3 : 2;
  public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 3; 
  
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 6; 
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 1;
  public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 5; 

  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 2;
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = IS_2022 ? 2 : 0;
  public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 1; 

  public static final int INTAKE_SWEEPER_MOTOR = 14;
  // public static final int INTAKE_KICKER_MOTOR = 16; // from 2021 code, not used in 2022 

  // public static final int SHOOTER_TOP_CAN = 12;
  public static final int SHOOTER_BOTTOM_CAN = 13;

  public static final int UPPER_KICKER_MOTOR = 11;
  public static final int ELEVATOR_MOTOR_CHANNEL = 10;
//----------------------------------------------------------------------

  public static final int L_SHOOTER_WITHVISION_BUTTON = 1;
  public static final int L_INTAKE_BUTTON = 2;
  public static final int L_VISION_BUTTON = 4;
  public static final int L_REFERENCE_RESET_BUTTON = 8;
  public static final int L_INTAKE_REVERSE_BUTTON = 3;
  // public static final int L_INTAKE_ACTUATE_UP_BUTTON = 6;
  // public static final int L_INTAKE_ACTUATE_MIDDLE_BUTTON = 9;
  // public static final int L_INTAKE_ACTUATE_DOWN_BUTTON = 7;
  public static final int L_INTAKE_MID_STATE_BUTTON = 9;


  public static final int R_SHOOTER_NOVISION_BUTTON = 1;

  public static final int R_INTAKE_EXTEND_BUTTON = 2;
  public static final int R_INTAKE_RETRACT_BUTTON = 3;
  public static final int R_SHOOTER_REVERSE_BUTTON = 11;
  // public static final int R_ELEVATOR_INDEX_UP_BUTTON = 3;
  public static final int R_ELEVATOR_DOWN_BUTTON = 5;
  public static final int R_ELEVATOR_UP_BUTTON = 4;

  public static final int R_INTAKE_IN_BUTTON = 10;
  public static final int R_INTAKE_OUT_BUTTON = 9;
  // public static final int R_INDEX_ZONE_BUTTON = 6;

  // public static final int R_SHOOTER_IDLE_BUTTON = 7;
  // public static final int R_SHOOTER_ZERO_RPM_BUTTON = 8;
  public static final int R_EXTEND_CLIMBER_BUTTON = 6;
  public static final int R_RETRACT_CLIMBER_BUTTON = 7;


  
  public static final int BACK_PISTON_BLOCKED = 14;
  public static final int BACK_PISTON_UNBLOCKED = 15;

  public static final int FRONT_PISTON_BLOCKED = 6;
  public static final int FRONT_PISTON_UNBLOCKED = 7;
  public static final int CLIMBER_FORWARD_CHANNEL = 0;
  public static final int CLIMBER_REVERSE_CHANNEL = 1;



//90.7
  public static final double TARGET_HEIGHT =81.25-38.2; //height to center target - camera height
  //public static final double CAMERA_ANGLE = -1*Math.atan(TARGET_HEIGHT/183);
  public static final double CAMERA_ANGLE = Math.toRadians(26);

  public static final double TARGET_TRIGGER_DISTANCE = 40; 

  public static double OBJECT_DETECTION_LATENCY = 0.217; // seconds
  /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   kI   kD   kF          Iz    PeakOut */
 	//public final static Gains kGains_Velocit = new Gains( 0.25, 0.001, 20, 1023.0/7200.0,  300,  1.00);
	 static final double kP = 0.04; 	// 0.05 2022
	 static final double kI = 0.54*0.83/200; 	// 0.002241
	 static final double kD = 0.003; 	// 0.22636364
	 static final double kF = 0.05086878976; 			// previously 0.047 on 2021 bot, tuned 2/21
	 //public final static Gains kGains_Velocit = new Gains( kP, kI, kD, kF,  0,  1.00);

   // new PID values added 2/21 for upper shooter wheel (talon ID 12) on 2021 bot 
  //  static final double kP2 = 0.4125;
   
	//  public final static Gains kGains_Velocit = new Gains(kP2, 0.0, 0.0, 0,  0,  1.00); // before 2/21 we were using kF = 0.047
  public final static Gains kGains_Velocit = new Gains(kP, 0.0, 0.0, kF,  0,  1.00);

   public final static double SHOOTER_MOTOR_HIGH_DEFAULT_SPEED = 2000; // can go up to 6000, 10000+ in phoenix tuner
   public final static double SHOOTER_MOTOR_LOW_DEFAULT_SPEED = 3100;
   public final static double SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED = 2200; //1742 to 500 on 2/10/22


  
}
