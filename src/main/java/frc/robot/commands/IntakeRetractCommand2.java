// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetractCommand2 extends Command {
  /** Creates a new IntakeExtendCommand. */
  private boolean m_isInMidPos; 
  private boolean turnOnMidState; 

  public IntakeRetractCommand2(double timeOut) {
    // Use addRequirements() here to declare subsystem dependencies.
    // super(timeOut);
    super(timeOut); 
    turnOnMidState = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isInMidPos = IntakeSubsystem.isCargoIn(); 
    Robot.intakeSubsystem.retractIntake(); 
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("IntakeRetractCommand2 execute");
    // if (!m_isInMidPos) {
      boolean currentlyDetected = Robot.intakeSubsystem.isCargoIn();
      
       System.out.println("!m_isInMidPos + " + currentlyDetected);

      if (!m_isInMidPos && currentlyDetected != m_isInMidPos) {
        System.out.println("Cargo/yellow plate detected"); 
        // Robot.intakeSubsystem.midState();
        m_isInMidPos = currentlyDetected;
        turnOnMidState = true;
      }
    // }    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    // System.out.println("mid state reached end function");
    Robot.intakeSubsystem.midState();
    turnOnMidState = false;
    m_isInMidPos = false; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnOnMidState || super.isTimedOut();
  }
}
