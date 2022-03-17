// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetractCommand extends Command {
  /** Creates a new IntakeExtendCommand. */
  private boolean m_isInMidPos; 
  private boolean enableMidState; 

  public IntakeRetractCommand(double timeOut, boolean enableMidState) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(timeOut); 
    this.enableMidState = enableMidState; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isInMidPos = IntakeSubsystem.isCargoIn(); 
    
    if (enableMidState) {
      Robot.intakeSubsystem.midState();
    } else {
      Robot.intakeSubsystem.retractIntake();

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_isInMidPos) {
      boolean currentlyDetected = Robot.intakeSubsystem.isCargoIn();
    
      if (!m_isInMidPos && currentlyDetected != m_isInMidPos) {
        // System.out.println("Cargo detected"); 
        // Robot.intakeSubsystem.midState();
        m_isInMidPos = currentlyDetected;
      }
    }    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
