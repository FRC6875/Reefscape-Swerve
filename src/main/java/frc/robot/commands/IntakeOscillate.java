// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeOscillate extends Command {
  IntakeSubsystem m_intakeSubsystem;
  double m_posi;
  double m_resistence;
  /** Creates a new IntakeOscillate. */
  public IntakeOscillate(IntakeSubsystem intakeSubsystem, double resistence) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = intakeSubsystem;
    m_resistence = resistence;
    addRequirements(RobotContainer.m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_posi = m_intakeSubsystem.getEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intakeSubsystem.getEncoderValue() > m_posi + 0.05){
      m_intakeSubsystem.setSpeed(-m_resistence);
    }
    else if(m_intakeSubsystem.getEncoderValue() < m_posi - 0.05){
      m_intakeSubsystem.setSpeed(m_resistence*0.5);
    }
    
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
