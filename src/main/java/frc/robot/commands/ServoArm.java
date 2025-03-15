
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.generated.MechanismConstants.ServoConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ServoArm extends Command {
  /** Creates a new ServoArm. */
  ServoSubsystem m_servoSubsystem;
  double m_angle;
  public ServoArm(ServoSubsystem servoSubsystem, double angle) {
    m_angle=angle;
    m_servoSubsystem = servoSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_servoSubsystem.setAngle(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_servoSubsystem.setAngle(m_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
