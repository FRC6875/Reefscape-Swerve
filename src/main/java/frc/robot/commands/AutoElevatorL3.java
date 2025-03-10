// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoElevatorL3 extends Command {
  ElevatorSubsystem m_elevatorSubsystem;
  double m_dist;
  String m_direction;
  /** Creates a new AutoElevatorTrough. */
  public AutoElevatorL3(ElevatorSubsystem elevatorSubsystem, double dist, String direction) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_dist = dist;
    m_direction = direction;

  addRequirements(RobotContainer.m_elevatorSubsystem);
  }
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_elevatorSubsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setSpeed(0.3);
    m_elevatorSubsystem.runToPosition(7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}