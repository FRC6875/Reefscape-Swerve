// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LaserSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionTeleopElevator extends Command {
  /** Creates a new PositionTeleopElevator. */
  ElevatorSubsystem m_elevatorSubsystem;
  DoubleSupplier m_dist;//(code that gives a double)
  String m_direction;
  
  public PositionTeleopElevator(ElevatorSubsystem elevatorSubsystem, DoubleSupplier dist) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_dist = dist;
    //m_direction = direction;
    addRequirements(RobotContainer.m_elevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_elevatorSubsystem.runToPosition(m_dist);
   m_elevatorSubsystem.moveToPosition(m_dist.getAsDouble() * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if the encoder value reaches the target,stop
     if (Math.abs(m_elevatorSubsystem.getEncoderValue()-m_dist.getAsDouble())<=0.3) return true;
    //else if the laser value reaches the target,stop
   // else if(m_laserSubsystem.getValue()>=m_laserDist)return true;

    return false;
  }
}
