// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopElevator extends Command {
 ElevatorSubsystem m_elevatorSubsystem;
  double speed;
  String m_direction;
  DoubleSupplier up,down;

  public TeleopElevator(ElevatorSubsystem elevatorSubsystem, DoubleSupplier up,DoubleSupplier down) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    this.up=up;
    this.down=down;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //reset encoders
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed=up.getAsDouble()-down.getAsDouble();

    m_elevatorSubsystem.setSpeed(speed);
    //run motor in specified direction
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //check if encoder value is greater than or equal to distance inputted
    
    //without tolerance
   return false;
    // 2 is the tolerance
//    if(Math.abs(m_elevatorSubsystem.getEncoderValue() - m_dist) < 0.5) return true;
//    else return false;
 }
}