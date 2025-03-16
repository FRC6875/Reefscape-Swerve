// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.MechanismConstants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LaserSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionTeleopElevator extends Command {
  /** Creates a new PositionTeleopElevator. */
  ElevatorSubsystem m_elevatorSubsystem;
  LaserSubsystem m_laserSubsystem;
  double m_dist;
  double m_laserDist;
  String m_direction;
  SparkMax elevatorMotor = new SparkMax(ElevatorConstants.kElevatorPort, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder(); //42
  SparkClosedLoopController elevatorController = elevatorMotor.getClosedLoopController();


  
  public PositionTeleopElevator(ElevatorSubsystem elevatorSubsystem, LaserSubsystem laserSubsystem,double dist, double laserDist, String direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_laserSubsystem = laserSubsystem;
    m_dist = dist;
    m_laserDist = laserDist;
    m_direction = direction;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_elevatorSubsystem.runToPosition(m_dist);
  // m_elevatorSubsystem.moveToPosition(m_dist);
  double kP = 0.1; // Proportional constant, adjust as needed
  double tolerance = 0.2; // Allowable error margin

  double error = m_dist - elevatorEncoder.getPosition();//diff between current position and target
  double speed = kP * error; // Calculate speed based on error

  speed = Math.max(-0.5, Math.min(0.5, speed)); // Clamp speed between -0.5 and 0.5

  if (Math.abs(error) > tolerance) {
      elevatorMotor.set(speed); // Move the motor
  } else {
      elevatorMotor.stopMotor(); // Stop if within tolerance
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

 //if(m_laserSubsystem.getValue()>=m_laserDist)return true;
  //  else
    return false;
  }
}
