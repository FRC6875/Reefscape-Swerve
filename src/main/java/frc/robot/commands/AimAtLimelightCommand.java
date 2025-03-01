// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtLimelightCommand extends Command {
  /** Creates a new AimAtLimelightCommand. */

  private SwerveSubsystem m_swerveSubsystem;
  private PIDController m_pidController;
  private final String limelightName="limelight";
  private final int targetId=1;

  public AimAtLimelightCommand(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem=swerveSubsystem;
    m_pidController = new PIDController(0.1, 0, 0);
    m_pidController.setTolerance(2);
    m_pidController.setSetpoint(0);
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(LimelightHelpers.getFiducialID(limelightName)==-1){
      m_swerveSubsystem.driveFieldOrientated(0);//ask(set drive speed to 0 if don't see a apriltag)

    }else{
      double response = m_pidController.calculate(LimelightHelpers.getTX(limelightName));
      m_swerveSubsystem.driveFieldOrientated(response);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
m_swerveSubsystem.driveFieldOrientated(0);//setSpeed

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}
