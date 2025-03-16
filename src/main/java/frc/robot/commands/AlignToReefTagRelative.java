// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.MechanismConstants.LimeLightConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReefTagRelative extends Command {
  /** Creates a new AimAtLimelightCommand. */

  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem m_swerveSubsystem;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem swerveSubsystem) {
    xController = new PIDController(LimeLightConstants.kXReefAlignmentP, 0.0, 0);  // Vertical movement
    yController = new PIDController(LimeLightConstants.kYReefAlignmentP, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(LimeLightConstants.kRotReefAlignmentP, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(LimeLightConstants.kRotSetpointReefAlignment);
    rotController.setTolerance(LimeLightConstants.kRotToleranceReefAlignment);

    xController.setSetpoint(LimeLightConstants.kXSetpointReefAlignment);
    xController.setTolerance(LimeLightConstants.kXToleranceReefAlignment);

    yController.setSetpoint(isRightScore ? LimeLightConstants.kYSetpointReefAlignment : -LimeLightConstants.kYSetpointReefAlignment);
    yController.setTolerance(LimeLightConstants.kYToleranceReefAlignment);

     tagID = LimelightHelpers.getFiducialID("");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");

      double xSpeed = xController.calculate(postions[2]);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);
      
     m_swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);


      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      m_swerveSubsystem.drive(new Translation2d(), 0, false);
    }
   

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_swerveSubsystem.drive(new Translation2d(), 0, false);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

 return this.dontSeeTagTimer.hasElapsed(LimeLightConstants.kDontSeeTagWaitTime) ||
        stopTimer.hasElapsed(LimeLightConstants.kPoseValidationTime);
        }
}
