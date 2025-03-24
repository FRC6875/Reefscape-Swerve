// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAuto extends ParallelCommandGroup {
  

  public RunAuto(IntakeSubsystem intakeSubsystem, DoubleSupplier holderPosition, Command auto) {
    addCommands(
      new PositionIntake(intakeSubsystem, holderPosition),
      auto
    );
  }
}
