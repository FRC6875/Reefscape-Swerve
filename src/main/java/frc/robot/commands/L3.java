// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeWheelsSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3 extends ParallelCommandGroup {
  /** Creates a new IntakeWheels. */
  IntakeWheelsSubsystem m_intakeWheelsSubsystem;
  double speed;

  public L3(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, IntakeWheelsSubsystem intakeWheelsSubsystem, DoubleSupplier elevatorPosition, DoubleSupplier holderPosition) {
    addCommands(
      new PositionTeleopElevator(elevatorSubsystem, elevatorPosition),
      new PositionIntake(intakeSubsystem, holderPosition),
      new IntakeWheels(intakeWheelsSubsystem, 0.4)
    );
  }
}
