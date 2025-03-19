// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.MechanismConstants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  SparkMax intakeMotor = new SparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();

  public IntakeSubsystem() {
   
    config
    //.inverted(true)
    .idleMode(IdleMode.kBrake);
    config.encoder
    .positionConversionFactor(IntakeConstants.kIntakeEncoderConvFact);
    config.closedLoop
    .p(1)
    .i(0)
    .d(0)
    .outputRange(-0.3, 0.3);

    resetEncoder();

  }

  public void moveToPosition(double position) {
  double kP = 0.1; // Proportional constant, adjust as needed
  double tolerance = 0.2; // Allowable error margin

  double error = position - intakeEncoder.getPosition();//diff between current position and target
  double speed = kP * error; // Calculate speed based on error

  speed = Math.max(-0.3, Math.min(0.3, speed)); // Clamp speed between -0.5 and 0.5

  if (Math.abs(error) > tolerance) {
      intakeMotor.set(speed); // Move the motor
  } else {
      intakeMotor.stopMotor(); // Stop if within tolerance

  }
}

 public void resetEncoder(){
    intakeMotor.getEncoder().setPosition(0);
  }
  
  public double getEncoderValue(){
    return intakeMotor.getEncoder().getPosition();
  }

  public void stop(){
    intakeMotor.stopMotor();
  }

  public void setSpeed(double speed) {

      intakeMotor.set(speed);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
