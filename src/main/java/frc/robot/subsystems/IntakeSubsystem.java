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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    .p(5)
    .i(0)
    .d(0.35)
    .outputRange(-0.3, 0.3);

    resetEncoder();

  }

  public void moveToPosition(double position) {
  double kP = 0.4; // Proportional constant, adjust as needed

  double error = position - intakeEncoder.getPosition(); //diff between current position and target
  double speed = kP * error; // Calculate speed based on error

  speed = Math.max(-0.2, Math.min(0.2, speed)); // Clamp speed. NOTE: NEGATIVE IS UP AND POSITIVE IS DOWN!!!
  
  intakeMotor.set(speed); // Move the motor
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
    SmartDashboard.putNumber("Intake Positions", getEncoderValue());
    // This method will be called once per scheduler run


  }
}
