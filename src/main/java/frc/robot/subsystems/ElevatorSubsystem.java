// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.generated.MechanismConstants;
import frc.robot.generated.MechanismConstants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;

public class ElevatorSubsystem extends SubsystemBase {
  SparkMax elevatorMotor = new SparkMax(ElevatorConstants.kElevatorPort, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  //RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder(Type.kHallSensor, 42);
  


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    config
    //.inverted(true)
    .idleMode(IdleMode.kBrake);
    config.encoder
    .positionConversionFactor(ElevatorConstants.kElevatorEncoderConvFact);
    // config.closedLoop
    // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // .pid(1.0, 0.0, 0.0);

  }


  public void resetEncoder(){
    elevatorMotor.getEncoder().setPosition(0);
  }
  public void Stop(){
    elevatorMotor.stopMotor();
  }

  public void move(int speed) {
elevatorMotor.set(speed);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
