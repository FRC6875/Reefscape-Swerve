// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.MechanismConstants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  SparkMax climbMotor = new SparkMax(ClimbConstants.kClimbPort, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  RelativeEncoder climbEncoder = climbMotor.getEncoder(); //42
  

  public ClimbSubsystem() {

      config
      //.inverted(true)
      .idleMode(IdleMode.kBrake);
      config.encoder
      .positionConversionFactor(ClimbConstants.kClimbEncoderConvFact);
      config.closedLoop
      .p(1.0)
      .i(0)
      .d(0)//?
      .outputRange(-0.3, 0.3);
  
      resetEncoder();
    

  }
  public void resetEncoder(){
    climbMotor.getEncoder().setPosition(0);
  }

  public void setSpeed(double speed) {

      climbMotor.set(speed);
    }

    public void stop(){
      climbMotor.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
