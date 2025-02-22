// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//gracescommet

// Natalie's test comment
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.MechanismConstants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  SparkMax elevatorMotor = new SparkMax(ElevatorConstants.kElevatorPort, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder(); //42
  


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
  
  public double getEncoderValue(){
    return elevatorMotor.getEncoder().getPosition();
  }

  public void stop(){
    elevatorMotor.stopMotor();
  }

  public void setSpeed(double speed, String direction) {

    if (direction.equals("up")) {
      elevatorMotor.set(speed);
    } 
    else if (direction.equals("down")) {
    elevatorMotor.set(speed*-1);
    }
//  public void moveToPosition(variable for distance if possible ){
//  run motor at desired speed
//  if encoder get position is at whatever coral level
//    stop motor
// }
  }

 /* public void moveToPosition(double distance,String direction){
    elevatorMotor.set(0.5);
    if(elevatorEncoder.getPosition() >= distance){
      elevatorMotor.stopMotor();
    }
    

  } */
//  run motor at desired speed
//  if encoder get position is at whatever coral level
//    stop motor
// }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
