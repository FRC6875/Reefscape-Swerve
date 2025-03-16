// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.MechanismConstants.ServoConstants;

public class ArmSubsystem extends SubsystemBase {
  Servo servo = new Servo(ServoConstants.kServoPort);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {}

  public void setAngle(int angle){
     servo.setAngle(angle);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
