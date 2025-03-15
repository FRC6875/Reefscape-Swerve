// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.MechanismConstants.LaserConstants;


public class LaserSubsystem extends SubsystemBase {

  AnalogInput analog = new AnalogInput(LaserConstants.kLaserPort);

  /** Creates a new LaserSubsystem. */
  public LaserSubsystem() {

  }

  public double getValue(){
    return (analog.getValue()*48.78136376-4.985354503)*-1;//return the distance in inches
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
