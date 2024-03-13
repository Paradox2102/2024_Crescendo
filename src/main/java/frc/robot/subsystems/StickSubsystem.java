// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
55 cansparkmaxes
relative emcoder 
setPoint
rachandpinion - motors run by a it (so changing the motors changes both)

13 & 14
*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StickSubsystem extends SubsystemBase {
  private CANSparkMax m_leftMotor = new CANSparkMax(Constants.StickConstants.k_leftStickMotor, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(Constants.StickConstants.m_rightStickMotor, MotorType.kBrushless);

  // private RelativeEncoder m_leftstickEncoder;

  /** Creates a new StickSubsystem. */
  public StickSubsystem() {
    // m_leftStickEncoder = m_leftMotor.getEncoder();
  }

  public void setPoint() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
