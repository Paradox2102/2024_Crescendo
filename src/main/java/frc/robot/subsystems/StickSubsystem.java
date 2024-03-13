// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
55 cansparkmaxes
relative emcoder 
setPoint
rack and pinion - motors run by a it (so changing the motors changes both)

13 & 14
*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StickSubsystem extends SubsystemBase {
  private CANSparkMax m_leftMotor = new CANSparkMax(Constants.StickConstants.k_leftStickMotor, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(Constants.StickConstants.m_rightStickMotor, MotorType.kBrushless);

  private RelativeEncoder m_leftStickEncoder;
  private RelativeEncoder m_rightStickEncoder;

  private double m_stickPoint;

  private static final double k_tiksToInches = 1;

  //NEVER CHANGE
  private static final double k_currentLimit = 10;
  
  /** Creates a new StickSubsystem. */
  public StickSubsystem() {
    m_leftStickEncoder = m_leftMotor.getEncoder();
    m_leftStickEncoder = m_rightMotor.getEncoder();
  }

  public void setPoint(double point) {
    m_stickPoint = point;
  }

  //getting the positions
  public double getLeftRawPosition() {
    return m_leftStickEncoder.getPosition();
  }
  public double getRightRawPosition() {
    return m_rightStickEncoder.getPosition();
  }

  public double getLeftCookedPosition() {
    return m_leftStickEncoder.getPosition() * k_tiksToInches;
  }
  public double getRightCookedPosition() {
    return m_rightStickEncoder.getPosition() * k_tiksToInches;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
