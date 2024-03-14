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
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StickSubsystem extends SubsystemBase {
  private CANSparkMax m_leftMotor = new CANSparkMax(Constants.StickConstants.k_leftStickMotor, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(Constants.StickConstants.k_rightStickMotor, MotorType.kBrushless);

  private RelativeEncoder m_leftEncoder;

  public boolean m_retracted = true;

  //NEVER CHANGE
  private static final int k_currentLimit = 10;
  
  /** Creates a new StickSubsystem. */
  public StickSubsystem() {
    m_leftEncoder = m_leftMotor.getEncoder();

    m_leftMotor.setSmartCurrentLimit(k_currentLimit);
    m_rightMotor.setSmartCurrentLimit(k_currentLimit);

    setBrakeMode(true);

    // m_rightMotor.follow(m_leftMotor, true);
  }

  //getting the positions
  public double getPositionInRotations() {
    return m_leftEncoder.getPosition();
  }

  public void setBrakeMode(boolean brake) {
    m_leftMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setPower(double power) {
    m_leftMotor.set(power);
    m_rightMotor.set(-power);
  }

  public void stop() {
    setPower(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Stick Extent", getPositionInRotations());
  }
}
