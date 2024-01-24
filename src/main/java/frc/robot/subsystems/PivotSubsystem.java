// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
  private double m_power;
  private double m_targetAngleInDegrees = 0;

  private static final double k_f = 0;
  private static final double k_p = 0;
  private static final double k_i = 0;
  private static final double k_d = 0;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);
  private boolean m_PIDOn = false;
  private double m_setPoint = Constants.ArmConstants.k_armStartingPos;

  private CANSparkFlex m_armMotor = new CANSparkFlex(Constants.ArmConstants.k_armMotor, MotorType.kBrushless);
  DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(0);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    m_armMotor.restoreFactoryDefaults();
    setBrakeMode(true);
  }

  public void setBrakeMode(boolean brake) {
    m_armMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setPower(double power) {
    m_PIDOn = false;
    m_power = power;
  }

  public void setPosition(double angle) {
    m_PIDOn = true;
    m_setPoint = angle;
  }

  private double getRawAngle() {
    return m_armEncoder.getAbsolutePosition();
  }

  private double getAngleInDegrees() {
    return m_armEncoder.getAbsolutePosition() * Constants.ArmConstants.k_armTicksToDegrees;
  }

  private double setFterm(double angle) {
    m_targetAngleInDegrees = angle;
    double fTerm = (-k_f * Math.sin(Math.toRadians(m_targetAngleInDegrees)));
    return fTerm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Raw Encoder Value", getAngleInDegrees());
    if(m_PIDOn){
      m_power = setFterm(m_setPoint) + m_PID.calculate(getAngleInDegrees(), m_setPoint);
    }
    m_armMotor.set(m_power);
  }
}
