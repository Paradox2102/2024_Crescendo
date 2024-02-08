// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.ParadoxField;

public class PivotSubsystem extends SubsystemBase {
  private double m_power;
  // private double m_targetAngleInDegrees = 0;

  private final double k_outwardFF = -0.015;
  private final double k_inwardFF = 0.015;
  private final double k_f = .015;
  private static final double k_p = 0.015;
  private static final double k_i = 0.035;
  private static final double k_d = 0;
  private static final double k_iZone = 6;
  private static final double k_holdPower = 0;
  private static final double k_deadzone = .5;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);
  private boolean m_PIDOn = false;
  private double m_setPoint = 0;

  private CANSparkFlex m_pivotMotor = new CANSparkFlex(Constants.PivotConstants.k_pivotMotor, MotorType.kBrushless);
  DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(0);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    m_pivotMotor.restoreFactoryDefaults();
    setBrakeMode(true);
    m_pivotEncoder.setPositionOffset(-0.8);
    m_PID.setIZone(k_iZone);
  }

  public void setBrakeMode(boolean brake) {
    m_pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setPower(double power) {
    m_PIDOn = false;
    m_power = power;
  }

  public void setPositionDegrees(double angle) {
    m_PIDOn = true;
    m_setPoint = angle;
  }

  private double getRawAngle() {
    return m_pivotEncoder.getAbsolutePosition();
  }

  public double getAngleInDegrees() {
    return ParadoxField.normalizeAngle(m_pivotEncoder.getAbsolutePosition() *  Constants.PivotConstants.k_pivotTicksToDegrees - Constants.PivotConstants.k_pivotZeroAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Raw Encoder Value", getRawAngle());
    SmartDashboard.putNumber("Angle in Degrees", getAngleInDegrees());
    double FF;
    double pid;
    double angle = getAngleInDegrees();
    // if (angle < 25) {
    //   FF = k_inwardFF;
    // } else if (angle > 45) {
    //   FF = k_outwardFF;
    // } else {
    //   FF = 0;
    // } 
    FF = k_f * Math.sin(angle - 35);
    if(Math.abs(getAngleInDegrees() - m_setPoint) > k_deadzone){
      pid = m_PID.calculate(angle, m_setPoint);
    } else {
      pid = 0;
    }
    m_power = FF + pid;
    SmartDashboard.putNumber("Power", m_power);
    SmartDashboard.putNumber("Calculated Error", Math.abs(getAngleInDegrees() - m_setPoint));
    SmartDashboard.putNumber("Set Point", m_setPoint);
    SmartDashboard.putNumber("Pivot PID", pid);
    m_pivotMotor.set(m_power);
  }
}
