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
import frc.robot.Constants;
import frc.robot.ParadoxField;

public class PivotSubsystem extends SubsystemBase {
  private double m_power;
  // private double m_targetAngleInDegrees = 0;

  private static final double k_deadzone = 0;
  private PIDController m_PID;
  private double m_setPoint = Constants.PivotConstants.k_resetPositionDegrees;

  private double[] k_distancse = Constants.PivotConstants.k_distancesFront;

  private double[] k_angles = Constants.PivotConstants.k_anglesFront;

  private CANSparkFlex m_pivotMotor = new CANSparkFlex(Constants.PivotConstants.k_pivotMotor, MotorType.kBrushless);
  DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(0);

  private DriveSubsystem m_driveSubsystem;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(DriveSubsystem driveSubsystem) {
    m_PID = new PIDController(Constants.PivotConstants.k_p, Constants.PivotConstants.k_i, Constants.PivotConstants.k_d);
    m_driveSubsystem = driveSubsystem;
    m_pivotMotor.setSmartCurrentLimit(38);
    m_pivotMotor.restoreFactoryDefaults();
    setBrakeMode(true);
    m_pivotEncoder.setPositionOffset(-0.8);
    m_PID.setIZone(Constants.PivotConstants.k_iZone);
    m_pivotMotor.setInverted(Constants.PivotConstants.k_isInverted);
  }

  public void setBrakeMode(boolean brake) {
    m_pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setPower(double power) {
    m_power = power;
  }

  public void setPositionDegrees(double angle) {
    m_setPoint = angle;
  }

  public double getPivotAngleFromRobotPos(boolean predictFuture) {
    double distance = predictFuture ? m_driveSubsystem.getFutureTranslationDistanceFromSpeakerMeters() : m_driveSubsystem.getTranslationalDistanceFromSpeakerMeters();
    if (distance > 6.1 || distance < 1.6){
      return 0;
    }
    for (int i = 0; i < k_distancse.length; i++) {
      if (distance > k_distancse[i] && distance < k_distancse[i+1]) {
        double roc = (k_angles[i+1] - k_angles[i]) / (k_distancse[i+1] - k_distancse[i]);
        double dist = distance - k_distancse[i];
        return k_angles[i] + dist * roc; 
      }
    }
    return 0;
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

    FF = Constants.PivotConstants.k_f * Math.sin(angle - 25);
    if(Math.abs(getAngleInDegrees() - m_setPoint) > k_deadzone){
      pid = m_PID.calculate(angle, m_setPoint);
    } else {
      pid = 0;
    }
    m_power = FF + pid;
    SmartDashboard.putNumber("Pivot Power", m_power);
    SmartDashboard.putNumber("Calculated Error", Math.abs(getAngleInDegrees() - m_setPoint));
    SmartDashboard.putNumber("Set Point", m_setPoint);
    if (Constants.States.m_enableSuperstructure) {
      m_pivotMotor.set(m_power);
    }
  }
}
