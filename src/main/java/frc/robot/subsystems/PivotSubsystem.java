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
  private static final double k_p = 0.017;
  private static final double k_i = 0.03;
  private static final double k_d = 0;
  private static final double k_iZone = 10;
  private static final double k_holdPower = 0;
  private static final double k_deadzone = 0;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);
  private boolean m_PIDOn = false;
  private double m_setPoint = 0;

  private double[] m_distances = {
    2,
    2.25,
    2.5,
    2.75,
    3,
    3.25,
    3.5,
    3.75,
    4,
    4.25,
    4.5,
    4.75,
    5,
    5.5,
    5.75
  };

  private double[] m_angles = {
    0, // 2
    7, // 2.25
    10.5, // 2.5
    14, // 2.75
    17, // 3
    18.2, // 3.25
    19.9, // 3.5
    21, // 3.75
    23.1, // 4
    23.4, // 4.25
    23.9, // 4.5
    24.3, // 4.75
    25, // 5
    25.3, // 5.5
    26.5 // 5.75
  };

  private CANSparkFlex m_pivotMotor = new CANSparkFlex(Constants.PivotConstants.k_pivotMotor, MotorType.kBrushless);
  DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(0);

  private DriveSubsystem m_driveSubsystem;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
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

  public double getPivotAngleFromRobotPos() {
    double distance = m_driveSubsystem.getTranslationalDistanceFromSpeakerMeters();
    if (distance < 2 || distance > 5.75) {
      return 0;
    }
    for (int i = 0; i < m_distances.length; i++) {
      if (distance > m_distances[i] && distance < m_distances[i+1]) {
        double roc = (m_angles[i+1] - m_angles[i]) / (m_distances[i+1] - m_distances[i]);
        double dist = distance - m_distances[i];
        return m_angles[i] + dist * roc; 
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

    FF = k_f * Math.sin(angle - 25);
    if(Math.abs(getAngleInDegrees() - m_setPoint) > k_deadzone){
      pid = m_PID.calculate(angle, m_setPoint);
    } else {
      pid = 0;
    }
    m_power = FF + pid;
    // SmartDashboard.putNumber("Power", m_power);
    SmartDashboard.putNumber("Calculated Error", Math.abs(getAngleInDegrees() - m_setPoint));
    SmartDashboard.putNumber("Set Point", m_setPoint);
    // SmartDashboard.putNumber("Pivot PID", pid);
    m_pivotMotor.set(m_power);
  }
}
