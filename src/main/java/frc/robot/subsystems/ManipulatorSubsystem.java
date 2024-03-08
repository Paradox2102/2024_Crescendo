// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkFlex m_motor;
  private RelativeEncoder m_encoder;
  private final SparkPIDController m_PID;
  boolean m_shooter;

  private double m_velocity = 0;
  /** Creates a new FrontSubsystem. */
  public ManipulatorSubsystem(int id) {
    m_shooter = id == Constants.ShooterConstants.k_shooterMotor;
    SmartDashboard.putNumber("Amp Velo", 0);
    m_motor = new CANSparkFlex(id, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_PID = m_motor.getPIDController();
    if (m_shooter) {
      m_PID.setFF(Constants.ShooterConstants.k_f);
      m_PID.setP(Constants.ShooterConstants.k_p);
      m_PID.setI(Constants.ShooterConstants.k_i);
      m_PID.setD(Constants.ShooterConstants.k_d);
      m_PID.setIZone(Constants.ShooterConstants.k_iZone);
      m_motor.setInverted(Constants.States.m_isCompetition);
    } else {
      m_PID.setFF(Constants.HolderConstants.k_f);
      m_PID.setP(Constants.HolderConstants.k_p);
      m_PID.setI(Constants.HolderConstants.k_i);
      m_PID.setD(Constants.HolderConstants.k_d);
      m_PID.setIZone(Constants.HolderConstants.k_iZone);
      m_motor.setInverted(!Constants.States.m_isCompetition);
    }
    setBrakeMode(true);
    m_motor.setSmartCurrentLimit(80);
    setName(m_shooter ? "ShooterSubsystem" : "HolderSubsystem");
    m_motor.setInverted(m_shooter);
    m_motor.burnFlash();
  }

  public boolean isReady() {
    return Math.abs(getVelocityRPM()) > m_velocity - (m_shooter ? Constants.ShooterConstants.k_deadzone : Constants.HolderConstants.k_deadzone);
  }

  public void setPower(double power) {
    m_PID.setReference(power, ControlType.kDutyCycle);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setVelocityRPM(double velocity) {
    m_velocity = velocity;
    m_PID.setReference(velocity, ControlType.kVelocity);
  }

  public double getVelocityRPM() {
    return m_encoder.getVelocity();
  }

  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {
    double velocity = getVelocityRPM();
    SmartDashboard.putNumber(getName() + " Speed", Math.abs(velocity));
    SmartDashboard.putNumber(getName() + " Target Speed", m_velocity);
    SmartDashboard.putBoolean(getName() + "in shoot speed range", Math.abs(velocity) >= 4500);
    Constants.ShooterConstants.k_ampShootVelocityRPM = SmartDashboard.getEntry("Amp Velo").getDouble(0);
  }
}
