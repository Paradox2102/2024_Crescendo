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
  boolean m_front;
  //do we want to adjust speed based on distance?
  DriveSubsystem m_driveSubsystem;

  private double m_velocity = 0;
  /** Creates a new FrontSubsystem. */
  //config class in constants to control which side, or pass in m_front instead of the id
  public ManipulatorSubsystem(DriveSubsystem driveSubsystem, int id) {
    m_front = id == Constants.FrontConstants.k_frontMotor;
    //SmartDashboard.putNumber("Amp Velo", 0);
    m_motor = new CANSparkFlex(id, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_PID = m_motor.getPIDController();
    m_driveSubsystem = driveSubsystem;
    //probably need two sets of constants based on which side we are shooting out of, all of this is very messy
    if (m_front) {
      m_PID.setFF(Constants.FrontConstants.k_f);
      m_PID.setP(Constants.FrontConstants.k_p);
      m_PID.setI(Constants.FrontConstants.k_i);
      m_PID.setD(Constants.FrontConstants.k_d);
      m_PID.setIZone(Constants.FrontConstants.k_iZone);
      m_motor.setInverted(Constants.States.m_isCompetition);
    } else {
      m_PID.setFF(Constants.BackConstants.k_f);
      m_PID.setP(Constants.BackConstants.k_p);
      m_PID.setI(Constants.BackConstants.k_i);
      m_PID.setD(Constants.BackConstants.k_d);
      m_PID.setIZone(Constants.BackConstants.k_iZone);
      m_motor.setInverted(!Constants.States.m_isCompetition);
    }
    setBrakeMode(true);
    m_motor.setSmartCurrentLimit(80);
    setName(m_front ? "FrontSubsystem" : "BackSubsystem");
    //m_motor.setInverted(Constants.States.m_isCompetition ? !m_front : m_front);
    m_motor.burnFlash();
  }

  public boolean isReady() {
    return Math.abs(getVelocityRPM()) > Math.abs(m_velocity) - (m_front ? Constants.FrontConstants.k_deadzone : Constants.BackConstants.k_deadzone);
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

  public double getRevSpeed() {
    double distanceFromSpeaker = m_driveSubsystem.getFutureTranslationDistanceFromSpeakerMeters();
    for (int i = 0; i < Constants.FrontConstants.k_revDistances.length; i++) {
      if (distanceFromSpeaker <= Constants.FrontConstants.k_revDistances[i]) {
        return Constants.FrontConstants.k_revSpeeds[i];
      }
    }
    return 0;
  }

  @Override
  public void periodic() {
    double velocity = getVelocityRPM();
    SmartDashboard.putNumber(getName() + " Speed", Math.abs(velocity));
    // SmartDashboard.putNumber(getName() + " Target Speed", m_velocity);
    // SmartDashboard.putBoolean(getName() + "in shoot speed range", Math.abs(velocity) >= 4500);
    //Constants.FrontConstants.k_ampShootVelocityRPM = SmartDashboard.getEntry("Amp Velo").getDouble(0);
  }
}
