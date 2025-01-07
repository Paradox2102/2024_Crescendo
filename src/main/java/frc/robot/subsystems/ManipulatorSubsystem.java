// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkFlex m_motor;
  private RelativeEncoder m_encoder;
  private final SparkPIDController m_PID;
  boolean m_front;
  DoubleSupplier m_distanceFromSpeaker;

  private double m_velocity = 0;
  /** Creates a new FrontSubsystem. */
  public ManipulatorSubsystem(boolean front, DoubleSupplier distance) {
    m_front = front;
    //SmartDashboard.putNumber("Amp Velo", 0);
    m_motor = new CANSparkFlex(m_front ? Constants.FrontConstants.k_frontMotor : Constants.BackConstants.k_backMotor, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_PID = m_motor.getPIDController();
    m_distanceFromSpeaker = distance;
    m_PID.setFF(m_front ? Constants.FrontConstants.k_f : Constants.BackConstants.k_f);
    m_PID.setP(m_front ? Constants.FrontConstants.k_p : Constants.BackConstants.k_p);
    m_PID.setI(m_front ? Constants.FrontConstants.k_i : Constants.BackConstants.k_i);
    m_PID.setD(m_front ? Constants.FrontConstants.k_d : Constants.BackConstants.k_d);
    setBrakeMode(true);
    m_motor.setSmartCurrentLimit(80);
    setName(m_front ? "FrontSubsystem" : "BackSubsystem");
    m_motor.setInverted(Constants.States.m_isCompetition ? !m_front : m_front);
    m_motor.burnFlash();
  }

  // BUG: This is not the right way to do this test.  Why not use MathUtil.isNear()? - Gavin
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

  // public double getRevSpeed() {
  //   for (int i = 0; i < Constants.FrontConstants.k_revDistances.length; i++) {
  //     if (m_distanceFromSpeaker.getAsDouble() <= Constants.FrontConstants.k_revDistances[i]) {
  //       return Constants.FrontConstants.k_revSpeeds[i];
  //     }
  //   }
  //   return Constants.FrontConstants.k_revSpeeds[Constants.FrontConstants.k_revSpeeds.length - 1];
  // }

  public double getRevSpeed() {
    double distance = m_distanceFromSpeaker.getAsDouble();
    double[] distances = Constants.FrontConstants.k_revDistances;
    double[] speeds = Constants.FrontConstants.k_revSpeeds;
    if (distance > distances[distances.length - 1] || distance < distances[0]){
      return 0;
    }
    for (int i = 0; i < distances.length; i++) {
      if (distance > distances[i] && distance <= distances[i+1]) {
        double roc = (speeds[i+1] - speeds[i]) / (distances[i+1] - distances[i]);
        double dist = distance -distances[i];
        return (speeds[i] + dist * roc); 
      }
    }
    return 0;
  }

  @Override
  public void periodic() {
    double velocity = getVelocityRPM();
    SmartDashboard.putNumber(getName() + " Speed", Math.abs(velocity));
    SmartDashboard.putNumber(getName() + " Target Speed", m_velocity);
    // SmartDashboard.putBoolean(getName() + "in shoot speed range", Math.abs(velocity) >= 4500);
    //Constants.FrontConstants.k_ampShootVelocityRPM = SmartDashboard.getEntry("Amp Velo").getDouble(0);
  }
}
