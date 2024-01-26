// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkFlex m_motor = new CANSparkFlex(Constants.ShooterConstants.k_shooterMotor, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_motor.getEncoder();

  private final double k_p = .0002;
  private final double k_i = .002;
  private final double k_d = 0;
  private final double k_iZone = 150;
  private final double k_deadZone = 20;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  private double m_velocity = 0;
  /** Creates a new FrontSubsystem. */
  public ShooterSubsystem() {
    setBrakeMode(true);
    m_PID.setIZone(k_iZone);
  }

  public void setPower(double power) {
    m_motor.set(power);
  }

  public void setVelocityRPM(double velocity) {
    m_velocity = velocity;
  }

  public double getVelocityRPM() {
    return m_encoder.getVelocity();
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  public void stop() {
    setVelocityRPM(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentVelocity = getVelocityRPM();
    double F = m_velocity / 5000.0;
    
    double power = m_PID.calculate(currentVelocity, m_velocity);
    boolean intaking = m_velocity > 0;
    setPower(F + power);
    
    SmartDashboard.putNumber("Shooter Front Velo", currentVelocity);
    SmartDashboard.putNumber("Shooter Target Front Velocity", m_velocity);
    SmartDashboard.putNumber("Shooter Power", F + power);
  }
}
