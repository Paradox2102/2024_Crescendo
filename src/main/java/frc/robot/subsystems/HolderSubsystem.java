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

public class HolderSubsystem extends SubsystemBase {
  private CANSparkFlex m_motor = new CANSparkFlex(Constants.HolderConstants.k_holdingMotor, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_motor.getEncoder();

  private double m_finalPower = 0;

  private final double k_p = 0.00004;
  private final double k_i = 0.0015;
  private final double k_d = 0;
  private final double k_iZone = 50;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);
  private double m_velocity = 0;
  /** Creates a new FrontSubsystem. */
  public HolderSubsystem() {
    m_PID.setIZone(k_iZone);
    setBrakeMode(true);
    m_motor.setInverted(true);
  }

  public boolean isReady() {
    return Math.abs(m_velocity - getVelocityRPM()) < Constants.HolderConstants.k_deadzone;
  }

  public void setPower(double power) {
    m_motor.set(power);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setVelocityRPM(double velocity) {
    m_velocity = velocity;
  }

  public double getVelocityRPM() {
    return m_encoder.getVelocity();
  }

  public void stop() {
    setVelocityRPM(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentVelocity = getVelocityRPM();
    
    double F = m_velocity / 5350;

    SmartDashboard.putNumber("Holder FTerm", F);

    double power = m_PID.calculate(currentVelocity, m_velocity);

    SmartDashboard.putNumber("holder power without fterm", power);

    // If not shooting, make sure gamepiece is stowed, else shoot
    if (!Constants.m_runningShooterAndHolder) {
      if (!Constants.m_isGamePieceStowed && Constants.m_hasGamePiece) {
        // Move the motor in direction depending on which way to stow
        m_finalPower = Constants.m_shootIntakeSide ? -Constants.HolderConstants.k_adjustGamePiecePower : Constants.HolderConstants.k_adjustGamePiecePower;
      } else {
        m_finalPower = 0;
      }
    } else {
      m_finalPower = F + power;
    }
    setPower(m_finalPower);
    
    SmartDashboard.putNumber("Holder Front Velo", currentVelocity);
    SmartDashboard.putNumber("Holder Target Front Velocity", m_velocity);
    SmartDashboard.putNumber("Final Target Power", m_finalPower);
  }
}
