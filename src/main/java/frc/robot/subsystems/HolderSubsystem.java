// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HolderSubsystem extends SubsystemBase {
  private CANSparkFlex m_motor = new CANSparkFlex(Constants.HolderConstants.k_holdingMotor, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_motor.getEncoder();

  private double m_finalPower = 0;

  private PIDController m_PID;
  private double m_velocity = 0;
  private Timer m_timer = new Timer();
  /** Creates a new FrontSubsystem. */
  public HolderSubsystem() {
    m_PID = new PIDController(Constants.HolderConstants.k_p, Constants.HolderConstants.k_i, Constants.HolderConstants.k_d);
    m_PID.setIZone(Constants.HolderConstants.k_iZone);
    setBrakeMode(true);
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(1000);
    m_timer.reset();
    m_timer.start();
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
    if (!Constants.States.m_runningShooterAndHolder) {
      if (!Constants.States.m_isGamePieceStowed && Constants.States.m_hasGamePiece) {
        // Move the motor in direction depending on which way to stow
        m_velocity = Constants.States.m_shootIntakeSide ? -Constants.HolderConstants.k_adjustGamePiecePower : Constants.HolderConstants.k_adjustGamePiecePower;
        F = m_velocity * Constants.HolderConstants.k_f;
        m_finalPower = F + m_PID.calculate(currentVelocity, m_velocity);
      } else {
        m_finalPower = 0;
      }
    } else {
      m_finalPower = F + power;
    }
    if (Constants.States.m_enableSuperstructure) {
      setPower(m_finalPower);
    }
    
    SmartDashboard.putBoolean("has game piece", Constants.States.m_hasGamePiece);
    SmartDashboard.putNumber("Holder Front Velo", currentVelocity);
    SmartDashboard.putNumber("Holder Target Front Velocity", m_velocity);
    SmartDashboard.putNumber("Final Target Power", m_finalPower);
  }
}
