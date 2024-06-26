// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkFlex m_elevatorMotor = new CANSparkFlex(Constants.ElevatorConstants.k_elevatorMotor, MotorType.kBrushless);
  private RelativeEncoder m_elevatorEncoder;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorMotor.restoreFactoryDefaults();
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_elevatorMotor.setIdleMode(IdleMode.kBrake);
    m_elevatorMotor.burnFlash();
    m_elevatorMotor.setSmartCurrentLimit(100); // 80
  }

  public double getRawElevatorPosition() {
    return m_elevatorEncoder.getPosition();
  }

  public double getCookedElevatorPosition() {
    return m_elevatorEncoder.getPosition() * Constants.ElevatorConstants.k_ticksToInches;// + Constants.ElevatorConstants.k_zeroPoint;
  }

  public void setPower(double power) {
    // m_power = power;
    m_elevatorMotor.set(power);
  }

  @Override
  public void periodic() {}
}
