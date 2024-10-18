// Copyright (c) FIRST and other WPILib contributors.
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
  private int m_power;
  private int m_desiredPosition;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorMotor.restoreFactoryDefaults();
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_elevatorMotor.setIdleMode(IdleMode.kBrake);
    m_elevatorMotor.burnFlash();
    m_elevatorMotor.setSmartCurrentLimit(80); // 80
    
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

  public void setPosition(int desiredPos) {
    // 1 is max extension. 0 is all the way down.
    m_desiredPosition = desiredPos;
  }
  

  @Override
  public void periodic() {
    // if (getCookedElevatorPosition() <= Constants.ElevatorConstants.k_maxDistance && m_desiredPosition == 1) {
    //   setPower(1);
    // } else if (getCookedElevatorPosition() >= 1 && m_desiredPosition == 0) {
    //   setPower(-1);
    //   }
  }
}
