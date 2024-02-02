// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkFlex m_elevatorMotor = new CANSparkFlex(Constants.ElevatorConstants.k_elevatorMotor, MotorType.kBrushless);
  private RelativeEncoder m_elevatorEncoder;
  private double m_elevatorPoint;

  private double m_power;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
  }

  public double getRawElevatorPosition() {
    return m_elevatorEncoder.getPosition();
  }

  public void setPower(double power) {
    m_power = power;
  }

  public void setPosition(double position) {
    m_elevatorPoint = position;
  }

  @Override
  public void periodic() {
    //show the extention of 
    SmartDashboard.putNumber("Elevator Raw Position", getRawElevatorPosition());
    // This method will be called once per scheduler run
    //position (extend)
    //conversion factor ticks --> inches
  }
}
