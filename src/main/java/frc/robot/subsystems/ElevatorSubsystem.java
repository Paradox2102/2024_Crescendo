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

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkFlex m_elevatorMotor = new CANSparkFlex(Constants.ElevatorConstants.k_elevatorMotor, MotorType.kBrushless);
  private RelativeEncoder m_elevatorEncoder;
  private double m_elevatorPoint;

  //pid stuff
  private static final double k_p = 0;
  private static final double k_i = 0;
  private static final double k_d = 0;
  private static final double k_f = 0;
  private static final double k_iZone = 0;
  private static final double k_deadzone = 0;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);


  private double m_power;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_elevatorMotor.setIdleMode(IdleMode.kBrake);
    
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

  public void setPosition(double position) {
    m_elevatorPoint = position;
  }

  @Override
  public void periodic() {
    //setPosition
    double pid = m_PID.calculate(getCookedElevatorPosition(), m_elevatorPoint);
    SmartDashboard.putNumber("Distance from Elevator Point", m_elevatorPoint - getCookedElevatorPosition());
    //power
    SmartDashboard.putNumber("Elevator Power", m_power);
    m_power = k_f * Math.signum(pid) + pid;
    // m_elevatorMotor.set(m_power);
    //show displays on the SmartDasboard --> where elevator is positioned
    SmartDashboard.putNumber("Elevator Raw Position", getRawElevatorPosition());
    SmartDashboard.putNumber("Elevator Cooked Position", getCookedElevatorPosition());
    // This method will be called once per scheduler run
  }
}
