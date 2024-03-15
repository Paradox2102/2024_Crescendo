// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
55 cansparkmaxes
relative emcoder 
setPoint
rack and pinion - motors run by a it (so changing the motors changes both)

13 & 14
*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StickSubsystem extends SubsystemBase {
  private CANSparkMax m_leftMotor = new CANSparkMax(Constants.StickConstants.k_leftStickMotor, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(Constants.StickConstants.k_rightStickMotor, MotorType.kBrushless);

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  public boolean m_retracted = true;
  private boolean m_isRunning = false;
  private Timer m_stallTimer = new Timer();

  //NEVER CHANGE
  private static final int k_currentLimit = 10;
  
  /** Creates a new StickSubsystem. */
  public StickSubsystem() {
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(k_currentLimit);

    m_leftMotor.setSmartCurrentLimit(k_currentLimit);
    m_rightMotor.setSmartCurrentLimit(k_currentLimit);

    setBrakeMode(true);
    m_stallTimer.reset();
    m_stallTimer.start();
    // m_rightMotor.follow(m_leftMotor, true);
  }

  //getting the positions
  public double getPositionInRotations() {
    return m_leftEncoder.getPosition();
  }

  public void setBrakeMode(boolean brake) {
    m_leftMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setPower(double power) {
    m_isRunning = true;
    m_leftMotor.set(power);
    m_rightMotor.set(-power);
  }

  public void stop() {
    m_isRunning = false;
    setPower(0);
  }


  @Override
  public void periodic() {
    if(m_isRunning == true && Math.abs(m_leftEncoder.getVelocity()) < 0.5 && Math.abs(m_rightEncoder.getVelocity()) < 0.5){
      if(m_stallTimer.get() >= 0.1){
        m_isRunning = false;
        stop();
        m_leftEncoder.setPosition(m_retracted ? 0 : Constants.StickConstants.k_maxExtentRotations);
      }
    } else {
      m_stallTimer.reset();
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Stick Extent", getPositionInRotations());
    SmartDashboard.putNumber("Stick Velocity", m_leftEncoder.getVelocity());
  }
}
