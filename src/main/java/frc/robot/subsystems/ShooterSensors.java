// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSensors extends SubsystemBase {
  AnalogInput m_frontSensor = new AnalogInput(Constants.ShooterConstants.k_frontSensor);
  AnalogInput m_backSensor = new AnalogInput(Constants.ShooterConstants.k_backSensor);

  private double k_minDistanceFront = 520;
  private double k_minDistanceBack = 300;

  private int m_counter = 0;

  /** Creates a new ShooterSensors. */
  public ShooterSensors() {}

  public int getFrontDistance() {
    return m_frontSensor.getValue();
  }

  public int getBackDistance() {
    return m_backSensor.getValue();
  }

  public boolean getFrontSensor() {
    return getFrontDistance() > k_minDistanceFront;
  }

  public boolean getBackSensor() {
    return getBackDistance() > k_minDistanceBack;
  }

  public boolean hasGamePiece() {
    return getFrontSensor() || getBackSensor();
  }

  public boolean isGamePieceStowed() {
    // If we are shooting intake side, check back sensors and not front sensors, if not, the opposite.
    return Constants.m_shootIntakeSide ? (getBackSensor() && !getFrontSensor()) : (getFrontSensor() && !getBackSensor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Constants.m_hasGamePiece = hasGamePiece() && m_counter > 3;
    Constants.m_isGamePieceStowed = isGamePieceStowed();

    if (hasGamePiece()) {
      m_counter += 1;
    } else {
      m_counter = 0;
    }

    SmartDashboard.putNumber("Front Sensor", getFrontDistance());
    SmartDashboard.putNumber("Back Sensor", getBackDistance());
    SmartDashboard.putBoolean("Has Game Piece", Constants.m_hasGamePiece);
    SmartDashboard.putBoolean("Boolean Back", getBackSensor());
    SmartDashboard.putBoolean("Boolean Front", getFrontSensor());
    SmartDashboard.putBoolean("Game Piece Stowed", Constants.m_isGamePieceStowed);
  }
}
