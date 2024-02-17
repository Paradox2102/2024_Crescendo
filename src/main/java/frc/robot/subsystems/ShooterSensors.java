// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSensors extends SubsystemBase {
  DigitalInput m_frontSensor = new DigitalInput(Constants.ShooterConstants.k_frontSensor);
  DigitalInput m_backSensor = new DigitalInput(Constants.ShooterConstants.k_backSensor);

  private int m_frontCounter = 0;
  private int m_backCounter = 0;

  /** Creates a new ShooterSensors. */
  public ShooterSensors() {}

  public boolean getFrontSensor() {
    return m_frontSensor.get();
  }

  public boolean getBackSensor() {
    return m_backSensor.get();
  }

  public boolean getFrontSensorWithTime() {
    return getFrontSensor() && m_frontCounter > 3;
  }

  public boolean getBackSensorWithTime() {
    return getBackSensor() && m_backCounter > 3;
  }

  public boolean hasGamePiece() {
    return getFrontSensorWithTime() || getBackSensorWithTime();
  }

  public boolean isGamePieceStowed() {
    // If we are shooting intake side, check back sensors and not front sensors, if not, the opposite.
    return Constants.States.m_shootIntakeSide ? (getBackSensorWithTime() && !getFrontSensorWithTime()) : (getFrontSensorWithTime() && !getBackSensorWithTime());
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // Constants.m_hasGamePiece = hasGamePiece();
    // Constants.m_isGamePieceStowed = isGamePieceStowed();

    // if (getFrontSensor()) {
    //   if (m_frontCounter < 5) {
    //     m_frontCounter += 1;
    //   }
    // } else {
    //   m_frontCounter = 0;
    // }

    // if (getBackSensor()) {
    //   if (m_backCounter < 5) {
    //     m_backCounter += 1;
    //   }
    // } else {
    //   m_backCounter = 0;
    // }

    // SmartDashboard.putNumber("Front Sensor", getFrontDistance());
    // SmartDashboard.putNumber("Back Sensor", getBackDistance());
    // SmartDashboard.putBoolean("Has Game Piece", Constants.m_hasGamePiece);
    // SmartDashboard.putBoolean("Boolean Back", getBackSensorWithTime());
    // SmartDashboard.putBoolean("Boolean Front", getFrontSensorWithTime());
    // SmartDashboard.putBoolean("Game Piece Stowed", Constants.m_isGamePieceStowed);
    // SmartDashboard.putBoolean("Shoot Intake Side", Constants.m_shootIntakeSide);
  }
}
