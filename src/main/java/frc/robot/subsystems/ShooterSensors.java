// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSensors extends SubsystemBase {
  DigitalInput m_frontSensor = new DigitalInput(Constants.ShooterConstants.k_frontSensor);
  DigitalInput m_backSensor = new DigitalInput(Constants.ShooterConstants.k_backSensor);

  /** Creates a new ShooterSensors. */
  public ShooterSensors() {}

  public boolean getFrontSensor() {
    return Constants.States.m_shootIntakeSide ? !m_frontSensor.get() : !m_backSensor.get();
  }

  public boolean getBackSensor() {
    return Constants.States.m_shootIntakeSide? !m_backSensor.get():!m_frontSensor.get();
  }

  // public boolean getFrontSensorWithTime() {
  //   return getFrontSensor() && m_frontCounter > 3;
  // }

  // public boolean getBackSensorWithTime() {
  //   return getBackSensor() && m_backCounter > 3;
  // }

  public boolean hasGamePiece() {
    // return getFrontSensorWithTime() || getBackSensorWithTime();
    return getFrontSensor() || getBackSensor();
  }

  public boolean isGamePieceStowed() {
    // If we are shooting intake side, check back sensors and not front sensors, if not, the opposite.
    // return Constants.States.m_shootIntakeSide ? (getBackSensorWithTime() && !getFrontSensorWithTime()) : (getFrontSensorWithTime() && !getBackSensorWithTime());
    return Constants.States.m_shootIntakeSide ? (getBackSensor() && !getFrontSensor()) : (getFrontSensor() && !getBackSensor());
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    Constants.States.m_hasGamePiece = hasGamePiece();
    Constants.States.m_isGamePieceStowed = isGamePieceStowed();

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

    // SmartDashboard.putBoolean("Has Game Piece", Constants.States.m_hasGamePiece);
    // SmartDashboard.putBoolean("Game Piece Stowed", Constants.States.m_isGamePieceStowed);
    // SmartDashboard.putBoolean("Shoot Intake Side", Constants.States.m_shootIntakeSide);
    SmartDashboard.putBoolean("Back Sensor", getBackSensor());
    SmartDashboard.putBoolean("Front Sensor", getFrontSensor());
  }
}
