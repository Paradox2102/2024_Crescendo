// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  DriveSubsystem m_driveSubsystem;

  /** Creates a new FrontSubsystem. */
  public ManipulatorSubsystem(DriveSubsystem driveSubsystem, int id) {
    m_driveSubsystem = driveSubsystem;
  }

  public boolean isReady() {
    return false;
  }

  public void setPower(double power) {
  }

  public void setBrakeMode(boolean brake) {
  }

  public void setVelocityRPM(double velocity) {
  }

  public double getVelocityRPM() {
    return 0;
  }

  public void stop() {
  }

  public double getRevSpeed() {
    return 0;
  }

  @Override
  public void periodic() {
  }
}
