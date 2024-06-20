// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  DriveSubsystem m_driveSubsystem;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
  }

  public void setBrakeMode(boolean brake) {
  }

  public void setPower(double power) {
  }

  public void setPositionDegrees(double angle) {
  }

  // Autos only, to be removed
  public double getPivotAngleFromDistanceFromSpeaker(double distance) {
    return 0;
  }

  public double getPivotAngleFromRobotPos(boolean predictFuture) {
    return 0;
  }

  public double getAngleInDegrees() {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
