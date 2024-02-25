// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

public class ShooterSubsystem extends HolderSubsystem {
  /** Creates a new FrontSubsystem. */
  public ShooterSubsystem() {
    super(Constants.ShooterConstants.k_shooterMotor, true);
  }
}