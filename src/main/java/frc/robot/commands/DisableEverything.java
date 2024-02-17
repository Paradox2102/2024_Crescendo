// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DisableEverything extends InstantCommand {
  public DisableEverything(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem, PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, shooterSubsystem, holderSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.States.m_speakerMode = false;
  }
}
