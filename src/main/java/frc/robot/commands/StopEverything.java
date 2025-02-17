// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopEverything extends InstantCommand {
  public StopEverything(DriveSubsystem driveSubsystem, ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem, PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, frontSubsystem, backSubsystem, pivotSubsystem);
  }

  public StopEverything(DriveSubsystem driveSubsystem, ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, frontSubsystem, backSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
