// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetSubsystemsCommand extends InstantCommand {
  PivotSubsystem m_pivotSubsystem;
  ManipulatorSubsystem m_frontSubsystem;
  ManipulatorSubsystem m_backSubsystem;
  public ResetSubsystemsCommand(PivotSubsystem pivotSubsystem, ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivotSubsystem = pivotSubsystem;
    m_frontSubsystem = frontSubsystem;
    m_backSubsystem = backSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotSubsystem.setPositionDegrees(5);
    m_frontSubsystem.stop();
    m_backSubsystem.stop();
  }
}
