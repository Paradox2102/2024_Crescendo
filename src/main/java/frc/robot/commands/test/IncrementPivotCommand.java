// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.htm
public class IncrementPivotCommand extends InstantCommand {
  PivotSubsystem m_subsystem;
  boolean m_forward;
  public IncrementPivotCommand(PivotSubsystem pivotSubsystem, boolean forward) {
    m_subsystem = pivotSubsystem;
    m_forward = forward;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPositionDegrees(m_subsystem.getAngleInDegrees() + (m_forward ? 4 : -4));
  }
}
