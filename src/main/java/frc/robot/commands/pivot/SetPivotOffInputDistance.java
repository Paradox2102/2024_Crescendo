// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPivotOffInputDistance extends InstantCommand {
  PivotSubsystem m_subsystem;
  double m_distance;
  public SetPivotOffInputDistance(PivotSubsystem pivotSubsystem, double distance) {
    m_subsystem = pivotSubsystem;
    m_distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPositionDegrees(m_subsystem.getPivotAngleFromDistanceFromSpeaker(m_distance));
  }
}
