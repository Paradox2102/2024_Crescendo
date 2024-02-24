// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleBrakeMode extends InstantCommand {
  PivotSubsystem m_subsystem;
  Trigger m_brake;
  public ToggleBrakeMode(PivotSubsystem pivotSubsystem, Trigger brake) {
    m_subsystem = pivotSubsystem;
    m_brake = brake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setBrakeMode(m_brake.getAsBoolean());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
