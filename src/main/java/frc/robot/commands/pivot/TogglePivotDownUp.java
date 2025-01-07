// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class TogglePivotDownUp extends Command {
  PivotSubsystem m_subsystem;
  /** Creates a new TogglePivotDownUp. */
  public TogglePivotDownUp(PivotSubsystem pivotSubsystem) {
    m_subsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPositionDegrees(Constants.PivotConstants.k_resetPositionDegrees);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
