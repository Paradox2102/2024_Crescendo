// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class SetPivotAngleCommand extends Command {
  PivotSubsystem m_subsystem;
  private double m_angle; 
  /** Creates a new TestPivotPID. */
  public SetPivotAngleCommand(PivotSubsystem pivotSubsystem, double angle) {
    m_subsystem = pivotSubsystem;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPositionDegrees(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPositionDegrees(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
