// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class DefaultPivotCommand extends Command {
  /** Creates a new DefaultPivotCommand. */
  PivotSubsystem m_subsystem;
  boolean m_predictFuture;
  public DefaultPivotCommand(PivotSubsystem pivotSubsystem, boolean predictFuture) {
    m_predictFuture = predictFuture;
    m_subsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setPositionDegrees(m_subsystem.getPivotAngleFromRobotPos(m_predictFuture));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
