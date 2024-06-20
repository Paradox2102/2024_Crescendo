// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakeCommand extends Command {
  ManipulatorSubsystem m_backSubsystem;
  ManipulatorSubsystem m_frontSubsystem;
  PivotSubsystem m_pivotSubsystem;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(ManipulatorSubsystem backSubsystem, ManipulatorSubsystem frontSubsystem, PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_backSubsystem = backSubsystem;
    m_frontSubsystem = frontSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    addRequirements(m_backSubsystem, m_frontSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_backSubsystem.setVelocityRPM(Constants.BackConstants.k_intakeVelocityRPM);
    m_frontSubsystem.setVelocityRPM(Constants.FrontConstants.k_intakeVelocityRPM);
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
    Constants.States.m_intaking = true;
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_backSubsystem.stop();
    m_frontSubsystem.stop();
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_resetPositionDegrees);
    Constants.States.m_intaking = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.States.m_isGamePieceStowed;
  }
}
