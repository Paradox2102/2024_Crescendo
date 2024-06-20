// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class CountBulldoze extends Command {
  boolean m_lastState = false;
  ManipulatorSubsystem m_frontSubsystem;
  ManipulatorSubsystem m_backSubsystem;
  PivotSubsystem m_pivotSubsystem;
  int m_gamePiecesBulldozed = 0;
  /** Creates a new CountBulldoze. */
  public CountBulldoze(ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem, PivotSubsystem pivotSubsystem) {
    m_frontSubsystem = frontSubsystem;
    m_backSubsystem = backSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontSubsystem, m_backSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamePiecesBulldozed = 0;
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
    m_frontSubsystem.setVelocityRPM(Constants.FrontConstants.k_intakeVelocityRPM);
    m_backSubsystem.setVelocityRPM(Constants.FrontConstants.k_intakeVelocityRPM);
    // yes it's FrontConstants for back on purpose
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_lastState == true && !Constants.States.m_hasGamePiece) {
      m_gamePiecesBulldozed++;
    }
    m_lastState = Constants.States.m_hasGamePiece;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_backSubsystem.setVelocityRPM(Constants.BackConstants.k_intakeVelocityRPM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamePiecesBulldozed >= 4;
  }
}
