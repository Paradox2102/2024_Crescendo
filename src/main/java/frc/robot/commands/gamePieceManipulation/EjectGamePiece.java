// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class EjectGamePiece extends Command {
  /** Creates a new EjectGamePiece. */
  PivotSubsystem m_pivotSubsystem;
  ManipulatorSubsystem m_frontSubsystem;
  ManipulatorSubsystem m_backSubsystem;
  public EjectGamePiece(PivotSubsystem pivotSubsystem, ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem) {
    m_pivotSubsystem = pivotSubsystem;
    m_frontSubsystem = frontSubsystem;
    m_backSubsystem = backSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivotSubsystem, m_frontSubsystem, m_backSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotSubsystem.setPositionDegrees(70);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_frontSubsystem.setPower(-1);
    m_backSubsystem.setPower(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_resetPositionDegrees);
    m_frontSubsystem.stop();
    m_backSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
