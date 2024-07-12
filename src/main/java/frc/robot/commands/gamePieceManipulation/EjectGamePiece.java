// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants;

public class EjectGamePiece extends Command {
  /** Creates a new EjectGamePiece. */
  /* Gets rid of the game piece and ejects it away from the robot */
  PivotSubsystem m_pivotSubsystem;
  ManipulatorSubsystem m_shooterSubsystem;
  ManipulatorSubsystem m_holderSubsystem;

  public EjectGamePiece(PivotSubsystem pivotSubsystem, ManipulatorSubsystem shooterSubsystem,
      ManipulatorSubsystem holderSubsystem) {
    m_pivotSubsystem = pivotSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivotSubsystem, m_shooterSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotSubsystem.setPositionDegrees(70);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_holderSubsystem.setPower(-1);
    // m_shooterSubsystem.setPower(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_shooterSubsystem.stop();
    // m_holderSubsystem.stop();
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_resetPositionDegrees);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
