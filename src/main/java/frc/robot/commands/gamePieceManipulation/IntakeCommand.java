// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
  HolderSubsystem m_holderSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  PivotSubsystem m_pivotSubsystem;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(HolderSubsystem holderSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_holderSubsystem = holderSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    addRequirements(m_holderSubsystem, m_shooterSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_intakeVelocityRPM);
    m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_intakeVelocityRPM);
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
    Constants.m_runningShooterAndHolder = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_holderSubsystem.stop();
    m_shooterSubsystem.stop();
    m_pivotSubsystem.setPositionDegrees(0);
    Constants.m_runningShooterAndHolder = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.m_hasGamePiece;
  }
}