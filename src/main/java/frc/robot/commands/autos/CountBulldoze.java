// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class CountBulldoze extends Command {
  boolean m_lastState = false;
  ManipulatorSubsystem m_shooterSubsytem;
  ManipulatorSubsystem m_holderSubsystem;
  PivotSubsystem m_pivotSubsystem;
  int m_gamePiecesBulldozed = 0;
  /** Creates a new CountBulldoze. */
  public CountBulldoze(ManipulatorSubsystem shooterSubsystem, ManipulatorSubsystem holderSubsystem, PivotSubsystem pivotSubsystem) {
    m_shooterSubsytem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsytem, m_holderSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamePiecesBulldozed = 0;
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
    m_shooterSubsytem.setVelocityRPM(Constants.ShooterConstants.k_intakeVelocityRPM);
    m_holderSubsystem.setVelocityRPM(Constants.ShooterConstants.k_intakeVelocityRPM);
    // yes it's ShooterConstants for holder on purpose
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
    m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_intakeVelocityRPM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamePiecesBulldozed >= 4;
  }
}
