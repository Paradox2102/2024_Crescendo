// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultStowGamePiece extends Command {
  /** Creates a new DefaultStowGamePiece. */
  ShooterSubsystem m_shooterSubsystem;
  HolderSubsystem m_holderSubsystem;
  public DefaultStowGamePiece(ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (Constants.States.m_hasGamePiece && !Constants.States.m_isGamePieceStowed) {
      m_shooterSubsystem.setPower(Constants.States.m_shootIntakeSide ? Constants.ShooterConstants.k_adjustGamePiecePower : -Constants.ShooterConstants.k_adjustGamePiecePower);
      m_holderSubsystem.setPower(Constants.States.m_shootIntakeSide ? Constants.HolderConstants.k_adjustGamePiecePower : -Constants.HolderConstants.k_adjustGamePiecePower);
      System.out.println("yay");
    } else {
      m_shooterSubsystem.stop();
      m_holderSubsystem.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
