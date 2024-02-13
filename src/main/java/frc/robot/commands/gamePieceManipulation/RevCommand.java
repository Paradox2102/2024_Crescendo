// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RevCommand extends Command {
  ShooterSubsystem m_shooterSubsystem;
  HolderSubsystem m_holderSubsystem;

  /** Creates a new RevCommand. */
  public RevCommand(ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.m_runningShooterAndHolder = true;
    if (Constants.m_shootIntakeSide) {
      m_shooterSubsystem.setVelocityRPM(Constants.m_speaker ? Constants.ShooterConstants.k_speakerShootVelocityRPM : Constants.ShooterConstants.k_ampShootVelocityRPM);
    } else {
      m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_speakerShootVelocityRPM);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_holderSubsystem.stop();
    Constants.m_runningShooterAndHolder = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.m_runningShooterAndHolder;
  }
}
