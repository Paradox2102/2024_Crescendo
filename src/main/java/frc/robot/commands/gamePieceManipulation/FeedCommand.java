// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedCommand extends Command {
  ShooterSubsystem m_shooterSubsystem;
  HolderSubsystem m_holderSubsystem;
  Timer m_dwellTimer = new Timer();

  /** Creates a new RevCommand. */
  public FeedCommand(ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    m_dwellTimer.reset();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dwellTimer.reset();
    m_dwellTimer.start();
    Constants.m_runningShooterAndHolder = true;
    if (Constants.m_shootIntakeSide) {
      m_shooterSubsystem.setVelocityRPM(Constants.m_speaker ? Constants.ShooterConstants.k_speakerShootVelocityRPM : Constants.ShooterConstants.k_ampShootVelocityRPM);
      m_holderSubsystem.setVelocityRPM(Constants.m_speaker ? Constants.HolderConstants.k_speakerFeedVelocityRPM : Constants.HolderConstants.k_ampFeedVelocityRPM);
    } else {
      m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_speakerFeedVelocityRPM);
      m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_speakerShootVelocityRPM);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.m_hasGamePiece) {
      m_dwellTimer.reset();
    }
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
    return m_dwellTimer.get() > .5;
  }
}