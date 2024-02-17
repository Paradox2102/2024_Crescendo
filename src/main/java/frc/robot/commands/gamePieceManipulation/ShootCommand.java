// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  ShooterSubsystem m_shooterSubsystem;
  HolderSubsystem m_holderSubsystem;
  private Timer m_dwellTimer = new Timer();
  private boolean m_feeding = false;

  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    addRequirements(m_shooterSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.States.m_runningShooterAndHolder = true;
    m_dwellTimer.start();
    m_dwellTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if the game piece is acquired,
    // rev the shooter at different speeds depending on what side you are shooting
    // from and where you are aiming,
    // and feed the game piece in once the shooter reaches the target velocity
    // if (Constants.m_isGamePieceStowed) {
      if (Constants.States.m_speaker) {
        if (Constants.States.m_shootIntakeSide) {
          // shooting intake side to speaker
          m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_speakerShootVelocityRPM);
          if (m_shooterSubsystem.isReady()) {
            m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_speakerFeedVelocityRPM);
            m_feeding = true;
          }
        } else {
          // shooting holder side to speaker
          m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_speakerShootVelocityRPM);
          if (m_holderSubsystem.isReady()) {
            m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_speakerFeedVelocityRPM);
            m_feeding = true;
          }
        }
      } else {
        // shooting intake side to amp
        m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_ampShootVelocityRPM);
        if (m_shooterSubsystem.isReady()) {
          m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_ampFeedVelocityRPM);
          m_feeding = true;
        }
      }
    if (!m_feeding) {
      m_dwellTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_holderSubsystem.stop();
    Constants.States.m_runningShooterAndHolder = false;
    Constants.States.m_hasGamePiece = false;
    m_dwellTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_dwellTimer.get() > 2;
  }
}
