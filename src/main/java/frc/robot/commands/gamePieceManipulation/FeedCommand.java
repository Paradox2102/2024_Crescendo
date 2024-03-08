// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class FeedCommand extends Command {
  ManipulatorSubsystem m_shooterSubsystem;
  ManipulatorSubsystem m_holderSubsystem;
  Timer m_dwellTimer = new Timer();

  /** Creates a new RevCommand. */
  public FeedCommand(ManipulatorSubsystem shooterSubsystem, ManipulatorSubsystem holderSubsystem) {
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
    if (Constants.States.m_shootIntakeSide) {
      m_shooterSubsystem.setVelocityRPM(Constants.States.m_speakerMode ? Constants.ShooterConstants.k_speakerShootVelocityRPM : Constants.ShooterConstants.k_ampShootVelocityRPM);
      m_holderSubsystem.setPower(Constants.States.m_speakerMode ? Constants.HolderConstants.k_speakerFeedPower : Constants.HolderConstants.k_ampFeedPower);
    } else {
      m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_speakerFeedPower);
      m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_speakerShootVelocityRPM);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.States.m_hasGamePiece) {
      m_dwellTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Shooter automatically stops in default manipulator command (not added here so can flow into amp shoot)
    m_dwellTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_dwellTimer.get() > .5;
  }
}
