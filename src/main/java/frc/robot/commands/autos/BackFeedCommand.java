// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class BackFeedCommand extends Command {
  ManipulatorSubsystem m_frontSubsystem;
  Timer m_dwellTimer = new Timer();

  /** Creates a new RevCommand. */
  public BackFeedCommand(ManipulatorSubsystem frontSubsystem) {
    m_frontSubsystem = frontSubsystem;
    m_dwellTimer.reset();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dwellTimer.reset();
    m_dwellTimer.start();
    m_frontSubsystem.setPower(Constants.FrontConstants.k_speakerFeedPower);
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
    m_frontSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_dwellTimer.get() > .25;
  }
}
