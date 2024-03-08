// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class DefaultManipulatorCommand extends Command {
  /** Creates a new DefaultStowGamePiece. */
  ManipulatorSubsystem m_subsystem;
  DriveSubsystem m_driveSubsytem;
  boolean m_shooter;
  private final double k_revRangeMeters = 10;
  public DefaultManipulatorCommand(ManipulatorSubsystem subsystem, DriveSubsystem driveSubsystem, boolean shooter) {
    m_subsystem = subsystem;
    m_driveSubsytem = driveSubsystem;
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.States.m_hasGamePiece && !Constants.States.m_isGamePieceStowed) {
      double adjustPower = m_shooter ? Constants.ShooterConstants.k_adjustGamePiecePower : Constants.HolderConstants.k_adjustGamePiecePower;
      m_subsystem.setPower(Constants.States.m_shootIntakeSide ? -adjustPower : adjustPower);
    } else if (Constants.States.m_isGamePieceStowed && m_driveSubsytem.getTranslationalDistanceFromSpeakerMeters() < k_revRangeMeters && Constants.States.m_autoRotateAim) {
      if (m_shooter) {
        m_subsystem.setVelocityRPM(Constants.States.m_speakerMode ? Constants.ShooterConstants.k_speakerShootVelocityRPM : Constants.ShooterConstants.k_ampShootVelocityRPM);
      } else {
        m_subsystem.setVelocityRPM(Constants.HolderConstants.k_speakerShootVelocityRPM);
      }
    } else {
      m_subsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
