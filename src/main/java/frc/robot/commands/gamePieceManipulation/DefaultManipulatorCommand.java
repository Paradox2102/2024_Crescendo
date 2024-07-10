// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ShooterSensors;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class DefaultManipulatorCommand extends Command {
  /** Creates a new DefaultStowGamePiece. */
  // sucks the game piece into the storage position. If the front sensor is broken
  // then the game piece is not in the correct position, but if the back one is
  // broken and the front is not then the piece is correctly stowed.
  ManipulatorSubsystem m_subsystem;
  DriveSubsystem m_driveSubsytem;
  ShooterSensors m_shooterSensors;

  private final double k_revRangeMeters = 10;
  private State m_state;

  private enum State {
    empty,
    intaking,
    holding
  };

  public DefaultManipulatorCommand(ManipulatorSubsystem subsystem, DriveSubsystem driveSubsystem,
      ShooterSensors shooterSensors) {
    m_subsystem = subsystem;
    m_driveSubsytem = driveSubsystem;
    m_shooterSensors = shooterSensors;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = State.empty;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {
      case empty:
        handleEmptyState();
        break;

      case intaking:
        handleIntakingState();
        break;

      case holding:
        handleHoldingState();
        break;
    }

  }

  void handleEmptyState() {
    m_subsystem.setFrontPower(0);
    m_subsystem.setBackPower(0);
    if (m_shooterSensors.getBackSensor() && !m_shooterSensors.getFrontSensor()) {
      m_state = State.holding;
    } else if (m_shooterSensors.getFrontSensor()) {
      m_state = State.intaking;
    }

  }

  void handleIntakingState() {
    m_subsystem.setFrontPower(Constants.ShooterConstants.k_adjustGamePiecePower);
    m_subsystem.setBackPower(Constants.HolderConstants.k_adjustGamePiecePower);
    if (m_shooterSensors.getBackSensor() && !m_shooterSensors.getFrontSensor()) {
      m_state = State.holding;
    } else if (!m_shooterSensors.getBackSensor() && !m_shooterSensors.getFrontSensor()) {
      m_state = State.empty;
    }
  }

  void handleHoldingState() {
    if (m_driveSubsytem.getTranslationalDistanceFromSpeakerMeters() < k_revRangeMeters
        && Constants.States.m_autoRotateAim) {

      if (Constants.States.m_shootIntakeSide) {
        m_subsystem.setFrontVelocityRPM(Constants.States.m_speakerMode ? m_subsystem.getRevSpeed()
            : Constants.ShooterConstants.k_ampShootVelocityRPM);
        m_subsystem.stopBack();
      } else {
        m_subsystem.stopFront();
        m_subsystem.setBackVelocityRPM(-m_subsystem.getRevSpeed());
      }

    }else{
      m_subsystem.stopFront();
      m_subsystem.stopBack();
    }
    // tests if state should still be holding
    if (!(m_shooterSensors.getBackSensor() && !m_shooterSensors.getFrontSensor())) {
      if (m_shooterSensors.getFrontSensor()) {
        m_state = State.intaking;
      } else {
        m_state = State.empty;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}