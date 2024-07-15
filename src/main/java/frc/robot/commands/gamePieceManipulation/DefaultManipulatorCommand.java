// Copyright (c) FIRST and other WPILib contributors.
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
  Boolean m_front;

  private final double k_revRangeMeters = 10;
  // private State m_state;

  // private enum State {
  //   empty,
  //   intaking,
  //   holding
  // };

  public DefaultManipulatorCommand(ManipulatorSubsystem subsystem, DriveSubsystem driveSubsystem, ShooterSensors shooterSensors, boolean front) {
    m_subsystem = subsystem;
    m_driveSubsytem = driveSubsystem;
    m_shooterSensors = shooterSensors;
    m_front = front;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_state = State.empty;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Constants.States.m_shootIntakeSide ^ m_front) {
      // This motor is the holding motor
      // If the shooter sensor is broken and the holder sensor is not broken this motor should be set to intake
      // Otherwise turn this motor off
    } else {
      // This motor is the shooting motor.
      // If the shooter sensor is broken this motor should be set to intake
      // If the shooter sensor is not broken and the holder sensor is broken and it is within shooting range, spin up shooter
      // Otherwise turn this motor off
    }

    // switch (m_state) {
    //   case empty:
    //     handleEmptyState();
    //     break;

    //   case intaking:
    //     handleIntakingState();
    //     break;

    //   case holding:
    //     handleHoldingState();
    //     break;
    // }

  }

  // void handleEmptyState() {
  //   m_subsystem.setShooterPower(0);
  //   m_subsystem.setHolderPower(0);
  //   if (m_shooterSensors.getHolderSensor() && !m_shooterSensors.getShooterSensor()) {
  //     m_state = State.holding;
  //   } else if (m_shooterSensors.getShooterSensor()) {
  //     m_state = State.intaking;
  //   }

  // }

  // void handleIntakingState() {
  //   m_subsystem.setShooterPower(Constants.ShooterConstants.k_adjustGamePiecePower);
  //   m_subsystem.setHolderPower(Constants.HolderConstants.k_adjustGamePiecePower);
  //   if (m_shooterSensors.getHolderSensor() && !m_shooterSensors.getShooterSensor()) {
  //     m_state = State.holding;
  //   } else if (!m_shooterSensors.getHolderSensor() && !m_shooterSensors.getShooterSensor()) {
  //     m_state = State.empty;
  //   }
  // }

  // void handleHoldingState() {
  //   if (m_driveSubsytem.getTranslationalDistanceFromSpeakerMeters() < k_revRangeMeters
  //       && Constants.States.m_autoRotateAim) {

  //     if (Constants.States.m_shootIntakeSide) {
  //       m_subsystem.setShooterVelocityRPM(Constants.States.m_speakerMode ? m_subsystem.getRevSpeed()
  //           : Constants.ShooterConstants.k_ampShootVelocityRPM);
  //     } else {
  //       m_subsystem.setShooterVelocityRPM(-m_subsystem.getRevSpeed());
  //     }
  //      m_subsystem.stopHolder();

  //   } else {
  //     m_subsystem.stopShooter();
  //     m_subsystem.stopHolder();
  //   }
  //   // tests if state should still be holding
  //   if (!(m_shooterSensors.getHolderSensor() && !m_shooterSensors.getShooterSensor())) {
  //     if (m_shooterSensors.getShooterSensor()) {
  //       m_state = State.intaking;
  //     } else {
  //       m_state = State.empty;
  //     }
  //   }

  // }

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