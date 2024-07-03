// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import java.util.concurrent.CyclicBarrier;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class DefaultManipulatorCommand extends Command {
  /** Creates a new DefaultStowGamePiece. */
  // sucks the game piece into the storage position. If the front sensor is broken
  // then the game piece is not in the correct position, but if the back one is
  // broken and the front is not then the piece is correctly stowed.
  ManipulatorSubsystem m_subsystem;
  DriveSubsystem m_driveSubsytem;
  boolean m_isFront;
  private final double k_revRangeMeters = 10;
  private State m_state;

  private enum State {
    empty,
    intaking,
    holding
  };

  public DefaultManipulatorCommand(ManipulatorSubsystem subsystem, DriveSubsystem driveSubsystem, boolean isFront) {
    m_subsystem = subsystem;
    m_driveSubsytem = driveSubsystem;
    m_isFront = isFront;

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
void handleEmptyState(){

}
void handleIntakingState(){

}
void handleHoldingState(){
  
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