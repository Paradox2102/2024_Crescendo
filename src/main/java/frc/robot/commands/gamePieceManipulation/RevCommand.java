// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.Constants;

public class RevCommand extends Command {
  ManipulatorSubsystem m_shooterSubsystem;
  ManipulatorSubsystem m_holderSubsystem;

  /** Creates a new RevCommand. */
  public RevCommand(ManipulatorSubsystem shooterSubsystem, ManipulatorSubsystem holderSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.States.m_shootIntakeSide) {
      m_shooterSubsystem
          .setVelocityRPM(Constants.States.m_speakerMode ? Constants.ShooterConstants.k_speakerShootVelocityRPM
              : Constants.ShooterConstants.k_ampShootVelocityRPM);
      m_holderSubsystem.stop();
    } else {
      m_shooterSubsystem.stop();
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
    System.out.println("rev command end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
