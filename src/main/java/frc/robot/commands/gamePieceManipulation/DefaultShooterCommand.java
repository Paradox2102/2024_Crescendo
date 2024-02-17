// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends Command {
  /** Creates a new DefaultShooterCommand. */
  ShooterSubsystem m_subsystem;
  public DefaultShooterCommand(ShooterSubsystem shooterSubsystem) {
    m_subsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.States.m_isGamePieceStowed && Constants.States.m_faceSpeaker && Constants.States.m_speakerMode) {
      m_subsystem.setVelocityRPM(Constants.ShooterConstants.k_speakerShootVelocityRPM);
    } else {
      m_subsystem.setVelocityRPM(0);
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
