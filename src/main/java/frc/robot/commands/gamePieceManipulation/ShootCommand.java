// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  ShooterSubsystem m_shooterSubsystem;
  HolderSubsystem m_holderSubsystem;
  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.m_isGamePieceStowed) {
      if(Constants.m_speaker) {
        if(Constants.m_shootIntakeSide){
          m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_speakerVelocityRPM);
        } else {
          m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_speakerVelocityRPM);
        }
      } else {
        if(Constants.m_shootIntakeSide){
          m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_ampVelocityRPM);
        } else {
          m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_ampVelocityRPM);
        }
      }
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
