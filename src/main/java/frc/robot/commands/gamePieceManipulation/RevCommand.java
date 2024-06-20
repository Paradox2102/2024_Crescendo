// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class RevCommand extends Command {
  ManipulatorSubsystem m_frontSubsystem;
  ManipulatorSubsystem m_backSubsystem;

  /** Creates a new RevCommand. */
  public RevCommand(ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem) {
    m_frontSubsystem = frontSubsystem;
    m_backSubsystem = backSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontSubsystem, m_backSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("revCommand initialize");
    if (Constants.States.m_shootIntakeSide) {
      m_frontSubsystem.setVelocityRPM(Constants.States.m_speakerMode ? Constants.FrontConstants.k_speakerShootVelocityRPM : Constants.FrontConstants.k_ampShootVelocityRPM);
      m_backSubsystem.setVelocityRPM(0);
    } else {
      m_backSubsystem.setVelocityRPM(Constants.BackConstants.k_speakerShootVelocityRPM);
      m_frontSubsystem.setVelocityRPM(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
