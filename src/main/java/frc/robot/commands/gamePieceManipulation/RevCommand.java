// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class RevCommand extends Command {
  ManipulatorSubsystem m_manipulatorSubsystem;
  ManipulatorSubsystem m_holderSubsystem;

  /** Creates a new RevCommand. */
  public RevCommand(ManipulatorSubsystem manipulatorSubsystem, ManipulatorSubsystem holderSubsystem) {
    m_manipulatorSubsystem = manipulatorSubsystem;
    m_holderSubsystem = holderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_manipulatorSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("revCommand initialize");
    Constants.States.m_runningShooterAndHolder = true;
    if (Constants.States.m_shootIntakeSide) {
      m_manipulatorSubsystem.setVelocityRPM(Constants.States.m_speakerMode ? Constants.ShooterConstants.k_speakerShootVelocityRPM : Constants.ShooterConstants.k_ampShootVelocityRPM);
      m_holderSubsystem.setVelocityRPM(0);
    } else {
      m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_speakerShootVelocityRPM);
      m_manipulatorSubsystem.setVelocityRPM(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_ManipulatorSubsystem.stop();
    // m_holderSubsystem.stop();
    Constants.States.m_runningShooterAndHolder = false;
    System.out.println("rev command end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
