// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeAndGoToBackShooter extends Command {
  /** Creates a new IntakeAndGoToBackShooter. */
  ShooterSubsystem m_shooterSubsystem;
  PivotSubsystem m_pivotSubsystem;
  double m_endPos;
  public IntakeAndGoToBackShooter(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, double endPos) {
    m_shooterSubsystem = shooterSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_endPos = endPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.States.m_runningShooterAndHolder = true;
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
    m_shooterSubsystem.setVelocityRPM(1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_pivotSubsystem.setPositionDegrees(m_pivotSubsystem.getPivotAngleFromDistanceFromSpeaker(1.8));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.States.m_hasGamePiece;
  }
}
