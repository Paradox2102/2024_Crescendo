// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class DefaultPivotCommand extends Command {
  /** Creates a new DefaultPivotCommand. */
  PivotSubsystem m_subsystem;
  boolean m_predictFuture;
  DriveSubsystem m_driveSubsystem;
  public DefaultPivotCommand(PivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem, boolean predictFuture) {
    m_predictFuture = predictFuture;
    m_subsystem = pivotSubsystem;
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // I think this code is doing the following:
    //     If we don't have a gamepiece:
    //         Go to reset posiion
    //     Else if we're not in speaker mode:
    //         Go to amp position
    //     Else if we're close enough to the speaker and "auto rotate aim" is set:
    //         Position using distance from speaker* and then mysteriously add one degree
    //     Else:
    //         Position using distance from speaker*
    // * getPivotAngleFromRobotPos returns reset position over a certain distance.
    // - Gavin
    if (!Constants.States.m_speakerMode && Constants.States.m_hasGamePiece) {
      m_subsystem.setPositionDegrees(Constants.PivotConstants.k_ampPositionDegrees);
    } else if (Constants.States.m_speakerMode && m_driveSubsystem.shouldAimSpeaker()) {
      // doesn't need to check aiming zone because getPivotAngleFromRobotPos() does it for you
      m_subsystem.setPositionDegrees(m_subsystem.getPivotAngleFromRobotPos(m_predictFuture) + 1);
    } else if (Constants.States.m_hasGamePiece) {
      m_subsystem.setPositionDegrees(m_subsystem.getPivotAngleFromRobotPos(m_predictFuture));
    } else {
      // m_subsystem.setPositionDegrees(Constants.PivotConstants.k_sourceAngle);
      m_subsystem.setPositionDegrees(Constants.PivotConstants.k_resetPositionDegrees);
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
