// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aicamera;

import frc.robot.RobotContainer;
import java.util.List;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PositionTrackerPose;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

import frc.aiCamera.AiCamera;
import frc.apriltagsCamera.Logger;

public class FindLargestNote extends Command {
  /** Creates a new FindLargestNote. */
  AiCamera m_ai_camera;
  public FindLargestNote(PositionTrackerPose tracker) {
    m_ai_camera = new AiCamera(tracker);
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(m_driveSubsystem, m_backSubsystem, m_frontSubsystem, m_pivotSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // List coords = m_ai_camera.FindNotePositions();
    // Logger.log("Largest note position", 3, ""+coords);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
