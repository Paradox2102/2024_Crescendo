// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class SlowTurn extends Command {
  DriveSubsystem m_subsystem;
  ApriltagsCamera m_camera;

  /** Creates a new SlowTurn. */
  public SlowTurn(DriveSubsystem subsystem, ApriltagsCamera camera) {
    Logger.log("SlowTurn", 3, "SlowTurn()");

    m_subsystem = subsystem;
    m_camera = camera;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SlowTurn", 2, "initialize()");
    ApriltagsCamera.setLogging(true);

    m_subsystem.drive(
      0, 
      0, 
      0.15, 
      true, 
      true,
      Constants.DriveConstants.k_rotatePoint
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SlowTurn", 2, String.format("end(%b)", interrupted));

    ApriltagsCamera.setLogging(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
