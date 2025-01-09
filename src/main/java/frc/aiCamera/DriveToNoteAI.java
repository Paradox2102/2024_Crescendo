// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aiCamera;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.aiCamera.PhotonTracker.*;

public class DriveToNoteAI extends Command {
  /** Creates a new DriveToNoteAI. */

  private DriveSubsystem m_subsystem;
  private DoubleSupplier m_getX;
  private DoubleSupplier m_getY;
  private DoubleSupplier m_getRot;
  private Trigger m_slowMode;
  private Trigger m_slowMode1;
  private double gamePieceAngle; // angle robot should drive to (the gamepiece)
  private double gamePieceDistance;

  public DriveToNoteAI(DriveSubsystem driveSubsystem, DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getRot,
      Trigger slowMode, Trigger slowMode1) {
    m_subsystem = driveSubsystem;
    m_getX = getX;
    m_getY = getY;
    m_getRot = getRot;
    m_slowMode = slowMode;
    m_slowMode1 = slowMode1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  public void setGamePieceAngle(double angle) {
    gamePieceAngle = angle;
  }

  //
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveToNoteAI", 0, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -MathUtil.applyDeadband(m_getX.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double y = -MathUtil.applyDeadband(m_getY.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double rot = -MathUtil.applyDeadband(m_getRot.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    if (m_subsystem.shouldAim() && rot == 0) {
      rot = MathUtil.applyDeadband(m_subsystem.orientPID(m_subsystem.getFutureRotationalGoalFromTargetDegrees()), 0);
    }
    if (m_slowMode.getAsBoolean() || m_slowMode1.getAsBoolean()) {
      x *= .3;
      y *= .3;
    }
    if (gamePieceDistance > 2) { // if the gamepiece is greater than 2 meters away, drive towards it
      m_subsystem.drive(
          y * Math.sin(gamePieceAngle),
          x * Math.cos(gamePieceAngle),
          rot,
          true,
          true,
          Constants.DriveConstants.k_rotatePoint);
    } else { // if the gamepiece is less than 2 meters away, stop
      end();
    }

    // m_swerve.setModuleStates(m_defaultState);
    // System.out.println(String.format("x=%f, y=%f, rot=%f, isFieldRelative=%b", x,
    // y, rot, isFieldRelative));
  }

  // Called once the command ends or is interrupted.
  public void end() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
