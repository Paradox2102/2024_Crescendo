// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AimArcadeDrive extends Command {
     /** Creates a new ArcadeDrive. */

  private DriveSubsystem m_subsystem;
  private DoubleSupplier m_getX;
  private DoubleSupplier m_getY;

  public AimArcadeDrive(DriveSubsystem driveSubsystem, DoubleSupplier getX, DoubleSupplier getY) {
    m_subsystem = driveSubsystem;
    m_getX = getX;
    m_getY = getY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
  //
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDrive", 0, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -MathUtil.applyDeadband(m_getX.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double y = -MathUtil.applyDeadband(m_getY.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double rot = MathUtil.applyDeadband(m_subsystem.orientPID(m_subsystem.getFutureRotationalDistanceFromCornersDegrees()), 0);
    m_subsystem.drive(
      y, 
      x, 
      rot, 
      true, 
      true,
      Constants.DriveConstants.k_rotatePoint
    );

    
    // m_swerve.setModuleStates(m_defaultState);
    // System.out.println(String.format("x=%f, y=%f, rot=%f, isFieldRelative=%b", x, y, rot, isFieldRelative)); 
  }
  
  

  // Called once the command ends or is interrupted.
  public void end() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
