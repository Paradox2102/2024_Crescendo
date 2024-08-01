// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class ElasticChangeDegree extends Command {
  private final PivotSubsystem m_subsystem;

  /** Creates a new ElasticChangeDegree. */
  public ElasticChangeDegree(PivotSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ElasticChangeDegrees", 1, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degree = SmartDashboard.getNumber("Get Degrees", Constants.PivotConstants.k_resetPositionDegrees);
    SmartDashboard.putNumber("Currrent Degrees", degree);
    m_subsystem.setPositionDegrees(degree);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ElasticChangeDegree", 1, "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
