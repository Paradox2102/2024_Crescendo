// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;


import java.io.ObjectInputStream.GetField;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElasticChangeDegrees extends InstantCommand {
 private final PivotSubsystem m_subsystem;

  public ElasticChangeDegrees(PivotSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ElasticChangeDegrees", 1, "initialize");
    double degree = SmartDashboard.getNumber("Get Degrees", Constants.PivotConstants.k_resetPositionDegrees);
    SmartDashboard.putNumber("Currrent Degrees", degree);
    m_subsystem.setPositionDegrees(degree);
  }
}
