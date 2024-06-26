// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPivotOffRobotLocation extends InstantCommand {
  PivotSubsystem m_subsystem;
  public SetPivotOffRobotLocation(PivotSubsystem pivotSubsystem) {
    m_subsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.States.m_speakerMode) {
      m_subsystem.setPositionDegrees(m_subsystem.getPivotAngleFromRobotPos(false));
      System.out.println(m_subsystem.getPivotAngleFromRobotPos(false));
    }
  }
}
