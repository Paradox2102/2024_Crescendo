// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.stick.SetStickPos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StickSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequence extends SequentialCommandGroup {
  /** Creates a new ShootSequence. */
  public ShootSequence(ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem, StickSubsystem stickSubsystem, CommandXboxController joystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        // new DriveToPosition(driveSubsystem, () -> Constants.States.m_ampPos),
        new InstantCommand(), 
        new InstantCommand(() -> {Constants.States.m_slowMode = false;}), //false
        () -> !Constants.States.m_speakerMode
      ),
      new ConditionalCommand(
        new ParallelDeadlineGroup(
          new ShootCommand(frontSubsystem, backSubsystem),
          new SetStickPos(stickSubsystem, true)
        ),
        new ShootCommand(frontSubsystem, backSubsystem), 

        () -> !Constants.States.m_slowMode
      ),
      new InstantCommand(() -> {Constants.States.m_slowMode = false;})
    );
  }
}
