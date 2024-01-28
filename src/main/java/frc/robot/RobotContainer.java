// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoOrientCommand;
import frc.robot.commands.AutoPickUpGamePiece;
import frc.robot.commands.Autos;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RevCommand;
import frc.robot.commands.test.D2Intake;
import frc.robot.commands.test.IncrementPivotCommand;
import frc.robot.commands.test.TestPivot;
import frc.robot.commands.test.TestPivotPID;
import frc.robot.commands.test.TestShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final HolderSubsystem m_holderSubsystem = new HolderSubsystem();

  private final CommandJoystick m_joystick = new CommandJoystick(1);
  public final PositionTrackerPose m_tracker = new PositionTrackerPose(0, 0, m_driveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveSubsystem.setTracker(m_tracker);
    NamedCommands.registerCommand("shoot", new TestShooter(m_shooterSubsystem, m_holderSubsystem, m_pivotSubsystem, true));
    NamedCommands.registerCommand("intake", new TestShooter(m_shooterSubsystem, m_holderSubsystem, m_pivotSubsystem, false));
    NamedCommands.registerCommand("rev shooter", new RevCommand(m_shooterSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(
      m_driveSubsystem, 
      () -> m_driverController.getLeftX(), 
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getRightX()
    ));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.rightTrigger().whileTrue(new TestShooter(m_shooterSubsystem, m_holderSubsystem, m_pivotSubsystem, false));
    m_driverController.leftTrigger().whileTrue(new TestShooter(m_shooterSubsystem, m_holderSubsystem, m_pivotSubsystem, true));
    m_driverController.leftBumper().toggleOnTrue(new TestPivotPID(m_pivotSubsystem));
    m_driverController.rightTrigger().whileTrue(new AutoPickUpGamePiece(m_driveSubsystem, m_pivotSubsystem, m_shooterSubsystem, m_holderSubsystem, () -> m_driverController.getLeftY(), () -> m_driverController.getLeftX(), () -> m_driverController.getRightX()));

    m_driverController.y().onTrue(new AutoOrientCommand(m_driveSubsystem, 180, () -> -m_driverController.getLeftY(), () -> m_driverController.getLeftX()));
    m_driverController.a().onTrue(new AutoOrientCommand(m_driveSubsystem, 0, () -> -m_driverController.getLeftY(), () -> m_driverController.getLeftX()));
    m_driverController.b().onTrue(new AutoOrientCommand(m_driveSubsystem, 90, () -> -m_driverController.getLeftY(), () -> m_driverController.getLeftX()));
    m_driverController.x().onTrue(new AutoOrientCommand(m_driveSubsystem, -90, () -> -m_driverController.getLeftY(), () -> m_driverController.getLeftX()));

    m_driverController.povUp().onTrue(new ResetGyro(m_driveSubsystem, 180));
    m_driverController.povDown().onTrue(new ResetGyro(m_driveSubsystem, 0));
    m_driverController.povRight().onTrue(new ResetGyro(m_driveSubsystem, -90));
    m_driverController.povLeft().onTrue(new ResetGyro(m_driveSubsystem, 90));

    m_joystick.button(1).whileTrue(new RevCommand(m_shooterSubsystem));
    m_joystick.button(2).whileTrue(new D2Intake(m_shooterSubsystem, m_holderSubsystem, true));
    m_joystick.button(6).onTrue(new IncrementPivotCommand(m_pivotSubsystem, true));
    m_joystick.button(4).onTrue(new IncrementPivotCommand(m_pivotSubsystem, false));
  }

  public double getThrottle() {
    return m_joystick.getThrottle();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("wing 4 piece");
  }
}
