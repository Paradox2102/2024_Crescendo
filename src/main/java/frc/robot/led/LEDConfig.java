package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.apriltagsCamera.ApriltagsCamera;
import frc.robot.Robot;
import frc.robot.led.commands.Blinker;
import frc.robot.led.commands.ParadoxAnim;
import frc.robot.led.commands.RainbowAnim;
import frc.robot.led.subsystems.LEDSubsystem;

public class LEDConfig {
    private final LEDSubsystem m_string1 = new LEDSubsystem(0, 45);
    private final LEDSubsystem m_string2 = new LEDSubsystem(45, 46, true);
    private final LEDSubsystem m_string3 = new LEDSubsystem(45 + 46, 30);
    private final ApriltagsCamera m_camera;
    private final Robot m_robot;

    private Command m_currentCommand = null;

    private ParallelCommandGroup m_paradox = new ParallelCommandGroup(new ParadoxAnim(m_string1, 0.1),
            new ParadoxAnim(m_string2, 0.1), new ParadoxAnim(m_string3, 0.1));
    private ParallelCommandGroup m_rainbow = new ParallelCommandGroup(new RainbowAnim(m_string1),
            new RainbowAnim(m_string2), new RainbowAnim(m_string3));
    private ParallelCommandGroup m_cameraFailure = new ParallelCommandGroup(new Blinker(m_string1, 0.5, Color.kRed),
            new Blinker(m_string2, .5, Color.kRed), new Blinker(m_string3, 0.5, Color.kRed));

    public LEDConfig(Robot robot, ApriltagsCamera camera) {
        m_robot = robot;
        m_camera = camera;
    }

    public void periodic() {
        Command command = null;

        if (!m_camera.isConnected()) {
            command = m_cameraFailure;
        } else if (m_camera.isTagVisible() && m_robot.isDisabled()) {
            command = m_paradox;
        } else {
            command = m_rainbow;
        }

        if (command != m_currentCommand) {
            m_currentCommand = command;
            m_currentCommand.schedule();
        }
    }
}
