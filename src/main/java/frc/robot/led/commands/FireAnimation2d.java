// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led.commands;

import java.util.Random;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.Logger;
import frc.robot.led.subsystems.LEDSubsystem;

public class FireAnimation2d extends Command {
  private final LEDSubsystem m_subsystem;
  private final int heat[][];
  private final int m_width;
  private final int m_height;
  private final Random m_random = new Random();
  private final boolean gReverseDirection = false;

  // COOLING: How much does the air cool as it rises?
  // Less cooling = taller flames. More cooling = shorter flames.
  // Default 50, suggested range 20-100
  private final static int COOLING = 55;

  // SPARKING: What chance (out of 255) is there that a new spark will be lit?
  // Higher chance = more roaring fire. Lower chance = more flickery fire.
  // Default 120, suggested range 50-200.
  private final static int SPARKING = 120;

  /** Creates a new FireAnimation2d. */
  public FireAnimation2d(LEDSubsystem subsystem) {
    Logger.log("FireAnimation2d", 3, "FireAnimation2d()");

    m_subsystem = subsystem;
    m_width = m_subsystem.getWidth();
    m_height = m_subsystem.getHeight();
    heat = new int[m_subsystem.getHeight()][m_subsystem.getWidth()];
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  int random8(int min, int max) {
    return m_random.nextInt(max - min) + min;
  }

  int random8() {
    return m_random.nextInt(256);
  }

  int random8(int max) {
    return m_random.nextInt(max);
  }

  int qadd8(int i, int j) {
    int t = i + j;
    if (t > 255)
      t = 255;
    return t;
  }

  int qsub8(int i, int j) {
    int t = i - j;
    if (t < 0)
      t = 0;
    return t;
  }

  int scale8_video( int i, int scale)
{
   int j = (((int)i * (int)scale) >> 8) + (((i != 0) && (scale != 0))?1:0);
    return j;
}

  Color HeatColor(int temperature) {
    Color heatColor;

    // Scale 'heat' down from 0-255 to 0-191,
    // which can then be easily divided into three
    // equal 'thirds' of 64 units each.
    int t192 = scale8_video(temperature, 192);

    // calculate a value that ramps up from
    // zero to 255 in each 'third' of the scale.
    int heatramp = t192 & 0x3F; // 0..63
    heatramp <<= 2; // scale up to 0..252

    // now figure out which third of the spectrum we're in:
    if ((t192 & 0x80) != 0) {
      // we're in the hottest third
      heatColor = new Color(255, 255, heatramp);
      // heatcolor.r = 255; // full red
      // heatcolor.g = 255; // full green
      // heatcolor.b = heatramp; // ramp up blue

    } else if ((t192 & 0x40) != 0) {
      // we're in the middle third
      heatColor = new Color(255, heatramp, 0);
      // heatcolor.r = 255; // full red
      // heatcolor.g = heatramp; // ramp up green
      // heatcolor.b = 0; // no blue

    } else {
      // we're in the coolest third
      heatColor = new Color(heatramp, 0, 0);
      // heatcolor.r = heatramp; // ramp up red
      // heatcolor.g = 0; // no green
      // heatcolor.b = 0; // no blue
    }

    return heatColor;
  }

  void fire(int row) {
    // Array of temperature readings at each simulation cell

    // Step 1. Cool down every cell a little
    for (int i = 0; i < m_width; i++) {
      heat[row][i] = qsub8(heat[row][i], random8(0, ((COOLING * 10) / m_width) + 2));
    }

    // Step 2. Heat from each cell drifts 'up' and diffuses a little
    for (int k = m_width - 1; k >= 2; k--) {
      heat[row][k] = (heat[row][k - 1] + heat[row][k - 2] + heat[row][k - 2]) / 3;
    }

    // Step 3. Randomly ignite new 'sparks' of heat near the bottom
    if (random8() < SPARKING) {
      int y = random8(7);
      heat[row][y] = qadd8(heat[row][y], random8(160, 255));
    }

    // Step 4. Map from heat cells to LED colors
    for (int j = 0; j < m_width; j++) {
      Color color = HeatColor(heat[row][j]);
      int pixelnumber;
      if (gReverseDirection) {
        pixelnumber = (m_width - 1) - j;
      } else {
        pixelnumber = j;
      }
      m_subsystem.setLED(pixelnumber, row, color);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("FireAnimation2d", 2, "initialize()");
  }

  int m_count;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if ((++m_count % 2) == 0) {
      for (int row = 0 ; row < m_height ; row++) {
        fire(row);
      }
      m_subsystem.commit();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("FireAnimation2d", 2, String.format("end(%b)", interrupted));

    m_subsystem.setAllLEDs(Color.kBlack);
    m_subsystem.commit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
