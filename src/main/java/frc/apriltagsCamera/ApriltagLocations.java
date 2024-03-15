// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.apriltagsCamera;

/** All distances are in meters and all angles in degrees */
public class ApriltagLocations {
  public static double k_inPerM = 39.3701; // number of inches in a meter
  // Some of these locations are being changed to reflect the realities of
  // our practice field. We MUST make sure we use the correct values for the
  // competition field. - Gavin
  public static ApriltagLocation m_tags[] = {
      // new ApriltagLocation(7, 0, (323.0/2) / k_inPerM, 0 - 180),
      // new ApriltagLocation(1, 593.68 / k_inPerM, (9.68) / k_inPerM, 120 - 180), //
      // Official
      // new ApriltagLocation(2, 637.21 / k_inPerM, (34.79) / k_inPerM, 120 - 180), //
      // Official
      new ApriltagLocation(1, 587.06 / k_inPerM, (4.95) / k_inPerM, 120 - 180),
      new ApriltagLocation(2, 637.21 / k_inPerM, (29.81) / k_inPerM, 120 - 180),
      new ApriltagLocation(3, 652.73 / k_inPerM, (196.17) / k_inPerM, 180 - 180),
      new ApriltagLocation(4, 652.73 / k_inPerM, (218.42) / k_inPerM, 180 - 180),
      new ApriltagLocation(5, 578.77 / k_inPerM, (323.00) / k_inPerM, 270 - 180),
      new ApriltagLocation(6, 72.5 / k_inPerM, (323.00) / k_inPerM, 270 - 180),
      new ApriltagLocation(7, -1.5 / k_inPerM, (218.42) / k_inPerM, 0 - 180),
      new ApriltagLocation(8, -1.50 / k_inPerM, (196.17) / k_inPerM, 0 - 180),
      new ApriltagLocation(9, 14.02 / k_inPerM, (34.79) / k_inPerM, 60 - 180),
      new ApriltagLocation(10, 57.54 / k_inPerM, (9.68) / k_inPerM, 60 - 180),
      new ApriltagLocation(11, 468.69 / k_inPerM, (146.19) / k_inPerM, 300 - 180),
      new ApriltagLocation(12, 468.69 / k_inPerM, (177.10) / k_inPerM, 60 - 180),
      new ApriltagLocation(13, 441.74 / k_inPerM, (161.62) / k_inPerM, 180 - 180), // Official
      new ApriltagLocation(14, 209.48 / k_inPerM, (161.62) / k_inPerM, 0 - 180), // Official
      // new ApriltagLocation(15, 182.73 / k_inPerM, (177.10) / k_inPerM, 120 - 180),
      // // Official
      new ApriltagLocation(15, 182.73 / k_inPerM, (177.10) / k_inPerM, 130 - 180),
      new ApriltagLocation(16, 182.73 / k_inPerM, (146.19) / k_inPerM, 240 - 180),
  };
  public static boolean m_blue = false;
  public static double m_fieldLength = 8.21;
  public static double m_fieldWidth = 16.54;

  public static void setColor(boolean blue) {
    // MUSTFIX

    // if (blue != m_blue)
    // {
    // m_blue = blue;

    // for (ApriltagLocation tag : m_tags)
    // {
    // // tag.m_targetAngleDegrees =
    // ParadoxField.normalizeAngle(tag.m_targetAngleDegrees + 180);
    // tag.m_xInches = -tag.m_xInches;
    // tag.m_yInches = m_fieldLength - tag.m_yInches;
    // }
    // }
  }

  public static ApriltagLocation findTag(int id) {
    for (ApriltagLocation tag : m_tags) {
      if (tag.m_tag == id) {
        return (tag);
      }
    }

    return (null);
  }
}
