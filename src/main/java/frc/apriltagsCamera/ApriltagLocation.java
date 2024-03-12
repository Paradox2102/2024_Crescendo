package frc.apriltagsCamera;

public class ApriltagLocation {
  int m_tag;
  public double m_xMeters;
  public double m_yMeters;
  public double m_targetAngleDegrees;
  // public int m_invalidCount = 0;

  public ApriltagLocation(int tag, double x, double y, double angle) {
    m_tag = tag;
    m_xMeters = x;
    m_yMeters = y;
    m_targetAngleDegrees = angle;
  }
  // Should add a toString method and a toPose2d method. - Gavin
}
