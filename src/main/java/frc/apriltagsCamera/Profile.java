package frc.apriltagsCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Profile {
    String m_name;
    long m_startTime;
    int m_totalTime = 0;
    long m_intervalTime = 0;

    public Profile(String name) {
        m_name = name;
    }

    public void start()
    {
        if (m_intervalTime == 0) {
            m_intervalTime = System.currentTimeMillis();
        }

        m_startTime = System.nanoTime();
    }

    public void end()
    {
        long time = System.currentTimeMillis();
        long dt = (time - m_intervalTime);
        int delta = (int) ((System.nanoTime() - m_startTime) / 1000);

        m_totalTime += delta;

        if (dt >= 1000) {
            SmartDashboard.putNumber(String.format("P-%s", m_name), 20.0 * m_totalTime / (1000 * dt));
            m_totalTime = 0;
            m_intervalTime = time;
        }
    }   
}
