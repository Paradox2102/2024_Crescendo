package frc.apriltagsCamera;

import java.util.Timer;
import java.util.TimerTask;

import frc.apriltagsCamera.Network.NetworkReceiver;

public class PositionServer implements NetworkReceiver {
    private Network m_network = new Network();
    private Timer m_watchdogTimer = new Timer();
    private boolean m_connected = false;
    private double m_xPos = 0;
    private double m_yPos = 0;
    private boolean m_newPos = true;
    private Object m_lock = new Object();
    private static int k_maxButtons = 100;
    private boolean[] m_buttonStates = new boolean[k_maxButtons]; 
    private ApriltagsCamera m_camera;

    double m_testX = 10*12;
    double m_testY = 0;
    double m_angle = 90;

    public PositionServer(ApriltagsCamera camera) {
        m_camera = camera;
    }

    public void start() {
        m_network.listen(this, 5802);

        m_watchdogTimer.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
                // Logger.log("PositionServer", 1, "connected=" + m_connected);
				if (m_connected) {
					Logger.log("PositionServer",-1, "Send position");

                    double xPos;
                    double yPos;
                    double angle;
                    boolean newPos;

                    synchronized (m_lock)
                    {
                        xPos = m_xPos;
                        yPos = m_yPos;
                        angle = m_angle;
                        newPos = m_newPos;

                        m_newPos = false;
                    }

                    if (newPos)
                    {
					    m_network.sendMessage(String.format("+%.2f %.2f %.2f\n", angle, xPos, yPos));
                    }
                    else
                    {
                        m_network.sendMessage("-\n");       // keep alive
                    }
				}
			}
		}, 200, 200);   // Send current position 5 times a second
    }

    public void setPosition(double x, double y, double angle) {
        synchronized (m_lock) {
            m_xPos = x;
            m_yPos = y;
            m_angle = angle;
            m_newPos = true;
        }
    }

    boolean m_redAlliance;

    public class Target
    {
        public int m_no;
        public int m_level;
        public double m_x;
        public double m_y;
        public double m_h;

        Target(int no, int level, double x, double y, double h)
        {
            m_no = no;
            m_level = level;
            m_x = x;
            m_y = y;
            m_h = h;
        }
    }

    public boolean getButtonState(int buttonNo)
    {
        boolean state = false;

        if ((buttonNo >= 0) && (buttonNo < k_maxButtons))
        {
            state = m_buttonStates[buttonNo];
            m_buttonStates[buttonNo] = false;
        }

        if (state)
        {
            Logger.log("PositionServer", 1, String.format("Button %d true", buttonNo));
        }

        return(state);
    }

    void processButton(String cmd)
    {
        // Logger.log("PositionServer", 1, cmd);
        
        int[] arg = ApriltagsCamera.parseIntegers(cmd, 1);

        if (arg != null)
        {
            int buttonNo = arg[0];

            Logger.log("PositionServer", 1, String.format("Button %d pressed", buttonNo));

            if ((buttonNo >= 0) && (buttonNo < k_maxButtons))
            {
                m_buttonStates[buttonNo] = true;
            }
        }
        else {
            Logger.log("PositionServer", 1, "processButton: bad command");
        }
    }

    void processCamera(String cmd)
    {
        // Logger.log("PositionServer", 1, cmd);
        
        int[] arg = ApriltagsCamera.parseIntegers(cmd, 1);

        if (arg != null)
        {
            m_camera.disableCameras(arg[0] == 1);
        }
        else {
            Logger.log("PositionServer", 1, "processCamera: bad command");
        }
    }

    @Override
    public void processData(String data) {
        Logger.log("PositionServer", -1, String.format("Data: %s", data));

        switch (data.charAt(0)) {
            case 'B': // button
                processButton(data.substring(1).trim());
                break;

            case 'C':   // Camera command
                processCamera(data.substring(1).trim());
                break;

            case 'k':   // keep alive
                break;

            default:
                Logger.log("PositionServer", 3, String.format("Invalid command: %s", data));
                break;
        }
    }

    private void sendAllianceColor()
    {
        Logger.log("PositionServer", 1, String.format("sendAllianceColor: red =%b", m_redAlliance));
        m_network.sendMessage(String.format("c%c", m_redAlliance? 'r' : 'b'));
    }

    @Override
    public void connected() {
        Logger.log("PositionServer", 1, String.format("connected: red=%b", m_redAlliance));

        m_connected = true;

        sendAllianceColor();
    }

    @Override
    public void disconnected() {
        Logger.log("PositionServer", 1, "disconnected");

        m_connected = false;
    }

}
