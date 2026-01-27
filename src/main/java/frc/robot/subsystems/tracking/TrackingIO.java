package frc.robot.subsystems.tracking;

import org.littletonrobotics.junction.AutoLog;

public interface TrackingIO {
    @AutoLog
    public static class TrackingIOInputs {
        public double tx = 0.0;
        public double tv = 0.0;
        public double ta = 0.0;
        public double tz = 0.0;
        public double ty = 0.0;
        public double tx_3d = 0.0;
        public double hb = 0.0;
        public boolean connected = false;
    }

    public default void updateInputs(TrackingIOInputs inputs) {}

    public default void setpl(int pl) {}

    public default void setValidIds(double[] valid) {}
}