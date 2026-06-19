package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs{
        public double volts;
        public double amps;
        public double pos;
        public double vel;
    }
  
    public default void updateInputs(ClimbIOInputs inputs){};

    public default void setVoltage(double voltage){};
  
    public default void goToPos(double pos){};

    public default void hold(double pos){};

    public default void stop(){};

}
