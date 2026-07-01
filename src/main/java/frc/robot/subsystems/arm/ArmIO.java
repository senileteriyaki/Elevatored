package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public double elbowVoltage_v;
        public double elbowCurrent_a;
        public double elbowPos_deg;
        public double elbowVel_dps;

        public double shoulderVoltage_v;
        public double shoulderCurrent_a;
        public double shoulderPos_deg;
        public double shoulderVel_dps;
    }
    
    public default void updateInputs(ArmIOInputs inputs){}

    public default void setElbowVoltage(double voltage){}

    public default void setShoulderVoltage(double voltage){}
    
    public default void goToElbowPos(double pos){}

    public default void goToShoulderPos(double pos){}

    public default void holdElbow(double pos){}

    public default void holdShoulder(double pos){}

    public default void stopElbow(){}

    public default void stopShoulder(){}
    
}
