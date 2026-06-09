package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public double elbowVolts;
        public double elbowAmps;
        public double elbowPos;
        public double elbowVel;

        public double shoulderVolts;
        public double shoulderAmps;
        public double shoulderPos;
        public double shoulderVel;
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
