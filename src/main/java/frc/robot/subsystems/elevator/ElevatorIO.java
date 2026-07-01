package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double voltage_v;
        public double current_a;
        public double pos_m;
        public double vel_mps;
    }
    
    public default void updateInputs(ElevatorIOInputs inputs){};

    public default void setVoltage(double voltage){};
    
    public default void goToPos(double pos){};

    public default void hold(double pos){};

    public default void stop(){};

}
