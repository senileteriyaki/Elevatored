package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{

    }
    public default void updateInputs(ArmIOInputs inputs){}

    public default void setVoltage(double voltage){}
    
    public default void goToPos(double pos){}

    public default void hold(double pos){}

    public default void stop(){}
    
}
