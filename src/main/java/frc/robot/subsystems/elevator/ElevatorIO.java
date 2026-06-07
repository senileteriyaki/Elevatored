package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double volts;
        public double amps;
        public double pos;
        public double vel;
    }
    
    public default void updateInputs(ElevatorIOInputs inputs){};

    public default void setVoltage(double voltage){};
    
    public default void goToPos(double pos){};

    public default void hold(double pos){};

    public default void stop(){};

}
