package frc.robot.subsystems.climb;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimbIOInputsAutoLogged extends ClimbIO.ClimbIOInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("Volts", volts);
        table.put("Amps", amps);
        table.put("Pos", pos);
        table.put("Vel", vel);
    }

    @Override
    public void fromLog(LogTable table) {
        volts = table.get("Volts", volts);
        amps = table.get("Amps", amps);
        pos = table.get("Pos", pos);
        vel = table.get("Vel", vel);
    }

    public ClimbIOInputsAutoLogged clone() {
        ClimbIOInputsAutoLogged copy = new ClimbIOInputsAutoLogged();
        copy.volts = this.volts;
        copy.amps = this.amps;
        copy.pos = this.pos;
        copy.vel = this.vel;
        return copy;
    }
}
