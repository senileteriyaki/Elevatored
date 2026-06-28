package frc.robot.subsystems.climb;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimbIOInputsAutoLogged extends ClimbIO.ClimbIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Voltage_v", voltage_v);
    table.put("Current_a", current_a);
    table.put("Pos_deg", pos_deg);
    table.put("Vel_dps", vel_dps);
  }

  @Override
  public void fromLog(LogTable table) {
    voltage_v = table.get("Voltage_v", voltage_v);
    current_a = table.get("Current_a", current_a);
    pos_deg = table.get("Pos_deg", pos_deg);
    vel_dps = table.get("Vel_dps", vel_dps);
  }

  public ClimbIOInputsAutoLogged clone() {
    ClimbIOInputsAutoLogged copy = new ClimbIOInputsAutoLogged();
    copy.voltage_v = this.voltage_v;
    copy.current_a = this.current_a;
    copy.pos_deg = this.pos_deg;
    copy.vel_dps = this.vel_dps;
    return copy;
  }
}
