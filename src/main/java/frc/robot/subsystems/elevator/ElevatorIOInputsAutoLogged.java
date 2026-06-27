package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Voltage_v", voltage_v);
    table.put("Current_a", current_a);
    table.put("Pos_m", pos_m);
    table.put("Vel_mps", vel_mps);
  }

  @Override
  public void fromLog(LogTable table) {
    voltage_v = table.get("Voltage_v", voltage_v);
    current_a = table.get("Current_a", current_a);
    pos_m = table.get("Pos_m", pos_m);
    vel_mps = table.get("Vel_mps", vel_mps);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.voltage_v = this.voltage_v;
    copy.current_a = this.current_a;
    copy.pos_m = this.pos_m;
    copy.vel_mps = this.vel_mps;
    return copy;
  }
}
