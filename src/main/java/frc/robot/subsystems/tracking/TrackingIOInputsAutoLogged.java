package frc.robot.subsystems.tracking;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TrackingIOInputsAutoLogged extends TrackingIO.TrackingIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("Tx", tx);
    table.put("Tv", tv);
    table.put("Ta", ta);
    table.put("Tz", tz);
    table.put("Ty", ty);
    table.put("Tx_3d", tx_3d);
    table.put("Hb", hb);
    table.put("Connected", connected);
  }

  @Override
  public void fromLog(LogTable table) {
    tx = table.get("Tx", tx);
    tv = table.get("Tv", tv);
    ta = table.get("Ta", ta);
    tz = table.get("Tz", tz);
    ty = table.get("Ty", ty);
    tx_3d = table.get("Tx_3d", tx_3d);
    hb = table.get("Hb", hb);
    connected = table.get("Connected", connected);
  }

  @Override
  public TrackingIOInputsAutoLogged clone() {
    TrackingIOInputsAutoLogged copy = new TrackingIOInputsAutoLogged();
    copy.tx = this.tx;
    copy.tv = this.tv;
    copy.ta = this.ta;
    copy.tz = this.tz;
    copy.ty = this.ty;
    copy.tx_3d = this.tx_3d;
    copy.hb = this.hb;
    copy.connected = this.connected;
    return copy;
  }
}
