package frc.robot.subsystems.arm;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmIOInputsAutoLogged extends ArmIO.ArmIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ElbowVoltage_v", elbowVoltage_v);
    table.put("ElbowCurrent_a", elbowCurrent_a);
    table.put("ElbowPos_deg", elbowPos_deg);
    table.put("ElbowVel_dps", elbowVel_dps);
    table.put("ShoulderVoltage_v", shoulderVoltage_v);
    table.put("ShoulderCurrent_a", shoulderCurrent_a);
    table.put("ShoulderPos_deg", shoulderPos_deg);
    table.put("ShoulderVel_dps", shoulderVel_dps);
  }

  @Override
  public void fromLog(LogTable table) {
    elbowVoltage_v = table.get("ElbowVoltage_v", elbowVoltage_v);
    elbowCurrent_a = table.get("ElbowCurrent_a", elbowCurrent_a);
    elbowPos_deg = table.get("ElbowPos_deg", elbowPos_deg);
    elbowVel_dps = table.get("ElbowVel_dps", elbowVel_dps);
    shoulderVoltage_v = table.get("ShoulderVoltage_v", shoulderVoltage_v);
    shoulderCurrent_a = table.get("ShoulderCurrent_a", shoulderCurrent_a);
    shoulderPos_deg = table.get("ShoulderPos_deg", shoulderPos_deg);
    shoulderVel_dps = table.get("ShoulderVel_dps", shoulderVel_dps);
  }

  public ArmIOInputsAutoLogged clone() {
    ArmIOInputsAutoLogged copy = new ArmIOInputsAutoLogged();
    copy.elbowVoltage_v = this.elbowVoltage_v;
    copy.elbowCurrent_a = this.elbowCurrent_a;
    copy.elbowPos_deg = this.elbowPos_deg;
    copy.elbowVel_dps = this.elbowVel_dps;
    copy.shoulderVoltage_v = this.shoulderVoltage_v;
    copy.shoulderCurrent_a = this.shoulderCurrent_a;
    copy.shoulderPos_deg = this.shoulderPos_deg;
    copy.shoulderVel_dps = this.shoulderVel_dps;
    return copy;
  }
}
