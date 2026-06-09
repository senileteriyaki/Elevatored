package frc.robot.subsystems.arm;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmIOInputsAutoLogged extends ArmIO.ArmIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ElbowVolts", elbowVolts);
    table.put("ElbowAmps", elbowAmps);
    table.put("ElbowPos", elbowPos);
    table.put("ElbowVel", elbowVel);
    table.put("ShoulderVolts", shoulderVolts);
    table.put("ShoulderAmps", shoulderAmps);
    table.put("ShoulderPos", shoulderPos);
    table.put("ShoulderVel", shoulderVel);
  }

  @Override
  public void fromLog(LogTable table) {
    elbowVolts = table.get("ElbowVolts", elbowVolts);
    elbowAmps = table.get("ElbowAmps", elbowAmps);
    elbowPos = table.get("ElbowPos", elbowPos);
    elbowVel = table.get("ElbowVel", elbowVel);
    shoulderVolts = table.get("ShoulderVolts", shoulderVolts);
    shoulderAmps = table.get("ShoulderAmps", shoulderAmps);
    shoulderPos = table.get("ShoulderPos", shoulderPos);
    shoulderVel = table.get("ShoulderVel", shoulderVel);
  }

  public ArmIOInputsAutoLogged clone() {
    ArmIOInputsAutoLogged copy = new ArmIOInputsAutoLogged();
    copy.elbowVolts = this.elbowVolts;
    copy.elbowAmps = this.elbowAmps;
    copy.elbowPos = this.elbowPos;
    copy.elbowVel = this.elbowVel;
    copy.shoulderVolts = this.shoulderVolts;
    copy.shoulderAmps = this.shoulderAmps;
    copy.shoulderPos = this.shoulderPos;
    copy.shoulderVel = this.shoulderVel;
    return copy;
  }
}
