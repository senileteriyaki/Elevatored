package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Climb2D {
    private LoggedMechanismLigament2d climb;
    private LoggedMechanismLigament2d arm;
    private LoggedMechanism2d mech;
    private String name;

    public Climb2D(String name, Color8Bit color1, Color8Bit color2) {
        this.name = name;
        mech = new LoggedMechanism2d(4, 4);

    }

    public void set(double height) {
        climb.setLength(height);
    }

    public void periodic() {
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }

    public void setArm(double deg) {
        arm.setAngle(deg);
    }
}
