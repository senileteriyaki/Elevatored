package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Climb2D {
    private LoggedMechanismLigament2d climb;
    private LoggedMechanism2d mech;
    private String name;

    public Climb2D(String name, Color8Bit color1){
        this.name = name;
        mech = new LoggedMechanism2d(4, 4);
        climb = mech.getRoot("root", 2, 0.1)
                    .append(new LoggedMechanismLigament2d("climb", 0.5, 90, 5, color1));

    }

    public void set(double deg) {
        climb.setAngle(deg);
    }

    public void periodic() {
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }

}
