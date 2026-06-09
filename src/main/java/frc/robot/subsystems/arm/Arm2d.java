package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Arm2d {
    
    private LoggedMechanism2d mech;
    private LoggedMechanismLigament2d elbow;
    private LoggedMechanismLigament2d shoulder;
    private String name;

    public Arm2d(String name, Color8Bit color) {
        this.name = name;
        this.mech = new LoggedMechanism2d(3, 3);
    }

    public void setElbow(double angle) {
        elbow.setAngle(angle);
    }

    public void setShoulder(double angle) {
        shoulder.setAngle(angle);
    }

    public void periodic() {
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }
}
