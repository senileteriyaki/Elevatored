package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Arm2d {
    
    private LoggedMechanism2d mech;
    private LoggedMechanismLigament2d arm;
    private String name;

    public Arm2d(String name, Color8Bit color){ //top view of arm
        this.name = name;
        this.mech = new LoggedMechanism2d(3, 3);
        this.arm = mech.getRoot("root", 2, 1)
            .append(new LoggedMechanismLigament2d("armSimTop", 0.2, 0, 5, color));
    }

    public void set(double deg){
        arm.setAngle(deg);
    }

    public void periodic() {
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }
}