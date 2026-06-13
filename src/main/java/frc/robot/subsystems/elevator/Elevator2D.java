package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Elevator2D {
    private LoggedMechanismLigament2d elevator;
    private LoggedMechanism2d mech;
    private LoggedMechanismRoot2d root;
    private String name;

    public Elevator2D(String name, Color8Bit color){
        this.name = name;
        mech = new LoggedMechanism2d(4, 4);
        root =  mech.getRoot("root", 2, 0.1);
        elevator = root.append(new LoggedMechanismLigament2d("elevSim", 0.5, 90, 5, color)); 
    }

    public void set(double height){
        elevator.setLength(height);
    }

    public void periodic() {
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }
}