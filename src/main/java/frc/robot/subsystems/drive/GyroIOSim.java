package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

public class GyroIOSim implements GyroIO {
    private Pigeon2 pigeon;
    private Pigeon2SimState state;

    public GyroIOSim() {
        this.pigeon = new Pigeon2(10, ""); // TODO: Implement later
        this.state = new Pigeon2SimState(pigeon);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        
    }
}
