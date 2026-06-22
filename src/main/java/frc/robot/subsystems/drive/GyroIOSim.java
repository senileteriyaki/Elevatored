package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private Pigeon2 pigeon;
    private Pigeon2SimState state;

    public GyroIOSim() {
        this.pigeon = new Pigeon2(10, ""); // TODO: Implement later
        this.state = new Pigeon2SimState(pigeon);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw_Rot2d = Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
        inputs.pitch_Rot2d = Rotation2d.fromDegrees(pigeon.getPitch().getValueAsDouble());
        inputs.roll_Rot2d = Rotation2d.fromDegrees(pigeon.getRoll().getValueAsDouble());
        inputs.yawVel_radps = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        inputs.odometryYawTimestamps = PhoenixOdometryThread.getInstance()
            .makeTimestampQueue().stream().mapToDouble(val -> val).toArray();
        inputs.odometryYawPositions = PhoenixOdometryThread.getInstance()
            .registerSignal(pigeon.getYaw()).stream()
            .map(val -> Rotation2d.fromDegrees(val)).toArray(Rotation2d[]::new);
    }

    @Override
    public void zero() {
        state.setRawYaw(180);
    }
}
