package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {
    private Rotation2d yaw;
    private double yawVel_radps;

    public GyroIOSim() {
        this.yaw = new Rotation2d();
        this.yawVel_radps = 0;	
    }

    @Override
    public void setYaw(Rotation2d yaw, double yawVel_radps) {
        this.yaw = yaw;
        this.yawVel_radps = yawVel_radps;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yaw_Rot2d = yaw;
        inputs.yawVel_radps = yawVel_radps;
        inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryYawPositions = new Rotation2d[] {yaw};
    }

    @Override
    public void zero() {
        yaw = new Rotation2d();
        yawVel_radps = 0.0;
    }
}
