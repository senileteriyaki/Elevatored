package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {

    private SingleJointedArmSim elbowSim;
    private SingleJointedArmSim shoulderSim;

    private Constraints elbowConstraints;
    private Constraints shoulderConstraints;

    private ProfiledPIDController elbowController;
    private ProfiledPIDController shoulderController;

    private double elbowVoltsApplied;
    private double shoulderVoltsApplied;

    private ArmFeedforward elbowFF;
    private ArmFeedforward shoulderFF;

    public ArmIOSim() {
        this.elbowSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), ArmConstants.ELBOW_GEAR_RATIO, ArmConstants.ELBOW_MOI, ArmConstants.ELBOW_LENGTH, 
            Units.degreesToRadians(ArmConstants.eMin), Units.degreesToRadians(ArmConstants.eMax), 
            false, Units.degreesToRadians(ArmConstants.ELBOW_STOW)
        );
        this.shoulderSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), ArmConstants.SHOULDER_GEAR_RATIO, ArmConstants.SHOULDER_MOI, ArmConstants.SHOULDER_LENGTH, 
            Units.degreesToRadians(ArmConstants.sMin), Units.degreesToRadians(ArmConstants.sMax), 
            true, Units.degreesToRadians(ArmConstants.SHOULDER_STOW)
        );

        /* ethan - fancy pid stuff. lowk just run trapezoidprofile and call it a day. we're just
         * simming to see if the code works, no need to do the actual simulation of arm pid/ffw.
         */

        this.elbowConstraints = new Constraints(ArmConstants.ELBOW_MAX_VELOCITY_DPS, ArmConstants.ELBOW_MAX_ACCELERATION_DPS2);
        this.shoulderConstraints = new Constraints(ArmConstants.SHOULDER_MAX_VELOCITY_DPS, ArmConstants.SHOULDER_MAX_ACCELERATION_DPS2);

        this.elbowController = new ProfiledPIDController(ArmConstants.elbowKP, ArmConstants.elbowKI, ArmConstants.elbowKD, elbowConstraints);
        this.shoulderController = new ProfiledPIDController(ArmConstants.shoulderKP, ArmConstants.shoulderKI, ArmConstants.shoulderKD, shoulderConstraints);

        elbowController.setTolerance(ArmConstants.elbowTolerance);
        shoulderController.setTolerance(ArmConstants.shoulderTolerance);

        // ethan - it's sim so you don't rlly need all of this.
        this.elbowFF = new ArmFeedforward(ArmConstants.elbowKS, ArmConstants.elbowKG, ArmConstants.elbowKV);
        this.shoulderFF = new ArmFeedforward(ArmConstants.shoulderKS, ArmConstants.shoulderKG, ArmConstants.shoulderKV);

        this.elbowVoltsApplied = 0;
        this.shoulderVoltsApplied = 0;

        elbowController.reset(ArmConstants.ELBOW_STOW);
        shoulderController.reset(ArmConstants.SHOULDER_STOW);
    }
    
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        elbowSim.update(Constants.globalDelta_s);
        inputs.elbowPos_deg = Units.radiansToDegrees(elbowSim.getAngleRads());
        inputs.elbowVel_dps = Units.radiansToDegrees(elbowSim.getVelocityRadPerSec());
        inputs.elbowVoltage_v = elbowVoltsApplied;
        inputs.elbowCurrent_a = elbowSim.getCurrentDrawAmps();

        shoulderSim.update(Constants.globalDelta_s);
        inputs.shoulderPos_deg = Units.radiansToDegrees(shoulderSim.getAngleRads());
        inputs.shoulderVel_dps = Units.radiansToDegrees(shoulderSim.getVelocityRadPerSec());
        inputs.shoulderVoltage_v = shoulderVoltsApplied;
        inputs.shoulderCurrent_a = shoulderSim.getCurrentDrawAmps();
    }

    @Override
    public void setElbowVoltage(double voltage) {
        elbowVoltsApplied = MathUtil.clamp(voltage, -12, 12);
        elbowSim.setInputVoltage(elbowVoltsApplied);
    }

    @Override
    public void setShoulderVoltage(double voltage) {
        shoulderVoltsApplied = MathUtil.clamp(voltage, -12, 12);
        shoulderSim.setInputVoltage(shoulderVoltsApplied);
    }

    /* ethan - recommend trapezoidprofile here. */
    @Override
    public void goToElbowPos(double pos) {
        double pidOutput = elbowController.calculate(Units.radiansToDegrees(elbowSim.getAngleRads()), pos);
        setElbowVoltage(pidOutput + elbowFF.calculate(
            Units.degreesToRadians(elbowController.getSetpoint().position), Units.degreesToRadians(elbowController.getSetpoint().velocity)));
    }

    /* ethan - recommend trapezoidprofile here. */
    @Override
    public void goToShoulderPos(double pos) {
        double pidOutput = shoulderController.calculate(Units.radiansToDegrees(shoulderSim.getAngleRads()), pos);
        setShoulderVoltage(pidOutput + shoulderFF.calculate(
            Units.degreesToRadians(shoulderController.getSetpoint().position), Units.degreesToRadians(shoulderController.getSetpoint().velocity)));
    }

    /* ethan - what are these two methods for. */
    @Override
    public void holdElbow(double pos) {
        goToElbowPos(pos);
    }

    @Override
    public void holdShoulder(double pos) {
        goToShoulderPos(pos); 
    }

    @Override
    public void stopElbow() {
        setElbowVoltage(0);
    }

    @Override
    public void stopShoulder() {
        setShoulderVoltage(0);
    }
}