package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO{

    private SingleJointedArmSim elbowSim;
    private SingleJointedArmSim shoulderSim;

    private Constraints elbowConstraints;
    private Constraints shoulderConstraints;

    private ProfiledPIDController elbowController;
    private ProfiledPIDController shoulderController;

    private State elbowGoal;
    private State shoulderGoal;

    private Timer elbowTimer;
    private Timer shoulderTimer;

    private double elbowVoltsApplied;
    private double shoulderVoltsApplied;

    private double elbowTarget;
    private double shoulderTarget;
    public ArmIOSim(){
        this.elbowSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 127.5, 
                                   1.5, 0.2, ArmConstants.minAngle, 
                                   ArmConstants.maxAngle, true, 0, 0.1, 0);
        this.shoulderSim = new SingleJointedArmSim(DCMotor.getKrakenX60(2), 151, 
                                   1.5, 0.2, ArmConstants.minAngle, 
                                   ArmConstants.maxAngle, true, 0, 0.1, 0);

        this.elbowConstraints = new Constraints(ArmConstants.ELBOW_MAX_VELOCITY_DPS, ArmConstants.ELBOW_MAX_ACCELERATION_DPS2); // Unit in degrees
        this.shoulderConstraints = new Constraints(ArmConstants.SHOULDER_MAX_VELOCITY_DPS, ArmConstants.SHOULDER_MAX_ACCELERATION_DPS2);

        this.elbowController = new ProfiledPIDController(ArmConstants.elbowKP, ArmConstants.elbowKI, ArmConstants.elbowKD, elbowConstraints);
        this.shoulderController = new ProfiledPIDController(ArmConstants.shoulderKP, ArmConstants.shoulderKI, ArmConstants.shoulderKD, shoulderConstraints);

        elbowController.setTolerance(ArmConstants.elbowTolerance);
        shoulderController.setTolerance(ArmConstants.shoulderTolerance);

        this.elbowTimer = new Timer();
        elbowTimer.start();

        this.shoulderTimer = new Timer();
        shoulderTimer.start();

        this.elbowVoltsApplied = this.shoulderVoltsApplied = 0;
        this.elbowTarget = this.shoulderTarget = ArmConstants.minAngle;
    }
    
    @Override
    public void updateInputs(ArmIOInputs inputs){
        elbowSim.update(Constants.globalDelta_s);
        inputs.elbowPos_deg = Units.radiansToDegrees(elbowSim.getAngleRads());
        inputs.elbowVel_dps = Units.radiansToDegrees(elbowSim.getVelocityRadPerSec());
        inputs.elbowVoltage_v = elbowVoltsApplied;
        inputs.elbowCurrent_a = elbowSim.getCurrentDrawAmps();

        shoulderSim.update(Constants.globalDelta_s);
        inputs.shoulderPos_deg = Units.degreesToRadians(shoulderSim.getAngleRads());
        inputs.shoulderVel_dps = Units.degreesToRadians(shoulderSim.getVelocityRadPerSec());
        inputs.shoulderVoltage_v = shoulderVoltsApplied;
        inputs.shoulderCurrent_a = shoulderSim.getCurrentDrawAmps();
    }

    @Override
    public void setElbowVoltage(double voltage){
        elbowVoltsApplied = MathUtil.clamp(voltage, -12, 12);
        elbowSim.setInputVoltage(elbowVoltsApplied);
    }

    @Override
    public void setShoulderVoltage(double voltage) {
        shoulderVoltsApplied = MathUtil.clamp(voltage, -12, 12);
        shoulderSim.setInputVoltage(shoulderVoltsApplied);
    }

    @Override
    public void goToElbowPos(double pos) {
        if (pos != elbowTarget) {
            elbowTarget = pos;
            elbowGoal = new State(Units.degreesToRadians(elbowTarget), 0);
            elbowTimer.reset();
        }

        double controllerVoltage = elbowController.calculate(elbowSim.getAngleRads(), elbowGoal);

        setElbowVoltage(controllerVoltage);
    }

    @Override
    public void goToShoulderPos(double pos) {
        if (pos != shoulderTarget) {
            shoulderTarget = pos;
            shoulderGoal = new State(Units.degreesToRadians(shoulderTarget), 0);
            shoulderTimer.reset();
        }

        double controllerVoltage = shoulderController.calculate(shoulderSim.getAngleRads(), shoulderGoal);

        setElbowVoltage(controllerVoltage);
    }

    @Override
    public void holdElbow(double pos){
        goToElbowPos(pos);
    }

    @Override
    public void holdShoulder(double pos) {
        goToElbowPos(pos);
    }

    @Override
    public void stopElbow(){
        elbowVoltsApplied = 0;
        setElbowVoltage(elbowVoltsApplied);
    }

    @Override
    public void stopShoulder() {
        shoulderVoltsApplied = 0;
        setShoulderVoltage(shoulderVoltsApplied);
    }
}
