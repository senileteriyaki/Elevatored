package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim implements ArmIO{

    private SingleJointedArmSim elbowSim;
    private SingleJointedArmSim shoulderSim;

    private PIDController elbowPID;
    private PIDController shoulderPID;

    private double elbowVoltsApplied;
    private double shoulderVoltsApplied;

    private double elbowTarget;
    private double shoulderTarget;

    public ArmIOSim(){
        this.elbowSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 127.5, 
                                   1.5, 0.2, ArmConstants.minAngle, 
                                   ArmConstants.maxAngle, true, 0, 0.1, 0);
        this.elbowPID = new PIDController(ArmConstants.elbowKP, ArmConstants.elbowKI, ArmConstants.elbowKD);
        elbowPID.setTolerance(0.05);
        this.shoulderSim = new SingleJointedArmSim(DCMotor.getKrakenX60(2), 151, 
                                   1.5, 0.2, ArmConstants.minAngle, 
                                   ArmConstants.maxAngle, true, 0, 0.1, 0);
        this.shoulderPID = new PIDController(ArmConstants.shoulderKP, ArmConstants.shoulderKI, ArmConstants.shoulderKD);
        shoulderPID.setTolerance(0.05);
        this.elbowVoltsApplied = this.shoulderVoltsApplied = 0;
        this.elbowTarget = this.shoulderTarget = ArmConstants.minAngle;
    }
    
    @Override
    public void updateInputs(ArmIOInputs inputs){
        elbowSim.update(Constants.globalDelta_s);
        inputs.elbowPos = Units.radiansToDegrees(elbowSim.getAngleRads());
        inputs.elbowVel = Units.radiansToDegrees(elbowSim.getVelocityRadPerSec());
        inputs.elbowVolts = elbowVoltsApplied;
        inputs.elbowAmps = elbowSim.getCurrentDrawAmps();

        shoulderSim.update(Constants.globalDelta_s);
        inputs.shoulderPos = Units.degreesToRadians(shoulderSim.getAngleRads());
        inputs.shoulderVel = Units.degreesToRadians(shoulderSim.getVelocityRadPerSec());
        inputs.shoulderVolts = shoulderVoltsApplied;
        inputs.shoulderAmps = shoulderSim.getCurrentDrawAmps();
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
    public void goToElbowPos(double pos){
        elbowTarget = pos;
        setElbowVoltage(elbowPID.calculate(elbowSim.getAngleRads(), Units.degreesToRadians(elbowTarget))); // Raymond: Sure but use a trapezoidal profile here as well. Try to integrate that. 
    }

    @Override
    public void goToShoulderPos(double pos) {
        shoulderTarget = pos;
        setShoulderVoltage(shoulderPID.calculate(shoulderSim.getAngleRads(), Units.degreesToRadians(shoulderTarget)));
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
        elbowSim.setInputVoltage(elbowVoltsApplied);
    }

    @Override
    public void stopShoulder() {
        shoulderVoltsApplied = 0;
        shoulderSim.setInputVoltage(shoulderVoltsApplied);
    }
}
