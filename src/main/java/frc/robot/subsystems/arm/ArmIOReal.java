package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ArmConstants;

public class ArmIOReal implements ArmIO{

    private TalonFX elbowMotor = new TalonFX(2);
    private TalonFX shoulderMotor = new TalonFX(3);

    private final StatusSignal<Angle> elbowPos;
    private final StatusSignal<AngularVelocity> elbowVel;
    private final StatusSignal<Voltage> elbowVolts;
    private final StatusSignal<Current> elbowAmps;
    private final MotionMagicVoltage elbowMM;
    private final ArmFeedforward elbowFF;

    private final StatusSignal<Angle> shoulderPos;
    private final StatusSignal<AngularVelocity> shoulderVel;
    private final StatusSignal<Voltage> shoulderVolts;
    private final StatusSignal<Current> shoulderAmps;
    private final MotionMagicVoltage shoulderMM;
    private final ArmFeedforward shoulderFF;
    
    public ArmIOReal(){
        TalonFXConfiguration elbowConfig = new TalonFXConfiguration();
        this.elbowMM = new MotionMagicVoltage(ArmConstants.minAngle);
        this.elbowFF = new ArmFeedforward(0, 0, 0, 0);

        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
        this.shoulderMM = new MotionMagicVoltage(ArmConstants.minAngle);
        this.shoulderFF = new ArmFeedforward(0, 0, 0, 0);

        elbowPos = elbowMotor.getPosition();
        elbowVel = elbowMotor.getVelocity();
        elbowVolts = elbowMotor.getMotorVoltage();
        elbowAmps = elbowMotor.getStatorCurrent();

        shoulderPos = shoulderMotor.getPosition();
        shoulderVel = shoulderMotor.getVelocity();
        shoulderVolts = shoulderMotor.getMotorVoltage();
        shoulderAmps = shoulderMotor.getStatorCurrent();

        // NOTE: Apply motor configs here
        elbowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shoulderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    frc.robot.util.PhoenixUtil.tryUntilOk(5, () -> elbowMotor.getConfigurator().apply(elbowConfig));
    frc.robot.util.PhoenixUtil.tryUntilOk(5, () -> shoulderMotor.getConfigurator().apply(shoulderConfig));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        BaseStatusSignal.refreshAll(elbowPos, elbowVel, elbowVolts, elbowAmps,
            shoulderPos, shoulderVel, shoulderVolts, shoulderAmps);
        inputs.elbowPos = elbowPos.getValueAsDouble() * 360;
        inputs.elbowVel = elbowVel.getValueAsDouble() * 360;
        inputs.elbowVolts = elbowVolts.getValueAsDouble();
        inputs.elbowAmps = elbowAmps.getValueAsDouble();

        inputs.shoulderPos = shoulderPos.getValueAsDouble() * 360;
        inputs.shoulderVel = shoulderVel.getValueAsDouble() * 360;
        inputs.shoulderVolts = shoulderVolts.getValueAsDouble();
        inputs.shoulderAmps = shoulderAmps.getValueAsDouble();
    }

    @Override
    public void setElbowVoltage(double voltage){
        elbowMotor.setVoltage(voltage + elbowFF.calculate(elbowPos.getValueAsDouble(), 0));
    }

    @Override
    public void setShoulderVoltage(double voltage) {
        shoulderMotor.setVoltage(voltage + shoulderFF.calculate(shoulderPos.getValueAsDouble(), 0));
    }

    @Override
    public void goToElbowPos(double pos){
        elbowMotor.setControl(elbowMM
            .withPosition(pos)
            .withFeedForward(elbowFF.calculate(elbowPos.getValueAsDouble(), 0)));
    }

    @Override
    public void goToShoulderPos(double pos) {
        shoulderMotor.setControl(shoulderMM
            .withPosition(pos)
            .withFeedForward(shoulderFF.calculate(shoulderPos.getValueAsDouble(), 0)));
    }

    @Override
    public void holdElbow(double pos){
        goToElbowPos(pos);
    }

    @Override
    public void holdShoulder(double pos) {
        goToShoulderPos(pos);
    }

    @Override
    public void stopElbow(){
        elbowMotor.stopMotor();
    }

    @Override
    public void stopShoulder() {
        shoulderMotor.stopMotor();
    }
}
