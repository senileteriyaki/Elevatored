package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ArmIOReal implements ArmIO{

    private TalonFX elbowMotor = new TalonFX(ArmConstants.ELBOW_MOTOR_ID);
    private TalonFX shoulderMotor = new TalonFX(ArmConstants.SHOULDER_MOTOR_ID);

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
        this.elbowFF = new ArmFeedforward(0, 0, 0, 0); //useless but you might as well be able to set voltage
    
        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
        this.shoulderMM = new MotionMagicVoltage(ArmConstants.minAngle);
        this.shoulderFF = new ArmFeedforward(0, 0, 0, 0);

        shoulderConfig.CurrentLimits.StatorCurrentLimit = 80;
        shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        elbowConfig.CurrentLimits.StatorCurrentLimit = 80;
        elbowConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        shoulderConfig.MotionMagic.MotionMagicAcceleration = 160;
        shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        shoulderConfig.MotionMagic.MotionMagicJerk = 1600;

        elbowConfig.MotionMagic.MotionMagicAcceleration = 160;
        elbowConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        elbowConfig.MotionMagic.MotionMagicJerk = 1600;

        shoulderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        shoulderConfig.Slot0.kG = 0.1;
        shoulderConfig.Slot0.kP = 0.1;
        shoulderConfig.Slot0.kD = 0.05;

        elbowConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        elbowConfig.Slot0.kG = 0.1;
        elbowConfig.Slot0.kP = 0.1;
        elbowConfig.Slot0.kD = 0.05;

        elbowPos = elbowMotor.getPosition();
        elbowVel = elbowMotor.getVelocity();
        elbowVolts = elbowMotor.getMotorVoltage();
        elbowAmps = elbowMotor.getStatorCurrent();

        shoulderPos = shoulderMotor.getPosition();
        shoulderVel = shoulderMotor.getVelocity();
        shoulderVolts = shoulderMotor.getMotorVoltage();
        shoulderAmps = shoulderMotor.getStatorCurrent();

        // TODO: Apply motor configs here
        elbowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shoulderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.tryUntilOk(5, () -> elbowMotor.getConfigurator().apply(elbowConfig));
        PhoenixUtil.tryUntilOk(5, () -> shoulderMotor.getConfigurator().apply(shoulderConfig));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        BaseStatusSignal.refreshAll(elbowPos, elbowVel, elbowVolts, elbowAmps,
            shoulderPos, shoulderVel, shoulderVolts, shoulderAmps);
        
        inputs.elbowPos_deg = elbowPos.getValueAsDouble() * 360;
        inputs.elbowVel_dps = elbowVel.getValueAsDouble() * 360;
        inputs.elbowVoltage_v = elbowVolts.getValueAsDouble();
        inputs.elbowCurrent_a = elbowAmps.getValueAsDouble();

        inputs.shoulderPos_deg = shoulderPos.getValueAsDouble() * 360;
        inputs.shoulderVel_dps = shoulderVel.getValueAsDouble() * 360;
        inputs.shoulderVoltage_v = shoulderVolts.getValueAsDouble();
        inputs.shoulderCurrent_a = shoulderAmps.getValueAsDouble();
    }

    @Override
    public void setElbowVoltage(double voltage){
        elbowMotor.setVoltage(voltage + elbowFF.calculate(elbowPos.getValueAsDouble(), 0)); // Raymond: ueah this is good but there isn't that much of a point cuz when u just set voltage its kinda useless lowk. 
    }

    @Override
    public void setShoulderVoltage(double voltage) {
        shoulderMotor.setVoltage(voltage + shoulderFF.calculate(shoulderPos.getValueAsDouble(), 0)); 
    }

    @Override
    public void goToElbowPos(double pos){
        elbowMotor.setControl(elbowMM
            .withPosition(pos)
            .withFeedForward(elbowFF.calculate(elbowPos.getValueAsDouble(), 0))); // Raymond: sure you can use feed forwwards here but remember that motion magic already uses feedforward so like its not that useful. But this is good for gravitational compensation. Also make sure the units are right to what it corresponds to.
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
