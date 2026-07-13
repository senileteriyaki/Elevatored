package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
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
    private final MotionMagicVoltage elbowRequest;

    private final StatusSignal<Angle> shoulderPos;
    private final StatusSignal<AngularVelocity> shoulderVel;
    private final StatusSignal<Voltage> shoulderVolts;
    private final StatusSignal<Current> shoulderAmps;
    private final MotionMagicVoltage shoulderRequest;
    
    public ArmIOReal(){
        TalonFXConfiguration elbowConfig = new TalonFXConfiguration();
        MotionMagicConfigs elbowMMConfig = new MotionMagicConfigs();
        this.elbowRequest = new MotionMagicVoltage(ArmConstants.eMin);
    
        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
        MotionMagicConfigs shoulderMMConfig = new MotionMagicConfigs();
        this.shoulderRequest = new MotionMagicVoltage(ArmConstants.sMin);

        // Elbow configs
        elbowConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.ELBOW_CURRENT_LIMIT;
        elbowConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        elbowMMConfig.MotionMagicCruiseVelocity = ArmConstants.ELBOW_MAX_VELOCITY_DPS;
        elbowMMConfig.MotionMagicAcceleration = ArmConstants.ELBOW_MAX_ACCELERATION_DPS2;
        elbowMMConfig.MotionMagicJerk = ArmConstants.ELBOW_MAX_JERK;
        elbowConfig.MotionMagic = elbowMMConfig;

        elbowConfig.Feedback.SensorToMechanismRatio = ArmConstants.ELBOW_GEAR_RATIO;
        elbowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elbowConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        elbowConfig.Slot0.kG = ArmConstants.elbowKG;
        elbowConfig.Slot0.kS = ArmConstants.elbowKS;
        elbowConfig.Slot0.kV = ArmConstants.elbowKV;
        elbowConfig.Slot0.kP = ArmConstants.elbowKP;
        elbowConfig.Slot0.kI = ArmConstants.elbowKI;
        elbowConfig.Slot0.kD = ArmConstants.elbowKD;

        // Shoulder configs
        shoulderConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.SHOULDER_CURRENT_LIMIT;
        shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        shoulderMMConfig.MotionMagicCruiseVelocity = ArmConstants.SHOULDER_MAX_VELOCITY_DPS;
        shoulderMMConfig.MotionMagicAcceleration = ArmConstants.SHOULDER_MAX_ACCELERATION_DPS2;
        shoulderMMConfig.MotionMagicJerk = ArmConstants.SHOULDER_MAX_JERK;
        shoulderConfig.MotionMagic = shoulderMMConfig;

        shoulderConfig.Feedback.SensorToMechanismRatio = ArmConstants.SHOULDER_GEAR_RATIO;
        shoulderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shoulderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        shoulderConfig.Slot0.kG = ArmConstants.shoulderKG;
        shoulderConfig.Slot0.kS = ArmConstants.shoulderKS;
        shoulderConfig.Slot0.kV = ArmConstants.shoulderKV;
        shoulderConfig.Slot0.kP = ArmConstants.shoulderKP;
        shoulderConfig.Slot0.kI = ArmConstants.shoulderKI;
        shoulderConfig.Slot0.kD = ArmConstants.shoulderKD;

        elbowPos = elbowMotor.getPosition();
        elbowVel = elbowMotor.getVelocity();
        elbowVolts = elbowMotor.getMotorVoltage();
        elbowAmps = elbowMotor.getStatorCurrent();

        shoulderPos = shoulderMotor.getPosition();
        shoulderVel = shoulderMotor.getVelocity();
        shoulderVolts = shoulderMotor.getMotorVoltage();
        shoulderAmps = shoulderMotor.getStatorCurrent();

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
        elbowMotor.setVoltage(MathUtil.clamp(voltage, -12, 12)); // ethan - recommend voltageOut
    }

    @Override
    public void setShoulderVoltage(double voltage) {
        shoulderMotor.setVoltage(MathUtil.clamp(voltage, -12, 12)); // ethan - recommend voltageOut
    }

    @Override
    public void goToElbowPos(double pos){
        elbowMotor.setControl(elbowRequest
            .withPosition(pos));
    }
    @Override
    public void goToShoulderPos(double pos) {
        shoulderMotor.setControl(shoulderRequest
            .withPosition(pos));
    }

    @Override
    public void holdElbow(double pos){ // ethan - why do we have this? just run goToElbowPos()
        goToElbowPos(pos);
    }

    @Override
    public void holdShoulder(double pos) { // ethan - why do we have this? just run goToElbowPos()
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
