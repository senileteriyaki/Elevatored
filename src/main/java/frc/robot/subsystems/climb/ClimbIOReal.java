package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ClimbIOReal implements ClimbIO{
    private TalonFX left = new TalonFX(7); // as lead
    private TalonFX right = new TalonFX(8);

    private final MotionMagicVoltage request;

    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Angle> pos;
    private final StatusSignal<Current> amps;
    private final StatusSignal<AngularVelocity> vel;

    public ClimbIOReal(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        MotionMagicConfigs mmConfig = new MotionMagicConfigs();

        request = new MotionMagicVoltage(ClimberConstants.stowAngle);

        config.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO / 360;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit = ClimberConstants.currentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        mmConfig.MotionMagicCruiseVelocity = ClimberConstants.MAX_VELOCITY;
        mmConfig.MotionMagicAcceleration = ClimberConstants.MAX_ACCELERATION;
        mmConfig.MotionMagicJerk = ClimberConstants.MAX_JERK;
        config.MotionMagic = mmConfig;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = ClimberConstants.kG;
        config.Slot0.kS = ClimberConstants.kS;
        config.Slot0.kV = ClimberConstants.kV;
        config.Slot0.kP = ClimberConstants.kP;
        config.Slot0.kI = ClimberConstants.kI;
        config.Slot0.kD = ClimberConstants.kD;

        PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(config));
        right.setControl(new Follower(left.getDeviceID(), MotorAlignmentValue.Aligned));

        volts = left.getMotorVoltage();
        pos = left.getPosition();
        amps = left.getStatorCurrent();
        vel = left.getVelocity();
    }

    public void updateInputs(ClimbIOInputs inputs){
        BaseStatusSignal.refreshAll(volts, amps, pos, vel);
        
        inputs.voltage_v = volts.getValueAsDouble();
        inputs.current_a = amps.getValueAsDouble();
        inputs.pos_deg = pos.getValueAsDouble();
        inputs.vel_dps = vel.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        left.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public void goToPos(double pos){
        left.setControl(request
            .withPosition(pos));
    }

    @Override
    public void hold(double pos){
        goToPos(pos);
    }

    @Override
    public void stop(){
        left.stopMotor();
        right.stopMotor();
    }

}
