package frc.robot.subsystems.elevator;

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

public class ElevatorIOReal implements ElevatorIO{

    private TalonFX elevatorMotor = new TalonFX(1);
    private final MotionMagicVoltage request;
    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Angle> pos;
    private final StatusSignal<Current> amps;
    private final StatusSignal<AngularVelocity> vel;

    public ElevatorIOReal(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        MotionMagicConfigs mmConfig = new MotionMagicConfigs();

        request = new MotionMagicVoltage(ElevatorConstants.minHeight);

        config.Feedback.SensorToMechanismRatio = (2 * Math.PI * ElevatorConstants.drumRadius) / ElevatorConstants.GEAR_RATIO; 
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.currentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        mmConfig.MotionMagicCruiseVelocity = ElevatorConstants.maxVelocity;
        mmConfig.MotionMagicAcceleration = ElevatorConstants.maxAcceleration;
        mmConfig.MotionMagicJerk = ElevatorConstants.maxJerk;
        config.MotionMagic = mmConfig;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kG = ElevatorConstants.kG;
        config.Slot0.kS = ElevatorConstants.kS;
        config.Slot0.kV = ElevatorConstants.kV;
        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;
        
        PhoenixUtil.tryUntilOk(5, () -> elevatorMotor.getConfigurator().apply(config));

        volts = elevatorMotor.getMotorVoltage();
        pos = elevatorMotor.getPosition();
        amps = elevatorMotor.getStatorCurrent();
        vel = elevatorMotor.getVelocity();
    }

    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(volts, amps, pos, vel);
        
        inputs.voltage_v = volts.getValueAsDouble();
        inputs.current_a = amps.getValueAsDouble();
        inputs.pos_m = pos.getValueAsDouble();
        inputs.vel_mps = vel.getValueAsDouble();
    }
    
    @Override
    public void setVoltage(double voltage){
        elevatorMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public void goToPos(double pos){
        elevatorMotor.setControl(request
            .withPosition(pos));
    }

    @Override
    public void hold(double pos) {
        goToPos(pos);
    }

    @Override
    public void stop(){
        elevatorMotor.stopMotor();
    }
}
