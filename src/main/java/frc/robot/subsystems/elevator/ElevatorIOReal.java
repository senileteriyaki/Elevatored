package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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
    private final ElevatorFeedforward ff;
    private double ffVoltage;

    public ElevatorIOReal(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        MotionMagicConfigs mmConfig = new MotionMagicConfigs();

        request = new MotionMagicVoltage(ElevatorConstants.minHeight);
        ff = new ElevatorFeedforward(0, 0, 0, 0);

        config.Feedback.SensorToMechanismRatio = 1 / (2* Math.PI * ElevatorConstants.drumRadius); 
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        mmConfig.MotionMagicAcceleration = 10.0;
        mmConfig.MotionMagicCruiseVelocity = 5.0;
        config.MotionMagic = mmConfig;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kP = 10.0;
        config.Slot0.kD = 1.0;
        
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
        inputs.pos_m = pos.getValueAsDouble() * 360;
        inputs.vel_mps = vel.getValueAsDouble() * 360;
    }
    
    @Override
    public void setVoltage(double voltage){
        ffVoltage = ff.calculate(0);
        elevatorMotor.setVoltage(voltage + ffVoltage);
    }

    @Override
    public void goToPos(double pos){
        elevatorMotor.setControl(request
            .withPosition(pos)
            .withFeedForward(ffVoltage));
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
